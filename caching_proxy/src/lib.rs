use async_trait::async_trait;
use hyper::{
    client::{connect::dns::GaiResolver, HttpConnector},
    header::HeaderName,
    http::HeaderValue,
    service::{make_service_fn, service_fn},
    Body, Client, Request, Response, Server, StatusCode,
};
use hyper_tls::HttpsConnector;
use serde::{Deserialize, Serialize};
use std::{
    collections::{hash_map::DefaultHasher, HashMap},
    convert::Infallible,
    fs::File,
    hash::{Hash, Hasher},
    net::SocketAddr,
    path::{Path, PathBuf},
    str::FromStr,
    sync::Arc,
};
use tokio::{fs, sync::Mutex};

#[macro_use]
extern crate log;

const STRIPPED: [&str; 7] = [
    "content-length",
    "transfer-encoding",
    "accept-encoding",
    "content-encoding",
    "host",
    "connection",
    "location",
];

#[async_trait]
pub trait Cache {
    async fn init(&self) -> Result<(), String>;
    async fn has_resp(&self, url: String) -> bool;
    async fn get_resp(&self, url: String) -> Result<Response<Body>, String>;
    async fn cache_resp(&self, url: String, resp: Response<Body>) -> Result<(), String>;
}

#[derive(Serialize, Deserialize, Clone)]
struct CachedRequestData {
    url: String,
    headers: HashMap<String, String>,
}

pub struct FilesystemCache {
    cache_dir: PathBuf,
}

impl FilesystemCache {
    pub fn new(path: &Path) -> Self {
        FilesystemCache {
            cache_dir: path.to_owned(),
        }
    }

    fn hash_url(url: String) -> u64 {
        let mut hasher = DefaultHasher::new();
        url.hash(&mut hasher);
        hasher.finish()
    }
}

#[async_trait]
impl Cache for FilesystemCache {
    async fn init(&self) -> Result<(), String> {
        if !self.cache_dir.is_dir() {
            fs::create_dir(&self.cache_dir)
                .await
                .map_err(|e| e.to_string())?
        }

        Ok(())
    }

    async fn has_resp(&self, url: String) -> bool {
        let resp_dir = self.cache_dir.join(Self::hash_url(url).to_string());
        resp_dir.is_dir()
    }

    async fn get_resp(&self, url: String) -> Result<Response<Body>, String> {
        let resp_dir = self
            .cache_dir
            .join(Self::hash_url(url.to_owned()).to_string());

        let metadata_dir = resp_dir.join("data.json");
        let data_dir = resp_dir.join("response.bin");

        let body = Body::from(fs::read(data_dir).await.map_err(|e| e.to_string())?);

        let meta: CachedRequestData = serde_json::from_reader(std::io::BufReader::new(
            File::open(metadata_dir).map_err(|e| e.to_string())?,
        ))
        .map_err(|e| e.to_string())?;

        let mut resp = Response::new(body);
        let hdrs = resp.headers_mut();
        for (hk, hv) in meta.headers {
            hdrs.insert(
                HeaderName::from_str(&hk).map_err(|e| e.to_string())?,
                HeaderValue::from_str(&hv).map_err(|e| e.to_string())?,
            );
        }

        Ok(resp)
    }

    async fn cache_resp(&self, url: String, resp: Response<Body>) -> Result<(), String> {
        let resp_dir = self
            .cache_dir
            .join(Self::hash_url(url.to_owned()).to_string());
        fs::create_dir(&resp_dir).await.map_err(|e| e.to_string())?;

        let metadata_dir = resp_dir.join("data.json");
        let data_dir = resp_dir.join("response.bin");

        // Store metadata
        let cachedata = CachedRequestData {
            headers: resp
                .headers()
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_str().unwrap_or("").to_owned()))
                .filter(|(k, _)| !STRIPPED.contains(&k.as_str().to_ascii_lowercase().as_str()))
                .collect(),
            url,
        };

        let meta = serde_json::to_string_pretty(&cachedata).map_err(|e| e.to_string())?;
        fs::write(metadata_dir, meta)
            .await
            .map_err(|e| e.to_string())?;

        // Store body
        let body_bytes = hyper::body::to_bytes(resp.into_body())
            .await
            .map_err(|e| e.to_string())?;

        // assert_ne!(0, body_bytes.len());

        fs::write(data_dir, body_bytes)
            .await
            .map_err(|e| e.to_string())?;

        Ok(())
    }
}

pub struct CachingProxy<C>
where
    C: Cache + Send + Sync,
{
    pub cache: C,
    pub sock_addr: SocketAddr,
    client: Client<HttpsConnector<HttpConnector<GaiResolver>>>,
}

impl<C> CachingProxy<C>
where
    C: Cache + Send + Sync + 'static,
{
    pub fn new(sock: SocketAddr, cache: C) -> Self {
        CachingProxy {
            cache,
            sock_addr: sock,
            client: Client::builder().build(HttpsConnector::new()),
        }
    }

    pub async fn start(self) {
        // Slim wrapper for the handler allowing errors to be propagates easily
        async fn handle<C>(
            req: Request<Body>,
            context: Arc<Mutex<CachingProxy<C>>>,
        ) -> Result<Response<Body>, Infallible>
        where
            C: Cache + Send + Sync,
        {
            Ok(match handle_req(req, context).await {
                Ok(body) => body,
                Err((code, msg)) => {
                    error!("handled request with error: {}, status code: {}", msg, code);
                    return Ok(Response::builder()
                        .status(code)
                        .body(Body::empty())
                        .unwrap());
                }
            })
        }

        async fn handle_req<C>(
            req: Request<Body>,
            context: Arc<Mutex<CachingProxy<C>>>,
        ) -> Result<Response<Body>, (StatusCode, String)>
        where
            C: Cache + Send + Sync,
        {
            let context = context.lock().await;
            context
                .cache
                .init()
                .await
                .expect("Error initializing cache");

            let uri = req.uri().to_string();
            let req_encoded_url = uri.split('/').last().ok_or((
                StatusCode::NOT_FOUND,
                "No request URL specified".to_string(),
            ))?;
            let req_url = urlencoding::decode(req_encoded_url)
                .map_err(|e| {
                    (
                        StatusCode::BAD_REQUEST,
                        format!("URL decoding error: {}", e),
                    )
                })?
                .into_owned();

            // Use cached response if it exists, otherwise, request it if possible
            if context.cache.has_resp(req_url.clone()).await {
                context
                    .cache
                    .get_resp(req_url)
                    .await
                    .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, e))
            } else {
                let remote_req = Request::builder()
                    .uri(req_url.clone())
                    .body(Body::empty())
                    .map_err(|e| (StatusCode::BAD_REQUEST, e.to_string()))?;

                let remote_res = context
                    .client
                    .request(remote_req)
                    .await
                    .map_err(|e| (StatusCode::NOT_FOUND, e.to_string()))?;

                context
                    .cache
                    .cache_resp(req_url.clone(), remote_res)
                    .await
                    .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, e))?;

                context
                    .cache
                    .get_resp(req_url.clone())
                    .await
                    .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, e))
            }
        }

        let context = Arc::new(Mutex::new(self));

        let ctx_lock = context.lock().await;
        let sockaddr = ctx_lock.sock_addr;
        drop(ctx_lock); // Make sure to drop the lock to prevent deadlocks

        let server = Server::bind(&sockaddr).serve(make_service_fn(move |_conn| {
            let local_ctx = context.clone();
            async move {
                Ok::<_, Infallible>(service_fn(move |req| {
                    handle(req, local_ctx.clone())
                }))
            }
        }));

        if let Err(e) = server.await {
            eprintln!("server error: {}", e)
        }
    }
}
