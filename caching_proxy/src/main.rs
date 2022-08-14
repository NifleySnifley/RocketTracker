use std::path::Path;

use caching_proxy::{CachingProxy, FilesystemCache};

#[tokio::main]
async fn main() {
    env_logger::init();
    let cache = FilesystemCache::new(Path::new("./data"));
    let proxy = CachingProxy::new("127.0.0.1:8080".parse().unwrap(), cache);

    // Start the proxy server
    println!("Proxy running on http://{}", proxy.sock_addr);
    proxy.start().await;
}
