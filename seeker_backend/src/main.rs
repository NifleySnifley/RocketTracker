mod adapters;
mod endpoints;
mod events;
mod protos;
mod tracker;
mod websocket;

use adapters::Packet;
use caching_proxy::{CachingProxy, FilesystemCache};
use endpoints::{create_empty_flight, get_flights, FlightLogMetadata};
use events::{DebugData, Event};
use futures::{SinkExt, StreamExt};
use std::fs;
use std::path::Path;
use std::{net::SocketAddr, time::Duration};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast::{self, Receiver, Sender};
use tokio_tungstenite::{accept_async, tungstenite::Error};
use tungstenite::{Message, Result};
use warp::reply::json;
use warp::Filter;

// TODO See if I can fix the cursed +Send+Sync thing
// type Event = Arc<dyn WriteableEventData + Send + Sync + typetag>;

async fn accept_connection(peer: SocketAddr, stream: TcpStream, broadcast: Sender<Event>) {
    if let Err(e) = handle_connection(peer, stream, &broadcast).await {
        match e {
            Error::ConnectionClosed | Error::Protocol(_) | Error::Utf8 => (),
            err => eprintln!("Error processing connection: {}", err),
        }
    }
}

// Handles the connection to a single frontend client (websocket)
async fn handle_connection(
    peer: SocketAddr,
    stream: TcpStream,
    broadcast: &Sender<Event>,
) -> Result<()> {
    let ws_stream = accept_async(stream).await.expect("Failed to accept");
    println!("New WebSocket connection: {}", peer);
    let (mut ws_sender, mut ws_receiver) = ws_stream.split();
    // Echo incoming WebSocket messages and send a message periodically every second.

    let mut listener = broadcast.subscribe();

    loop {
        tokio::select! {
            msg = ws_receiver.next() => {
                if let Some(msg) = msg {
                    println!("Received '{}' from client", msg.unwrap().to_text().unwrap_or("?"));
                } else {
                    println!("Disconnecting from websocket...");
                    break;
                }
            },
            recv = listener.recv() => {
                if let Ok(s) = recv {
                ws_sender
                    .send(Message::Text(
                        serde_json::to_string_pretty(&s).unwrap_or_else(|_e| "{}".to_owned()),
                    ))
                    .await?;
                }
            }
        }
    }

    Ok(())
}

#[tokio::main]
async fn main() {
    let (client_broadcast, _) = broadcast::channel::<Event>(1);
    let (radio_send, _) = broadcast::channel::<Packet>(32);

    let ws_addr = "127.0.0.1:9002";
    let ws_listener = TcpListener::bind(&ws_addr).await.expect("Can't listen");
    println!("Listening on: {}", ws_addr);

    let tx = client_broadcast.clone();
    tokio::task::spawn(async move {
        while let Ok((stream, _)) = ws_listener.accept().await {
            let peer = stream
                .peer_addr()
                .expect("connected streams should have a peer address");
            // println!("Peer address: {}", peer);

            tokio::spawn(accept_connection(peer, stream, tx.clone()));
        }
    });

    tokio::task::spawn(async move {
        let cache = FilesystemCache::new(Path::new("./data"));
        let proxy = CachingProxy::new("127.0.0.1:8080".parse().unwrap(), cache);
        println!("Starting proxy on http://{}", proxy.sock_addr);

        // Start server (consumes object)
        proxy.start().await;
    });

    // broadcast_tx messages are sent to all connected websockets
    // this loop runs to send global messages, events, etc.
    // In the future it may handle receiving from the tracker
    tokio::task::spawn(async move {
        loop {
            tokio::time::sleep(Duration::from_secs(1)).await;
            client_broadcast
                .send(Event::Debug(DebugData::with_data("test".to_owned())))
                .unwrap_or(0);
        }
    });

    let flight_data_dir = Path::new("./flight_data");
    if !flight_data_dir.is_dir() {
        fs::create_dir(flight_data_dir).unwrap();
    }

    // Start the REST API for flight data
    // TODO Add subdir for API, subdir for frontend
    let rest_sockaddr: SocketAddr = ([127, 0, 0, 1], 3030).into();
    let get_flight_datas =
        warp::path("flights").map(|| json(&get_flights(flight_data_dir).unwrap_or_default()));
    let new_flight = warp::path!("flights" / "new").map(|| {
        let new_data = FlightLogMetadata::default();
        create_empty_flight(flight_data_dir, &new_data).unwrap();
        json(&new_data.id.to_string())
    });

    let routes = warp::get().and(new_flight).or(get_flight_datas);
    tokio::task::spawn(async move {
        println!("Starting REST flights API on http://{}", rest_sockaddr);
        warp::serve(routes).run(rest_sockaddr).await;
    });

    loop {
        // HACK Just let the thread wait indefinitely
        // TODO Do something in here...
        futures::future::pending::<()>().await;
    }
}
