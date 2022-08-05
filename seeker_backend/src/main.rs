mod adapters;
mod events;
mod protos;

use events::{DebugData, EventData};
use futures::{SinkExt, StreamExt};
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;
use std::{net::SocketAddr, time::Duration};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast::{self, Receiver, Sender};
use tokio_tungstenite::{accept_async, tungstenite::Error};
use tungstenite::{Message, Result};

// TODO See if I can fix the cursed +Send+Sync thing
type Event = Arc<dyn EventData + Send + Sync>;

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
        // tokio::select! {
        //     msg = ws_receiver.next() => {
        //         match msg {
        //             Some(msg) => {
        //                 let msg = msg?;
        //                 if msg.is_text() ||msg.is_binary() {
        //                     ws_sender.send(msg).await?;
        //                 } else if msg.is_close() {
        //                     break;
        //                 }
        //             }
        //             None => break,
        //         }
        //     }
        //     _ = interval.tick() => {
        //         ws_sender.send(Message::Text("tick".to_owned())).await?;
        //     }
        // }

        tokio::select! {
            msg = ws_receiver.next() => {
                if let Some(msg) = msg {
                    println!("Received '{}' from client", msg.unwrap().to_text().unwrap_or("?"));
                }
            },
            recv = listener.recv() => {
                if let Ok(s) = recv {
                ws_sender
                    .send(Message::Text(
                        serde_json::to_string_pretty(s.as_ref()).unwrap_or_else(|e| "{}".to_owned()),
                    ))
                    .await?;
                }
            }
        }
        // while let Ok(s) = listener.recv().await {
        //     ws_sender
        //         .send(Message::Text(
        //             serde_json::to_string_pretty(s.as_ref()).unwrap_or_else(|e| "{}".to_owned()),
        //         ))
        //         .await?;
        // }
    }

    Ok(())
}

#[tokio::main]
async fn main() {
    let (client_broadcast, _) = broadcast::channel::<Event>(1);
    let (radio_send, _) = broadcast::channel::<Rc<dyn prost::Message>>(32);

    let ws_addr = "127.0.0.1:9002";
    let ws_listener = TcpListener::bind(&ws_addr).await.expect("Can't listen");
    println!("Listening on: {}", ws_addr);

    let tx = client_broadcast.clone();
    tokio::task::spawn(async move {
        while let Ok((stream, _)) = ws_listener.accept().await {
            let peer = stream
                .peer_addr()
                .expect("connected streams should have a peer address");
            println!("Peer address: {}", peer);

            tokio::spawn(accept_connection(peer, stream, tx.clone()));
        }
    });

    println!("Eeheheee");

    // broadcast_tx messages are sent to all connected websockets
    loop {
        std::thread::sleep(Duration::from_secs(1));
        client_broadcast.send(Arc::new(DebugData::with_data("test".to_owned())));
    }
}
