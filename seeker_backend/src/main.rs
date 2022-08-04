mod adapters;
mod protos;

use tokio::{self, fs, sync};
use warp::{self, Filter};

#[tokio::main]
async fn main() {
    //     let webserver = tokio::spawn(async {
    //         warp::serve(warp::fs::dir("../seeker_frontend/build"))
    //             .run(([127, 0, 0, 1], 3000))
    //             .await;
    //     });

    //     println!("Serving frontend at http:/localhost:3000/");
    //     webserver.await.unwrap();
}
