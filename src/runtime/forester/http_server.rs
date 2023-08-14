use crate::runtime::blackboard::BlackBoard;
use crate::runtime::builder::ServerPort;
use crate::tracer::Tracer;
use axum::response::Html;
use axum::routing::get;
use axum::Router;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

pub struct HttpServer {
    port: ServerPort,
    bb: Arc<Mutex<BlackBoard>>,
    tracer: Arc<Mutex<Tracer>>,
}

impl HttpServer {
    pub fn new(port: ServerPort, bb: Arc<Mutex<BlackBoard>>, tracer: Arc<Mutex<Tracer>>) -> Self {
        Self { port, bb, tracer }
    }

    pub async fn start(&self) {
        tracing_subscriber::registry()
            .with(
                tracing_subscriber::EnvFilter::try_from_default_env()
                    .unwrap_or_else(|_| "forester instance".into()),
            )
            .with(tracing_subscriber::fmt::layer())
            .init();

        let app = Router::new().route("/", get(handler));

        // run it
        let addr = SocketAddr::from(([127, 0, 0, 1], 3000));

        axum::Server::bind(&addr)
            .serve(app.into_make_service())
            .await
            .unwrap();
    }
}
async fn handler() -> Html<&'static str> {
    Html("<h1>Hello, World!</h1>")
}
