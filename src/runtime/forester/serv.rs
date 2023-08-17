mod routes;
mod tests;

use crate::runtime::blackboard::BlackBoard;
use crate::runtime::builder::ServerPort;
use crate::tracer::{Event, Tracer};
use axum::routing::{get, post};
use axum::{Json, Router};

use crate::runtime::forester::serv::routes::*;
use crate::runtime::{RtOk, RtResult, RuntimeError};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use hyper::server::conn::AddrIncoming;
use hyper::server::Builder;
use serde::{Deserialize, Serialize};
use std::net::SocketAddr;
use std::sync::{Arc, LockResult, Mutex};
use tokio::runtime::Runtime;
use tokio::sync::oneshot::Sender;
use tokio::task::JoinHandle;

/// The struct defines the http server that can be used to interface the remote actions.
/// By default, the server is deployed to the localhost.
/// The port is selected dynamically if it is not specified.
///
/// #Notes
/// The main purpose of the server is to provide the api for blackboard and tracer.
/// The server is started automatically if there is at least one remote action registered.
#[derive(Clone)]
pub struct HttpServ {
    bb: Arc<Mutex<BlackBoard>>,
    tracer: Arc<Mutex<Tracer>>,
}

/// The struct defines the information of the server.
/// It is used to stop the server and get the status of the server.
pub struct ServInfo {
    pub status: JoinHandle<RtOk>,
    pub stop_cmd: StopCmd,
}

impl ServInfo {
    pub fn stop(mut self) -> Result<(), ()> {
        self.stop_cmd.send(())
    }
}

pub type StopCmd = Sender<()>;

impl HttpServ {
    fn new(bb: Arc<Mutex<BlackBoard>>, tracer: Arc<Mutex<Tracer>>) -> Self {
        Self { bb, tracer }
    }
}

/// starts the server for access from remote actions
///
/// # Parameters
/// - `rt` - the runtime for the server. Typically can be obtained from Forester instance
/// - `port` - the port for the server. If it is not specified, the port is selected dynamically.
/// - `bb` - the blackboard that is used to store the data
/// - `tracer` - the tracer that is used to store the trace events
///
/// # Returns
/// the information of the server
pub fn start(
    rt: &Runtime,
    port: ServerPort,
    bb: Arc<Mutex<BlackBoard>>,
    tracer: Arc<Mutex<Tracer>>,
) -> RtResult<ServInfo> {
    let (tx, rx) = tokio::sync::oneshot::channel::<()>();

    let handle: JoinHandle<RtOk> = rt.spawn(async {
        match bind(port) {
            Ok(builder) => {
                let service = routing(HttpServ::new(bb, tracer)).into_make_service();
                let server = builder.serve(service);

                debug!(target:"http_server", " the server is deployed to {} ",server.local_addr().port());
                let serv_with_shutdown = server.with_graceful_shutdown(async {
                    rx.await.ok();
                });
                if let Err(e) = serv_with_shutdown.await {
                    debug!(target:"http_server", "server error: {}", e);
                    Err(RuntimeError::MultiThreadError(format!("{}", e.to_string())))
                } else {
                    Ok(())
                }
            }
            Err(e) => {
                debug!(target:"http_server", "server error: {:?}", e);
                Err(RuntimeError::MultiThreadError(format!("{:?}", e)))
            }
        }
    });

    Ok(ServInfo {
        status: handle,
        stop_cmd: tx,
    })
}
fn bind(port: ServerPort) -> Result<Builder<AddrIncoming>, RuntimeError> {
    match port {
        ServerPort::None => Err(RuntimeError::Unexpected(
            "the port for http server is not selected.".to_string(),
        )),
        ServerPort::Dynamic => axum::Server::try_bind(&SocketAddr::from(([127, 0, 0, 1], 0)))
            .map_err(|e| RuntimeError::IOError(e.to_string())),
        ServerPort::Static(port) => {
            axum::Server::try_bind(&SocketAddr::from(([127, 0, 0, 1], port)))
                .map_err(|e| RuntimeError::IOError(e.to_string()))
        }
    }
}

fn routing(delegate: HttpServ) -> Router {
    Router::new()
        .route("/", get(|| async { "OK" }))
        .route("/tracer/custom", post(trace))
        .route("/tracer/print", get(print_trace))
        .route("/bb/:key/lock", get(bb_lock))
        .route("/bb/:key/unlock", get(bb_unlock))
        .route("/bb/:key/locked", get(bb_is_locked))
        .route("/bb/:key/contains", get(bb_contains))
        .route("/bb/:key/take", get(bb_take))
        .route("/bb/:key", post(bb_put))
        .route("/bb/:key", get(bb_get))
        .with_state(delegate)
}

fn err_handler<R>(r: RtResult<R>) -> Response
where
    R: IntoResponse,
{
    match r {
        Ok(r) => r.into_response(),
        Err(e) => {
            let err_str = format!("{:?}", e);
            debug!(target: "http_server", "internal error: {}",err_str);
            (StatusCode::INTERNAL_SERVER_ERROR, err_str).into_response()
        }
    }
}

#[derive(Debug, Deserialize, Serialize)]
pub(crate) struct CustomEvent {
    text: String,
    tick: usize,
}
