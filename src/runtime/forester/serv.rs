mod routes;

use crate::runtime::blackboard::BlackBoard;
use crate::tracer::Tracer;
use axum::routing::{get, post};
use axum::Router;

use crate::runtime::forester::serv::routes::*;
use crate::runtime::{RtOk, RtResult, RuntimeError};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use serde::{Deserialize, Serialize};
use std::net::{IpAddr, SocketAddr};
use std::sync::{Arc, Mutex};

use crate::runtime::env::RtEnv;
use tokio::sync::oneshot::Sender;
use tokio::task::JoinHandle;
use utoipa::OpenApi;

/// The struct defines the http server that can be used to interface the remote actions.
/// By default, the server is deployed to the localhost.
/// The port is selected dynamically if it is not specified.
///
/// #Notes
/// The main purpose of the server is to provide the api for blackboard and tracer.
/// The server is started automatically if there is at least one remote action registered.
/// When forester is finished it is automatically stops the server as well.
#[derive(Clone)]
pub struct HttpServ {
    bb: Arc<Mutex<BlackBoard>>,
    tracer: Arc<Mutex<Tracer>>,
}

/// The struct defines the information of the server.
/// It is used to stop the server and get the status of the server.
pub struct ServInfo {
    pub status: JoinHandle<RtOk>,
    pub url: String,
    pub stop_cmd: StopCmd,
}

impl ServInfo {
    pub fn stop(self) -> Result<(), ()> {
        self.stop_cmd.send(())
    }
}

pub type StopCmd = Sender<()>;

/// The configuration of the http server.
/// The server can be deployed to any host and port.
///
/// #Notes
/// If the port is `0` the server selects a random available port.
#[derive(Debug, Clone, PartialEq)]
pub struct HttpServConfig {
    /// The host the server binds to. Must be a valid IP address (e.g. `127.0.0.1` or `0.0.0.0`).
    pub host: String,
    /// The port the server binds to. `0` selects a random available port.
    pub port: u16,
}

impl Default for HttpServConfig {
    fn default() -> Self {
        Self {
            host: "127.0.0.1".to_string(),
            port: 0,
        }
    }
}

impl HttpServConfig {
    pub fn new(host: String, port: u16) -> Self {
        Self { host, port }
    }
}

impl HttpServ {
    fn new(bb: Arc<Mutex<BlackBoard>>, tracer: Arc<Mutex<Tracer>>) -> Self {
        Self { bb, tracer }
    }
}

/// starts the server for access from remote actions
///
/// # Parameters
/// - `rt` - the runtime for the server. Typically can be obtained from Forester instance
/// - `config` - the host and the port for the server. If the port is `0`, the port is selected dynamically.
/// - `bb` - the blackboard that is used to store the data
/// - `tracer` - the tracer that is used to store the trace events
///
/// # Returns
/// the information of the server
pub fn start(
    rt: Arc<Mutex<RtEnv>>,
    config: HttpServConfig,
    bb: Arc<Mutex<BlackBoard>>,
    tracer: Arc<Mutex<Tracer>>,
) -> RtResult<ServInfo> {
    let (tx, rx) = tokio::sync::oneshot::channel::<()>();
    let rt = rt.lock()?;

    let ip: IpAddr = config.host.parse().map_err(|_| {
        RuntimeError::Unexpected(format!(
            "the host '{}' is not a valid IP address",
            config.host
        ))
    })?;
    let addr = SocketAddr::from((ip, config.port));

    let listener = rt
        .runtime
        .block_on(tokio::net::TcpListener::bind(addr))
        .map_err(|e| RuntimeError::IOError(format!("{:?}", e)))?;

    let url = format!("http://{}:{}", config.host, listener.local_addr()?.port());

    debug!(target:"http_server", " the server is deployed to {} ", url);

    let handle: JoinHandle<RtOk> = rt.runtime.spawn(async move {
        let service = routing(HttpServ::new(bb, tracer)).into_make_service();
        let serv_with_shutdown = axum::serve(listener, service).with_graceful_shutdown(async {
            rx.await.ok();
        });
        if let Err(e) = serv_with_shutdown.await {
            debug!(target:"http_server", "server error: {}", e);
            Err(RuntimeError::IOError(format!("{}", e)))
        } else {
            Ok(())
        }
    });

    Ok(ServInfo {
        status: handle,
        url,
        stop_cmd: tx,
    })
}
fn routing(delegate: HttpServ) -> Router {
    Router::new()
        .route("/", get(|| async { "OK" }))
        .route("/openapi.json", get(openapi_json))
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

/// Serves the OpenAPI specification of the server as JSON.
async fn openapi_json() -> axum::Json<utoipa::openapi::OpenApi> {
    axum::Json(ApiDoc::openapi())
}

/// The OpenAPI documentation of the Forester HTTP server.
///
/// It describes the endpoints that expose the blackboard and the tracer
/// to the remote actions. The specification can be obtained with
/// [`ApiDoc::openapi`] and is also served at `/openapi.json`.
#[derive(utoipa::OpenApi)]
#[openapi(
    info(
        title = "Forester HTTP API",
        version = env!("CARGO_PKG_VERSION"),
        license(name = "Apache-2.0"),
        description = "The HTTP server of the Forester behavior tree runtime. It exposes the blackboard and the tracer to the remote actions.",
    ),
    paths(
        bb_lock,
        bb_unlock,
        bb_is_locked,
        bb_contains,
        bb_get,
        bb_take,
        bb_put,
        trace,
        print_trace,
    ),
    components(schemas(CustomEvent))
)]
pub struct ApiDoc;

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

/// A custom event to record in the tracer.
#[derive(Debug, Deserialize, Serialize, utoipa::ToSchema)]
pub(crate) struct CustomEvent {
    /// The text of the event.
    text: String,
    /// The tick the event belongs to.
    tick: usize,
}
