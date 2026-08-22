use crate::runtime::args::RtValue;
use crate::runtime::forester::serv::{err_handler, CustomEvent, HttpServ};
use crate::runtime::RuntimeError;
use crate::tracer::Event;
use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::response::Response;
use axum::Json;

/// Lock a blackboard key.
///
/// Locks the key so that it cannot be taken by other remote actions.
#[utoipa::path(
    get,
    path = "/bb/{key}/lock",
    params(("key" = String, Path, description = "The blackboard key to lock")),
    responses(
        (status = 200, description = "The key was locked"),
        (status = 500, description = "Internal server error")
    )
)]
pub(crate) async fn bb_lock(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(Into::<RuntimeError>::into)
            .and_then(|mut bb| bb.lock(key))
            .map(|_| StatusCode::OK),
    )
}

/// Get a value from the blackboard.
///
/// Returns the value stored under the key, or `null` if the key is absent.
#[utoipa::path(
    get,
    path = "/bb/{key}",
    params(("key" = String, Path, description = "The blackboard key to read")),
    responses(
        (status = 200, description = "The value stored under the key, or null if absent", body = Option<RtValue>),
        (status = 500, description = "Internal server error")
    )
)]
pub(crate) async fn bb_get(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(Into::<RuntimeError>::into)
            .and_then(|bb| bb.get(key).map(|v| v.cloned()))
            .map(|r| (StatusCode::OK, Json::from(r))),
    )
}

/// Take a value from the blackboard.
///
/// Returns the value stored under the key and removes it from the blackboard.
#[utoipa::path(
    get,
    path = "/bb/{key}/take",
    params(("key" = String, Path, description = "The blackboard key to take")),
    responses(
        (status = 200, description = "The taken value, or null if absent", body = Option<RtValue>),
        (status = 500, description = "Internal server error")
    )
)]
pub(crate) async fn bb_take(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(Into::<RuntimeError>::into)
            .and_then(|mut bb| bb.take(key))
            .map(|r| (StatusCode::OK, Json::from(r))),
    )
}

/// Unlock a blackboard key.
#[utoipa::path(
    get,
    path = "/bb/{key}/unlock",
    params(("key" = String, Path, description = "The blackboard key to unlock")),
    responses(
        (status = 200, description = "The key was unlocked"),
        (status = 500, description = "Internal server error")
    )
)]
pub(crate) async fn bb_unlock(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(Into::<RuntimeError>::into)
            .and_then(|mut bb| bb.unlock(key))
            .map(|_| StatusCode::OK),
    )
}

/// Check whether a blackboard key is locked.
#[utoipa::path(
    get,
    path = "/bb/{key}/locked",
    params(("key" = String, Path, description = "The blackboard key to check")),
    responses(
        (status = 200, description = "Whether the key is locked", body = bool),
        (status = 500, description = "Internal server error")
    )
)]
pub(crate) async fn bb_is_locked(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(Into::<RuntimeError>::into)
            .and_then(|mut bb| bb.is_locked(key))
            .map(|b| (StatusCode::OK, Json::from(b))),
    )
}

/// Check whether a blackboard key exists.
#[utoipa::path(
    get,
    path = "/bb/{key}/contains",
    params(("key" = String, Path, description = "The blackboard key to check")),
    responses(
        (status = 200, description = "Whether the key exists", body = bool),
        (status = 500, description = "Internal server error")
    )
)]
pub(crate) async fn bb_contains(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(Into::<RuntimeError>::into)
            .and_then(|bb| bb.contains(key))
            .map(|b| (StatusCode::OK, Json::from(b))),
    )
}

/// Store a value in the blackboard.
///
/// The body is a free-form JSON value (`string`, `number`, `boolean`, `array`
/// or `object`) that is stored under the key.
#[utoipa::path(
    post,
    path = "/bb/{key}",
    params(("key" = String, Path, description = "The blackboard key to write")),
    request_body = RtValue,
    responses(
        (status = 200, description = "The value was stored"),
        (status = 500, description = "Internal server error")
    )
)]
pub(crate) async fn bb_put(
    Path(key): Path<String>,
    State(s): State<HttpServ>,
    Json(v): Json<RtValue>,
) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(Into::<RuntimeError>::into)
            .and_then(|mut bb| bb.put(key, v))
            .map(|_| StatusCode::OK),
    )
}

/// Record a custom event in the tracer.
#[utoipa::path(
    post,
    path = "/tracer/custom",
    request_body = CustomEvent,
    responses(
        (status = 200, description = "The event was recorded"),
        (status = 500, description = "Internal server error")
    )
)]
pub(crate) async fn trace(State(s): State<HttpServ>, Json(event): Json<CustomEvent>) -> Response {
    err_handler(
        s.tracer
            .lock()
            .map_err(Into::<RuntimeError>::into)
            .and_then(|mut t| t.trace(event.tick, Event::Custom(event.text)))
            .map(|_| StatusCode::OK),
    )
}

/// Print the tracer content.
#[utoipa::path(
    get,
    path = "/tracer/print",
    responses(
        (status = 200, description = "The tracer content as a string", body = String),
        (status = 500, description = "Internal server error")
    )
)]
pub(crate) async fn print_trace(State(s): State<HttpServ>) -> Response {
    err_handler(
        s.tracer
            .lock()
            .map_err(Into::<RuntimeError>::into)
            .map(|t| t.to_string())
            .map(|s| (StatusCode::OK, s)),
    )
}
