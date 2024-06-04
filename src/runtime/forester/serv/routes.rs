
use crate::runtime::args::RtValue;
use crate::runtime::forester::serv::{err_handler, CustomEvent, HttpServ};
use crate::runtime::RuntimeError;
use crate::tracer::Event;
use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::response::Response;
use axum::Json;

pub(crate) async fn bb_lock(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(|e| Into::<RuntimeError>::into(e))
            .and_then(|mut bb| bb.lock(key))
            .map(|_| StatusCode::OK),
    )
}
pub(crate) async fn bb_get(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(|e| Into::<RuntimeError>::into(e))
            .and_then(|bb| bb.get(key).map(|v| v.cloned()))
            .map(|r| (StatusCode::OK, Json::from(r))),
    )
}
pub(crate) async fn bb_take(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(|e| Into::<RuntimeError>::into(e))
            .and_then(|mut bb| bb.take(key))
            .map(|r| (StatusCode::OK, Json::from(r))),
    )
}
pub(crate) async fn bb_unlock(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(|e| Into::<RuntimeError>::into(e))
            .and_then(|mut bb| bb.unlock(key))
            .map(|_| StatusCode::OK),
    )
}
pub(crate) async fn bb_is_locked(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(|e| Into::<RuntimeError>::into(e))
            .and_then(|mut bb| bb.is_locked(key))
            .map(|b| (StatusCode::OK, Json::from(b))),
    )
}
pub(crate) async fn bb_contains(Path(key): Path<String>, State(s): State<HttpServ>) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(|e| Into::<RuntimeError>::into(e))
            .and_then(|bb| bb.contains(key))
            .map(|b| (StatusCode::OK, Json::from(b))),
    )
}
pub(crate) async fn bb_put(
    Path(key): Path<String>,
    State(s): State<HttpServ>,
    Json(v): Json<RtValue>,
) -> Response {
    err_handler(
        s.bb.lock()
            .map_err(|e| Into::<RuntimeError>::into(e))
            .and_then(|mut bb| bb.put(key, v))
            .map(|_| StatusCode::OK),
    )
}

pub(crate) async fn trace(State(s): State<HttpServ>, Json(event): Json<CustomEvent>) -> Response {
    err_handler(
        s.tracer
            .lock()
            .map_err(|e| Into::<RuntimeError>::into(e))
            .and_then(|mut t| t.trace(event.tick, Event::Custom(event.text)))
            .map(|_| StatusCode::OK),
    )
}
pub(crate) async fn print_trace(State(s): State<HttpServ>) -> Response {
    err_handler(
        s.tracer
            .lock()
            .map_err(|e| Into::<RuntimeError>::into(e))
            .map(|t| t.to_string())
            .map(|s| (StatusCode::OK, s)),
    )
}
