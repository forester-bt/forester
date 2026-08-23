//! Wire models for the Forester remote action (RA) protocol.
//!
//! These types are not part of the blackboard/tracer HTTP server; they
//! describe the request that the Forester runtime POSTs to a remote action
//! ([`RemoteActionRequest`]) and the [`TickResult`] that the remote action
//! must respond with.
//!
//! They mirror the serde (externally tagged) representation of the Rust
//! enums/structs in the `forester-rs` crate so a remote action can
//! deserialize the incoming body and serialize its answer back.

use serde::{Deserialize, Serialize};

use crate::RtValue;

/// A single named argument sent to the remote action.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RtArgument {
    /// The argument name.
    pub name: String,
    /// The argument value (free-form JSON).
    pub value: RtValue,
}

impl RtArgument {
    /// Creates a new argument.
    pub fn new(name: impl Into<String>, value: RtValue) -> Self {
        Self {
            name: name.into(),
            value,
        }
    }
}

/// The request that the Forester runtime POSTs to a remote action.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RemoteActionRequest {
    /// The current tick of the tree.
    pub tick: usize,
    /// The arguments passed to the remote action.
    pub args: Vec<RtArgument>,
    /// The URL of the embedded Forester HTTP server exposing the blackboard
    /// and the tracer.
    pub serv_url: String,
}

/// The result that a remote action must respond with.
///
/// Serializes to the same wire format as the runtime's `TickResult`:
/// `"Success"`, `"Running"` or `{"Failure": "reason"}`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TickResult {
    Success,
    Failure(String),
    Running,
}

impl TickResult {
    /// A successful result.
    pub fn success() -> Self {
        TickResult::Success
    }

    /// A still-running result.
    pub fn running() -> Self {
        TickResult::Running
    }

    /// A failed result with a reason.
    pub fn failure(reason: impl Into<String>) -> Self {
        TickResult::Failure(reason.into())
    }
}

#[cfg(test)]
mod tests {
    use super::{RemoteActionRequest, RtArgument, TickResult};

    #[test]
    fn tick_result_success_is_a_string() {
        assert_eq!(
            serde_json::to_string(&TickResult::success()).unwrap(),
            "\"Success\""
        );
    }

    #[test]
    fn tick_result_running_is_a_string() {
        assert_eq!(
            serde_json::to_string(&TickResult::running()).unwrap(),
            "\"Running\""
        );
    }

    #[test]
    fn tick_result_failure_is_an_object() {
        assert_eq!(
            serde_json::to_string(&TickResult::failure("boom")).unwrap(),
            "{\"Failure\":\"boom\"}"
        );
    }

    #[test]
    fn request_roundtrips() {
        let request = RemoteActionRequest {
            tick: 1,
            args: vec![
                RtArgument::new("a", serde_json::json!(1)),
                RtArgument::new("b", serde_json::json!([1, 2])),
            ],
            serv_url: "http://127.0.0.1:46123".to_string(),
        };

        let json = serde_json::to_value(&request).unwrap();
        let back: RemoteActionRequest = serde_json::from_value(json).unwrap();
        assert_eq!(back, request);
    }
}
