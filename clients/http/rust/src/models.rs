//! Data models for the Forester HTTP API.
//!
//! The models mirror the OpenAPI specification served by the Forester HTTP
//! server (see `openapi.json`). The blackboard exchanges a free-form JSON
//! value ([`RtValue`]), while the tracer records [`CustomEvent`]s.

use serde::{Deserialize, Serialize};

/// A free-form value stored in the blackboard: string, number, boolean,
/// array or object.
pub type RtValue = serde_json::Value;

/// A custom event to record in the tracer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CustomEvent {
    /// The text of the event.
    pub text: String,
    /// The tick the event belongs to.
    pub tick: usize,
}

impl CustomEvent {
    /// Creates a new custom event.
    pub fn new(text: impl Into<String>, tick: usize) -> Self {
        Self {
            text: text.into(),
            tick,
        }
    }
}
