//! # forester-client-http
//!
//! Asynchronous HTTP client for the [Forester](https://github.com/besok/forester)
//! behavior tree runtime. Forester embeds a small HTTP server that exposes the
//! **blackboard** and the **tracer** to remote actions; this crate talks to
//! that server.
//!
//! The client is generated from the OpenAPI specification produced by the
//! Forester `gen-openapi` binary (`cargo run --bin gen-openapi`). The source
//! specification is kept in `clients/http/openapi.json`.
//!
//! # Example
//!
//! ```no_run
//! use forester_client_http::ForesterClient;
//!
//! # async fn run() -> forester_client_http::Result<()> {
//! let client = ForesterClient::new("http://127.0.0.1:46123")?;
//!
//! client.put("count", serde_json::json!(42)).await?;
//! assert_eq!(client.get("count").await?, Some(serde_json::json!(42)));
//!
//! client.trace("robot armed", 3).await?;
//! println!("{}", client.print_trace().await?);
//! # Ok(())
//! # }
//! ```

mod client;
mod error;
mod models;
pub mod ra;

pub use client::ForesterClient;
pub use error::{Error, Result};
pub use models::{CustomEvent, RtValue};
pub use ra::{RemoteActionRequest, RtArgument, TickResult};
