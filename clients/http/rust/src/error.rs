//! Error types for the Forester HTTP client.

use reqwest::StatusCode;

/// Errors produced by the [`crate::ForesterClient`].
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// The underlying HTTP request failed (network, TLS, serialization, ...).
    #[error("request failed: {0}")]
    Request(#[from] reqwest::Error),

    /// The server answered with a non-success status code.
    #[error("the server answered with status {status}: {body}")]
    Http {
        /// The status code returned by the server.
        status: StatusCode,
        /// The error message returned in the response body.
        body: String,
    },

    /// The provided base URL is invalid.
    #[error("invalid base URL: {0}")]
    InvalidUrl(String),
}

/// Convenient alias for results produced by the client.
pub type Result<T> = std::result::Result<T, Error>;
