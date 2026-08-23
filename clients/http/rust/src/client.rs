//! Asynchronous HTTP client for the Forester runtime.
//!
//! The client talks to the HTTP server embedded in a Forester instance and
//! exposes the blackboard and the tracer to remote actions. It is generated
//! from the OpenAPI specification produced by `cargo run --bin gen-openapi`
//! (see `openapi.json`).

use reqwest::{Client, Url};

use crate::error::{Error, Result};
use crate::models::{CustomEvent, RtValue};

/// A client for the Forester HTTP API.
///
/// # Examples
///
/// ```no_run
/// # async fn run() -> forester_client_http::Result<()> {
/// use forester_client_http::ForesterClient;
///
/// let client = ForesterClient::new("http://127.0.0.1:46123")?;
/// client.put("count", serde_json::json!(42)).await?;
/// assert_eq!(client.get("count").await?, Some(serde_json::json!(42)));
/// # Ok(())
/// # }
/// ```
#[derive(Debug, Clone)]
pub struct ForesterClient {
    base_url: Url,
    http: Client,
}

impl ForesterClient {
    /// Creates a new client for the given base URL of the Forester server.
    pub fn new(base_url: impl AsRef<str>) -> Result<Self> {
        Self::with_client(base_url, Client::new())
    }

    /// Creates a new client with a custom [`reqwest::Client`].
    pub fn with_client(base_url: impl AsRef<str>, http: Client) -> Result<Self> {
        let base_url =
            Url::parse(base_url.as_ref()).map_err(|e| Error::InvalidUrl(e.to_string()))?;
        Ok(Self { base_url, http })
    }

    /// Builds an absolute URL from the given path segments (each segment is
    /// percent-encoded).
    fn url(&self, segments: &[&str]) -> Url {
        let mut url = self.base_url.clone();
        // `base_url` is always a valid base URL, so `path_segments_mut` cannot
        // fail here.
        let mut paths = url
            .path_segments_mut()
            .expect("base_url is a valid base URL");
        paths.pop_if_empty().extend(segments.iter().copied());
        drop(paths);
        url
    }

    /// Sends a request and checks the response status.
    async fn send(&self, request: reqwest::RequestBuilder) -> Result<reqwest::Response> {
        let response = request.send().await?;
        if response.status().is_success() {
            Ok(response)
        } else {
            let status = response.status();
            let body = response.text().await.unwrap_or_default();
            Err(Error::Http { status, body })
        }
    }

    // -- health and metadata --------------------------------------------------

    /// Returns the health status. Always `"OK"`.
    pub async fn health(&self) -> Result<String> {
        let response = self.send(self.http.get(self.url(&[]))).await?;
        Ok(response.text().await?)
    }

    /// Returns the OpenAPI specification of the server as a JSON value.
    pub async fn openapi(&self) -> Result<serde_json::Value> {
        let response = self
            .send(self.http.get(self.url(&["openapi.json"])))
            .await?;
        Ok(response.json().await?)
    }

    // -- blackboard ------------------------------------------------------------

    /// Reads the value stored under `key` (`None` if absent).
    pub async fn get(&self, key: &str) -> Result<Option<RtValue>> {
        let response = self.send(self.http.get(self.url(&["bb", key]))).await?;
        Ok(response.json().await?)
    }

    /// Stores `value` under `key`.
    pub async fn put(&self, key: &str, value: RtValue) -> Result<()> {
        self.send(self.http.post(self.url(&["bb", key])).json(&value))
            .await?;
        Ok(())
    }

    /// Reads and removes the value stored under `key`.
    pub async fn take(&self, key: &str) -> Result<Option<RtValue>> {
        let response = self
            .send(self.http.get(self.url(&["bb", key, "take"])))
            .await?;
        Ok(response.json().await?)
    }

    /// Locks `key` so it cannot be taken.
    pub async fn lock(&self, key: &str) -> Result<()> {
        self.send(self.http.get(self.url(&["bb", key, "lock"])))
            .await?;
        Ok(())
    }

    /// Unlocks `key`.
    pub async fn unlock(&self, key: &str) -> Result<()> {
        self.send(self.http.get(self.url(&["bb", key, "unlock"])))
            .await?;
        Ok(())
    }

    /// Returns whether `key` is locked.
    pub async fn is_locked(&self, key: &str) -> Result<bool> {
        let response = self
            .send(self.http.get(self.url(&["bb", key, "locked"])))
            .await?;
        Ok(response.json().await?)
    }

    /// Returns whether `key` exists.
    pub async fn contains(&self, key: &str) -> Result<bool> {
        let response = self
            .send(self.http.get(self.url(&["bb", key, "contains"])))
            .await?;
        Ok(response.json().await?)
    }

    // -- tracer ----------------------------------------------------------------

    /// Records a custom event in the tracer.
    pub async fn trace(&self, text: &str, tick: usize) -> Result<()> {
        self.send(
            self.http
                .post(self.url(&["tracer", "custom"]))
                .json(&CustomEvent::new(text, tick)),
        )
        .await?;
        Ok(())
    }

    /// Returns the tracer content as a string.
    pub async fn print_trace(&self) -> Result<String> {
        let response = self
            .send(self.http.get(self.url(&["tracer", "print"])))
            .await?;
        Ok(response.text().await?)
    }
}
