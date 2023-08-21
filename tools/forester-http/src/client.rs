use crate::api::ForesterHttpApi;
use reqwest::Response;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::time::Duration;

/// the client to the Forester instance of http server
/// It accepts the api and timeout to request
///
/// ```no-run
/// use forester_http::client::ForesterHttpClient;
///
/// async fn main() {
///     let client = ForesterHttpClient::new("http://localhost:10000".to_string());
///     client.new_trace_event(1, "test".to_string());
/// }
///
/// ```
pub struct ForesterHttpClient {
    api: ForesterHttpApi,
    timeout: Option<Duration>,
}

impl ForesterHttpClient {
    pub fn new(base: String) -> Self {
        let api = ForesterHttpApi::new(base);
        Self { api, timeout: None }
    }
    pub async fn new_with(base: String, timeout: Duration) -> Self {
        let api = ForesterHttpApi::new(base);
        Self {
            api,
            timeout: Some(timeout),
        }
    }

    /// creates a new trace event
    pub async fn new_trace_event(&self, tick: usize, text: String) -> Result<Response, TickError> {
        Ok(self
            .client()?
            .post(&self.api.trace_event())
            .json(&CustomEvent { text, tick })
            .send()
            .await?)
    }
    /// prints the trace or if the file is big the tail of the trace (last 100 lines)
    pub async fn print_trace(&self) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.print_trace()).send().await?)
    }
    /// lock the key in the blackboard
    pub async fn lock(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.lock(id)).send().await?)
    }
    /// unlock the key in the blackboard
    pub async fn unlock(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.unlock(id)).send().await?)
    }
    /// check if the key is locked in the blackboard
    pub async fn locked(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.locked(id)).send().await?)
    }
    /// check if the key is in the blackboard
    pub async fn contains(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.contains(id)).send().await?)
    }
    /// take the key from the blackboard.
    pub async fn take(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.take(id)).send().await?)
    }
    /// get the key from the blackboard.
    pub async fn get(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.get(id)).send().await?)
    }
    /// put the key to the blackboard.
    pub async fn put(&self, id: String, value: Value) -> Result<Response, TickError> {
        Ok(self
            .client()?
            .post(&self.api.put(id))
            .json(&value)
            .send()
            .await?)
    }

    fn client(&self) -> Result<reqwest::Client, TickError> {
        match self.timeout {
            Some(timeout) => Ok(reqwest::Client::builder().timeout(timeout).build()?),
            None => Ok(reqwest::Client::new()),
        }
    }
}

/// Error
#[derive(Debug)]
pub struct TickError {
    pub message: String,
}

impl From<reqwest::Error> for TickError {
    fn from(value: reqwest::Error) -> Self {
        Self::new(format!("{:?}", value))
    }
}

impl TickError {
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// The event that will be recorded to tracer
#[derive(Debug, Deserialize, Serialize)]
pub struct CustomEvent {
    text: String,
    tick: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::client::ForesterHttpClient;

    #[test]
    fn trace_event_then_print_event() {
        let client = ForesterHttpClient::new("http://localhost:10000".to_string());
        let resp = client.new_trace_event(1, "test".to_string()).unwrap();
        assert_eq!(resp.status(), 200);
        let resp = client.print_trace().unwrap();
        assert_eq!(resp.status(), 200);
        let text = resp.text().unwrap();
        assert_eq!("[1]custom: test\r\n", text)
    }
}
