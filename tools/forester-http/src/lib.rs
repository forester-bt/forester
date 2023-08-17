use reqwest::blocking::Response;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::time::Duration;

/// The result that the node returns
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum TickResult {
    Success,
    Failure(String),
    Running,
}

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

pub struct ForesterHttpClient {
    api: ForesterHttpApi,
    timeout: Option<Duration>,
}

impl ForesterHttpClient {
    pub fn new(base: String) -> Self {
        let api = ForesterHttpApi::new(base);
        Self { api, timeout: None }
    }
    pub fn new_with(base: String, timeout: Duration) -> Self {
        let api = ForesterHttpApi::new(base);
        Self {
            api,
            timeout: Some(timeout),
        }
    }

    pub fn post_event(&self, tick: usize, text: String) -> Result<Response, TickError> {
        Ok(self
            .client()?
            .post(&self.api.trace_event())
            .json(&CustomEvent { text, tick })
            .send()?)
    }
    pub fn print_trace(&self) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.print_trace()).send()?)
    }

    fn client(&self) -> Result<reqwest::blocking::Client, TickError> {
        match self.timeout {
            Some(timeout) => Ok(reqwest::blocking::Client::builder()
                .timeout(timeout)
                .build()?),
            None => Ok(reqwest::blocking::Client::new()),
        }
    }
}

pub struct ForesterHttpApi {
    pub base: String,
}

impl ForesterHttpApi {
    pub fn trace_event(&self) -> String {
        format!("{}/tracer/custom", self.base)
    }
    pub fn print_trace(&self) -> String {
        format!("{}/tracer/print", self.base)
    }
    pub fn lock(&self, key: String) -> String {
        format!("{}/bb/{key}/lock", self.base)
    }
    pub fn unlock(&self, key: String) -> String {
        format!("{}/bb/{key}/unlock", self.base)
    }
    pub fn locked(&self, key: String) -> String {
        format!("{}/bb/{key}/locked", self.base)
    }
    pub fn contains(&self, key: String) -> String {
        format!("{}/bb/{key}/contains", self.base)
    }
    pub fn take(&self, key: String) -> String {
        format!("{}/bb/{key}/take", self.base)
    }
    pub fn get(&self, key: String) -> String {
        format!("{}/bb/{key}", self.base)
    }
    pub fn put(&self, key: String) -> String {
        format!("{}/bb/{key}", self.base)
    }
    pub fn new(base: String) -> Self {
        Self { base }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    #[test]
    fn trace_event_then_print_event() {
        let client = ForesterHttpClient::new("http://localhost:10000".to_string());
        let resp = client.post_event(1, "test".to_string()).unwrap();
        assert_eq!(resp.status(), 200);
        let resp = client.print_trace().unwrap();
        assert_eq!(resp.status(), 200);
        let text = resp.text().unwrap();
        assert_eq!("[1]custom: test\r\n", text)
    }
}
