use crate::api::ForesterHttpApi;
use reqwest::blocking::Response;
use serde::{Deserialize, Serialize};
use std::time::Duration;

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

    pub fn new_trace_event(&self, tick: usize, text: String) -> Result<Response, TickError> {
        Ok(self
            .client()?
            .post(&self.api.trace_event())
            .json(&CustomEvent { text, tick })
            .send()?)
    }
    pub fn print_trace(&self) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.print_trace()).send()?)
    }
    pub fn lock(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.lock(id)).send()?)
    }
    pub fn unlock(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.unlock(id)).send()?)
    }
    pub fn locked(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.locked(id)).send()?)
    }
    pub fn contains(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.contains(id)).send()?)
    }
    pub fn take(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.take(id)).send()?)
    }
    pub fn get(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.get(&self.api.get(id)).send()?)
    }
    pub fn put(&self, id: String) -> Result<Response, TickError> {
        Ok(self.client()?.post(&self.api.unlock(id)).send()?)
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
