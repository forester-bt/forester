//! Integration tests for the Forester HTTP client.

use forester_client_http::{Error, ForesterClient};
use wiremock::matchers::{body_json, method, path};
use wiremock::{Mock, MockServer, ResponseTemplate};

async fn server() -> MockServer {
    MockServer::start().await
}

#[tokio::test]
async fn health_returns_ok() {
    let server = server().await;
    Mock::given(method("GET"))
        .and(path("/"))
        .respond_with(ResponseTemplate::new(200).set_body_string("OK"))
        .mount(&server)
        .await;

    let client = ForesterClient::new(server.uri()).unwrap();
    assert_eq!(client.health().await.unwrap(), "OK");
}

#[tokio::test]
async fn openapi_returns_json() {
    let server = server().await;
    Mock::given(method("GET"))
        .and(path("/openapi.json"))
        .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!({
            "info": { "title": "Forester HTTP API" }
        })))
        .mount(&server)
        .await;

    let client = ForesterClient::new(server.uri()).unwrap();
    let spec = client.openapi().await.unwrap();
    assert_eq!(spec["info"]["title"], "Forester HTTP API");
}

#[tokio::test]
async fn get_parses_value_and_null() {
    let server = server().await;
    Mock::given(method("GET"))
        .and(path("/bb/count"))
        .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!(42)))
        .mount(&server)
        .await;
    Mock::given(method("GET"))
        .and(path("/bb/missing"))
        .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!(null)))
        .mount(&server)
        .await;

    let client = ForesterClient::new(server.uri()).unwrap();
    assert_eq!(
        client.get("count").await.unwrap(),
        Some(serde_json::json!(42))
    );
    assert_eq!(client.get("missing").await.unwrap(), None);
}

#[tokio::test]
async fn put_sends_json_body() {
    let server = server().await;
    Mock::given(method("POST"))
        .and(path("/bb/count"))
        .and(body_json(serde_json::json!(42)))
        .respond_with(ResponseTemplate::new(200))
        .mount(&server)
        .await;

    let client = ForesterClient::new(server.uri()).unwrap();
    client.put("count", serde_json::json!(42)).await.unwrap();
}

#[tokio::test]
async fn booleans_are_parsed() {
    let server = server().await;
    Mock::given(method("GET"))
        .and(path("/bb/k/contains"))
        .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!(true)))
        .mount(&server)
        .await;
    Mock::given(method("GET"))
        .and(path("/bb/k/locked"))
        .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!(false)))
        .mount(&server)
        .await;

    let client = ForesterClient::new(server.uri()).unwrap();
    assert!(client.contains("k").await.unwrap());
    assert!(!client.is_locked("k").await.unwrap());
}

#[tokio::test]
async fn trace_sends_custom_event() {
    let server = server().await;
    Mock::given(method("POST"))
        .and(path("/tracer/custom"))
        .and(body_json(serde_json::json!({ "text": "hello", "tick": 3 })))
        .respond_with(ResponseTemplate::new(200))
        .mount(&server)
        .await;

    let client = ForesterClient::new(server.uri()).unwrap();
    client.trace("hello", 3).await.unwrap();
}

#[tokio::test]
async fn print_trace_returns_text() {
    let server = server().await;
    Mock::given(method("GET"))
        .and(path("/tracer/print"))
        .respond_with(ResponseTemplate::new(200).set_body_string("trace content"))
        .mount(&server)
        .await;

    let client = ForesterClient::new(server.uri()).unwrap();
    assert_eq!(client.print_trace().await.unwrap(), "trace content");
}

#[tokio::test]
async fn error_status_is_reported() {
    let server = server().await;
    Mock::given(method("GET"))
        .and(path("/bb/k"))
        .respond_with(ResponseTemplate::new(500).set_body_string("boom"))
        .mount(&server)
        .await;

    let client = ForesterClient::new(server.uri()).unwrap();
    match client.get("k").await {
        Err(Error::Http { status, body }) => {
            assert_eq!(status, 500);
            assert_eq!(body, "boom");
        }
        other => panic!("expected Error::Http, got {:?}", other.map(|v| v.is_some())),
    }
}

#[tokio::test]
async fn keys_are_percent_encoded() {
    let server = server().await;
    Mock::given(method("GET"))
        .and(path("/bb/my%20key"))
        .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!(1)))
        .mount(&server)
        .await;

    let client = ForesterClient::new(server.uri()).unwrap();
    assert_eq!(
        client.get("my key").await.unwrap(),
        Some(serde_json::json!(1))
    );
}
