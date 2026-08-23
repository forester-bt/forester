# Forester HTTP client (Rust)

Asynchronous HTTP client for the [Forester](https://github.com/besok/forester)
behavior tree runtime. Forester embeds a small HTTP server that exposes the
**blackboard** and the **tracer** to remote actions; this crate talks to that
server.

The client is generated from the OpenAPI specification produced by the
Forester `gen-openapi` binary (`cargo run --bin gen-openapi`). The source
specification is kept in `clients/http/openapi.json`.

## Installation

```toml
[dependencies]
forester-client-http = "0.6"
```

## Usage

```rust
use forester_client_http::ForesterClient;

#[tokio::main]
async fn main() -> forester_client_http::Result<()> {
    let client = ForesterClient::new("http://127.0.0.1:46123")?;

    // health
    assert_eq!(client.health().await?, "OK");

    // blackboard
    client.put("count", serde_json::json!(42)).await?;
    assert_eq!(client.get("count").await?, Some(serde_json::json!(42)));
    assert!(client.contains("count").await?);

    let taken = client.take("count").await?; // Some(42), key removed

    client.put("config", serde_json::json!({"timeout": 5})).await?;
    client.lock("config").await?;
    assert!(client.is_locked("config").await?);
    client.unlock("config").await?;

    // tracer
    client.trace("robot armed", 3).await?;
    println!("{}", client.print_trace().await?);

    Ok(())
}
```

## API

| Method | Endpoint |
| ------ | -------- |
| `health()` | `GET /` |
| `openapi()` | `GET /openapi.json` |
| `get(key)` | `GET /bb/{key}` |
| `put(key, value)` | `POST /bb/{key}` |
| `take(key)` | `GET /bb/{key}/take` |
| `lock(key)` | `GET /bb/{key}/lock` |
| `unlock(key)` | `GET /bb/{key}/unlock` |
| `is_locked(key)` | `GET /bb/{key}/locked` |
| `contains(key)` | `GET /bb/{key}/contains` |
| `trace(text, tick)` | `POST /tracer/custom` |
| `print_trace()` | `GET /tracer/print` |

All methods are async and require a Tokio runtime.

## License

Apache-2.0
