# Library to create Remote Actions using Rust

The Forester provides an http library that alleviates writing the remote http actions.

## Usage

```toml
forester-http = { version = "0.1.0" }
```

The contract is defined in the following way:
```rust
pub trait ForesterRemoteAction {
    fn tick(&self, request: RemoteActionRequest) -> TickResult;
}
```

where RemoteActionRequest is defined as:
```rust
pub struct RemoteActionRequest {
    /// current tick
    pub tick: usize,
    /// the list of arguments from the tree invocation
    pub args: Vec<RtArgument>,
    /// the address of the server to access to blackboard and other services
    pub serv_url: String,
}
```

On the other hand, the library provides a helper API `ForesterHttpApi` and Client `ForesterHttpClient` (async reqwest) to access the server.

## Example

The code is available in the [forester-examples](https://github.com/besok/forester-examples/tree/main/remote_action/simple_action) repository.

The gist is the following:

```rust

#[tokio::main]
async fn main() {
    let routing = Router::new()
        .route("/", get(|| async { "OK" }))
        .route("/action", post(handler))
        .into_make_service_with_connect_info::<SocketAddr>();

    axum::Server::bind(&SocketAddr::from(([127, 0, 0, 1], 10000)))
        .serve(routing)
        .await
        .unwrap();
}


/// RemoteActionRequest defines the request from the tree
async fn handler(Json(req): Json<RemoteActionRequest>) -> impl IntoResponse {
    let url = req.clone().serv_url;
    /// the client to access the server
    let client = ForesterHttpClient::new(url);
    let trace = client .print_trace(); /// print the trace of the tree

    let result = client.put("test".to_string(), json!({"f1":1, "f2":2, "f3":3})).await;
    println!("result of putting {:?}", result);
    
    client.lock("test".to_string()).await.unwrap();

    (StatusCode::OK, Json::from(RemoteAction.tick(req)))
}

struct RemoteAction;

impl ForesterRemoteAction for RemoteAction {
    fn tick(&self, request: RemoteActionRequest) -> TickResult {
        println!("tick: {:?}", request);
        TickResult::Success
    }
}


```

