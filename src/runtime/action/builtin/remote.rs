use crate::runtime::action::{Impl, ImplRemote, Tick};
use crate::runtime::args::{RtArgs, RtArgument};
use crate::runtime::context::{TreeContextRef, TreeRemoteContextRef};
use crate::runtime::{to_fail, RuntimeError, RuntimeError::*, TickResult};
use hyper::body::HttpBody;
use hyper::client::HttpConnector;
use hyper::{body, Body, Client, Method, Request};
use serde::{Deserialize, Serialize};

/// The struct defines the remote action that can be executed on the remote server.
///
/// #Notes
/// The connection is confined by the timeout of 30 seconds by default
/// The error is recovered as a failure and returns the Failure result
///
/// #Examples
/// ```
///
/// use std::path::PathBuf;
/// use std::time::Duration;
/// use forester_rs::runtime::action::builtin::remote::RemoteHttpAction;
/// use forester_rs::runtime::builder::ForesterBuilder;
///
/// fn build_forester(root:PathBuf){
///      let mut fb = ForesterBuilder::from_fs();
///      fb.main_file("main.tree".to_string());
///      fb.root(root);
///      
///      let action = RemoteHttpAction::new("http://localhost:10000".to_string());
///      let action_with_url = RemoteHttpAction::new_with("http://localhost:10001".to_string(), "http://127.0.0.1".to_string());
///      fb.register_remote_action("a", action);
///      fb.register_remote_action("b", action_with_url);
///     
///      let mut f = fb.build().unwrap();
/// }
/// ```
///
#[derive(Debug, Clone)]
pub struct RemoteHttpAction {
    url: String,
    serv_ip: Option<String>,
}

impl ImplRemote for RemoteHttpAction {
    fn tick(&self, args: RtArgs, ctx: TreeRemoteContextRef) -> Tick {
        let serv_url = self
            .serv_ip
            .clone()
            .unwrap_or("http://localhost".to_string());
        let request = RemoteActionRequest {
            tick: ctx.curr_ts,
            args: args.0,
            serv_url: format!("{}:{}", serv_url, ctx.port),
        };

        debug!(target:"remote_action", "remote request {:?} to {}",&request, &self.url.clone());

        let resp = ctx.env.runtime.block_on(async {
            let client: Client<HttpConnector, Body> =
                hyper::Client::builder().build(HttpConnector::new());
            /// todo with vec is slow. Bytes?
            let body_js = serde_json::to_vec(&request).unwrap();

            let request = Request::builder()
                .method(Method::POST)
                .header("Content-Type", "application/json")
                .uri(self.url.clone())
                .body(Body::from(body_js))
                .expect("flawless request");

            match to_fail(client.request(request).await) {
                Ok(r) => to_fail(body::to_bytes(r.into_body()).await)
                    .and_then(|bytes| to_fail(serde_json::from_slice::<TickResult>(&bytes))),
                Err(e) => Err(e),
            }
        });

        debug!(target:"http_serv_proxy", "remote_action: {:?}", resp);

        resp
    }
}

impl RemoteHttpAction {
    /// Create a new remote action with the url
    /// #Examples
    /// ```no_run
    /// use forester_rs::runtime::action::builtin::remote::RemoteHttpAction;
    /// let action = RemoteHttpAction::new("http://localhost:10000".to_string());
    /// ```
    ///
    /// #Notes
    /// The server ip is not set, the default is localhost
    pub fn new(url: String) -> Self {
        Self { url, serv_ip: None }
    }

    /// Create a new remote action with the url and the server ip
    pub fn new_with(url: String, serv_ip: String) -> Self {
        Self {
            url,
            serv_ip: Some(serv_ip),
        }
    }
}

/// The struct defines the remote action request that can be executed on the remote server.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RemoteActionRequest {
    /// The tick of the request
    pub tick: usize,
    /// The arguments of the request
    pub args: Vec<RtArgument>,
    /// The server url to get access to the blackboard and the tracer
    pub serv_url: String,
}
