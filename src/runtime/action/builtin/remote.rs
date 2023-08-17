use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::context::TreeContextRef;
use crate::runtime::{RuntimeError, TickResult};
use reqwest::blocking::Response;
use serde_json::json;

/// It is marker trait for remote actions that can be executed on the remote server.
/// The remote server can be a local process or a remote server, using http protocol.
pub trait Remote {}

/// The struct defines the remote action that can be executed on the remote server.
///
/// #Notes
/// The connection is confined by the timeout of 30 seconds.
pub struct RemoteHttpAction {
    url: String,
}

impl Remote for RemoteHttpAction {}

impl Impl for RemoteHttpAction {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        debug!(target:"remote_action", "remote request to {}",&self.url);

        Ok(reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(30))
            .build()?
            .post(&self.url)
            .json(&args)
            .send()?
            .json::<TickResult>()?)
    }
}

impl RemoteHttpAction {
    pub fn new(url: String) -> Self {
        Self { url }
    }
}
