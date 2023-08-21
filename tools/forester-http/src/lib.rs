//! # Forester HTTP
//! The library provides a contract to implement a remote action alongside with the  
//! API to the Forester instance of http server.
//! It is used to get access to the blackboard and to trace events.
//!
//! The library composes three main parts:
//! * `ForesterRemoteAction` - the contract for the remote action that is expected by the Forester instance
//! * `ForesterHttpApi` - the api to Forester instance of http server
//! * `ForesterHttpClient` - the client to the Forester instance of http server
//!
//! Client uses api to get access to the blackboard and to trace events.
//! Under the hood it uses reqwest(blocking) to send requests to the Forester instance.
//! If the library is willing to use another client it can use only api.
//!
pub mod api;
pub mod client;

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// The result that the node returns
/// It can be Success, Failure or Running
/// The Failure contains the error message
/// The Running means that the node is still running
/// and it will be executed on the next tick
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum TickResult {
    Success,
    Failure(String),
    Running,
}

/// The request that is sent from the Forester instance
/// It has the current tick and the arguments in the action from tree
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RemoteActionRequest {
    pub tick: usize,
    pub args: Vec<RtArgument>,
    pub serv_url: String,
}

/// The argument that is sent from the Forester instance
#[derive(Debug, PartialEq, Clone, Deserialize, Serialize)]
pub struct RtArgument {
    name: String,
    value: Value,
}

/// Describes the contract for the remote action that is expected by the Forester instance
/// The remote action is a remote implementation of the action node in the tree.
///
/// Therefore the implementation of the remote action should be integrated to the http api
/// that is provided on the other side.
///
/// # Example
/// Having the following b-tree:
/// ```f-tree
///
/// root main sequence {
///     remote_action(1,2,3)
/// }
///
/// impl remote_action(a:num, b:num, c:num);
///
/// ```
///
/// Register the action in the Forester instance:
/// ```pseudocode
///
/// fn build_forester(mut fb: ForesterBuilder){
///      ...
///      let action = RemoteHttpAction::new("http://localhost:10000/remote_action/".to_string());
///      fb.register_remote_action("remote_action", action);
///      ...
/// }
///
/// ```
/// Thus, now we need to implement
/// the remote action in the http api: `http://localhost:10000/remote_action/`
/// that will be called by the Forester instance.
///
pub trait ForesterRemoteAction {
    fn tick(&self, request: RemoteActionRequest) -> TickResult;
}
