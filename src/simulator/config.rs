use crate::read_file;
use crate::runtime::RtResult;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use std::path::PathBuf;

/// Just a profile to build the simulator
/// It has an information about the profile in common and also about every action that needs to be stubbed.
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct SimProfile {
    #[serde(default)]
    pub config: SimProfileConfig,
    #[serde(default)]
    pub actions: Vec<Action>,
}

impl SimProfile {
    pub fn parse(src: &str) -> RtResult<SimProfile> {
        Ok(serde_yaml::from_str(src)?)
    }
    pub fn parse_file(file: &PathBuf) -> RtResult<SimProfile> {
        Ok(serde_yaml::from_str(read_file(file)?.as_str())?)
    }
}

/// The tracer part
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct TracerSimConfig {
    pub file: Option<String>,
    pub dt_fmt: Option<String>,
}

/// The general part of the profile
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct SimProfileConfig {
    /// The tracer config
    #[serde(default)]
    pub tracer: TracerSimConfig,
    /// BB configuration
    #[serde(default)]
    pub bb: BbConfig,
    /// Svg to visualize the tree
    pub graph: Option<String>,
    /// The limitation for the Forester on ticks.
    pub max_ticks: Option<usize>,

    /// The port where needs to deploy the server to communicate with the remote actions.
    #[serde(default)]
    pub http: Option<HttpServ>,
}

/// The http server configuration
/// It is used to communicate with the remote actions.
/// The server is used to receive the requests from the remote actions and to send the responses.
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct HttpServ {
    pub port: u16,
}

#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
pub struct BbConfig {
    pub dump: Option<String>,
    pub load: Option<String>,
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Action {
    pub name: String,
    pub stub: String,
    #[serde(default)]
    pub params: HashMap<String, String>,
}
