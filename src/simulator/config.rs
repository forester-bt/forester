use crate::read_file;
use crate::runtime::args::RtValue;
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
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct HttpServ {
    /// The host the server binds to. Defaults to `127.0.0.1`.
    #[serde(default = "default_host")]
    pub host: String,
    /// The port the server binds to. `0` selects a random available port.
    pub port: u16,
}

fn default_host() -> String {
    "127.0.0.1".to_string()
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
    /// The key-value pairs to write to the blackboard on every tick of the action.
    /// Values accept strings, integers, floats, arrays and objects.
    #[serde(default)]
    pub bb: HashMap<String, RtValue>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runtime::args::RtValueNumber;

    #[test]
    fn parse_bb_section() {
        let profile = SimProfile::parse(
            r#"
actions:
  - name: sleep
    stub: success
    bb:
      a: 10
      b: "a"
      c: 1.5
      d: [1, 2, 3]
      e:
        x: 1
        y: "z"
"#,
        )
        .unwrap();

        let bb = &profile.actions[0].bb;
        assert_eq!(bb["a"], RtValue::Number(RtValueNumber::Int(10)));
        assert_eq!(bb["b"], RtValue::String("a".to_string()));
        assert_eq!(bb["c"], RtValue::Number(RtValueNumber::Float(1.5)));
        assert_eq!(
            bb["d"],
            RtValue::Array(vec![
                RtValue::Number(RtValueNumber::Int(1)),
                RtValue::Number(RtValueNumber::Int(2)),
                RtValue::Number(RtValueNumber::Int(3)),
            ])
        );
        assert_eq!(
            bb["e"],
            RtValue::Object(HashMap::from_iter(vec![
                ("x".to_string(), RtValue::Number(RtValueNumber::Int(1))),
                ("y".to_string(), RtValue::String("z".to_string())),
            ]))
        );
    }
}
