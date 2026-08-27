use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::runtime::RtResult;

/// The profile to run a tree with [`crate::runner::Runner`].
///
/// The profile is read from a yaml file where every section is optional
/// and falls back to its default value when absent.
///
/// # Example
/// ```yaml
/// run_until:
///   limit: 10          # or `run_until: no_limit`
///
/// bb:
///   load: "bb_init.json"
///   dump: "bb_final.json"
///
/// tracer:
///   indent: 2
///   time_format: "%H:%M:%S"
///   to_file: "trace.log"
///
/// api:
///   type: http
///   host: "localhost"
///   port: 8080
///
/// actions:
///   - type: http
///     name: fetch_data
///     url: "http://localhost:10000/action"
/// ```
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct RunProfile {
    /// The limitation for the Forester on ticks, `no_limit` by default.
    #[serde(with = "serde_yaml::with::singleton_map")]
    pub run_until: Option<RunUntil>,
    /// The blackboard configuration (`bb` section).
    #[serde(rename = "bb")]
    pub bb_config: Option<BBConfig>,
    /// The tracer configuration (`tracer` section).
    #[serde(rename = "tracer")]
    pub tracer_config: Option<TracerConfig>,
    /// The http server to communicate with the remote actions (`api` section).
    #[serde(rename = "api")]
    pub server_api: Option<ServerAPI>,
    /// The remote actions to register in the Forester.
    pub actions: Vec<RemoteAction>,
}

/// Defines when the run stops.
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunUntil {
    /// Runs until the root tree finishes (`run_until: no_limit`).
    #[default]
    NoLimit,
    /// Runs at most the given number of ticks (`run_until: {limit: 10}`).
    Limit(usize),
}

/// The server to communicate with the remote actions.
/// The remote actions send their requests to it to reach the blackboard and the tracer.
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerAPI {
    /// The http server (`type: http`).
    Http {
        /// The host the server binds to, `127.0.0.1` by default.
        host: Option<String>,
        /// The port the server binds to, `0` (a random available port) by default.
        port: Option<u16>,
    },
}

impl Default for ServerAPI {
    fn default() -> Self {
        ServerAPI::Http {
            host: Some("127.0.0.1".to_string()),
            port: Some(0),
        }
    }
}

/// The tracer part of the profile, mapped to [`crate::tracer::TracerConfig`].
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct TracerConfig {
    /// The indent for the nested trace lines, `2` by default.
    pub indent: Option<usize>,
    /// The time format for the trace timestamps (`chrono` format string).
    pub time_format: Option<String>,
    /// The file to write the trace to, relative to the root folder.
    /// When absent the trace stays in memory.
    pub to_file: Option<PathBuf>,
}

/// The blackboard part of the profile.
#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct BBConfig {
    /// The file to dump the blackboard snapshot to after the run,
    /// relative to the root folder.
    pub dump: Option<String>,
    /// The file with a blackboard snapshot in json format
    /// to load the initial data from before the run, relative to the root folder.
    pub load: Option<String>,
}

/// The remote action to register in the Forester.
/// The `name` should match the action name in the tree.
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum RemoteAction {
    /// The action delegating the tick to a remote http server (`type: http`),
    /// see [`crate::runtime::action::builtin::remote::RemoteHttpAction`].
    #[serde(rename = "http")]
    HttpAction {
        /// The name of the action in the tree.
        name: String,
        /// The url of the remote server executing the action.
        url: String,
    },
}

impl RunProfile {
    /// Reads the profile from a yaml file.
    /// Every section that is absent in the file falls back to its default value;
    /// an empty file gives the default profile.
    pub fn from_file<P: AsRef<Path>>(path: P) -> RtResult<RunProfile> {
        let text = std::fs::read_to_string(path)?;
        if text.trim().is_empty() {
            Ok(RunProfile::default())
        } else {
            Ok(serde_yaml::from_str(&text)?)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::assert_eq;

    fn file(name: &str, content: &str) -> PathBuf {
        let path = std::env::temp_dir().join(name);
        std::fs::write(&path, content).unwrap();
        path
    }

    #[test]
    fn empty_file_gives_default_profile() {
        let path = file("run_profile_empty.yaml", "");
        assert_eq!(RunProfile::from_file(path).unwrap(), RunProfile::default());
    }

    #[test]
    fn absent_sections_fall_back_to_default() {
        let path = file(
            "run_profile_partial.yaml",
            r#"
bb:
  dump: "dump.json"
"#,
        );
        let profile = RunProfile::from_file(path).unwrap();
        assert_eq!(
            profile,
            RunProfile {
                bb_config: Some(BBConfig {
                    dump: Some("dump.json".to_string()),
                    load: None,
                }),
                ..RunProfile::default()
            }
        );
    }

    #[test]
    fn run_until_no_limit() {
        let path = file("run_profile_no_limit.yaml", "run_until: no_limit");
        let profile = RunProfile::from_file(path).unwrap();
        assert_eq!(
            profile,
            RunProfile {
                run_until: Some(RunUntil::NoLimit),
                ..RunProfile::default()
            }
        );
    }

    #[test]
    fn full_profile() {
        let path = file(
            "run_profile_full.yaml",
            r#"
run_until:
  limit: 10
bb:
  dump: "dump.json"
  load: "load.json"
tracer:
  indent: 2
  time_format: "%H:%M:%S"
  to_file: "trace.log"
api:
  type: http
  host: "localhost"
  port: 8080
actions:
  - type: http
    name: "Action1"
    url: "http://localhost:8080/action"
"#,
        );
        let profile = RunProfile::from_file(path).unwrap();
        assert_eq!(
            profile,
            RunProfile {
                run_until: Some(RunUntil::Limit(10)),
                bb_config: Some(BBConfig {
                    dump: Some("dump.json".to_string()),
                    load: Some("load.json".to_string()),
                }),
                tracer_config: Some(TracerConfig {
                    indent: Some(2),
                    time_format: Some("%H:%M:%S".to_string()),
                    to_file: Some(PathBuf::from("trace.log")),
                }),
                server_api: Some(ServerAPI::Http {
                    host: Some("localhost".to_string()),
                    port: Some(8080),
                }),
                actions: vec![RemoteAction::HttpAction {
                    name: "Action1".to_string(),
                    url: "http://localhost:8080/action".to_string(),
                }],
            }
        );
    }

    #[test]
    fn missing_file_gives_error() {
        assert!(RunProfile::from_file("definitely_no_such_file.yaml").is_err());
    }
}
