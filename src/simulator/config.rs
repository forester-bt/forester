use crate::read_file;
use crate::runtime::{RtResult, RuntimeError};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct SimProfile {
    #[serde(default)]
    pub config: SimProfileConfig,
    #[serde(default)]
    pub actions: Vec<Action>,
}

impl SimProfile {
    pub fn parse(file: &str) -> RtResult<SimProfile> {
        Ok(serde_yaml::from_str(file)?)
    }
    pub fn parse_file(file: &PathBuf) -> RtResult<SimProfile> {
        Ok(serde_yaml::from_str(read_file(file)?.as_str())?)
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct SimProfileConfig {
    pub trace: Option<String>,
    #[serde(default)]
    pub bb: BbConfig,
    pub graph: Option<String>,
    pub max_ticks: Option<usize>,
}

#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
pub struct BbConfig {
    pub dump: Option<String>,
    pub load: Option<String>,
}

impl Default for SimProfileConfig {
    fn default() -> Self {
        SimProfileConfig {
            trace: None,
            bb: BbConfig::default(),
            graph: None,
            max_ticks: None,
        }
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Action {
    pub name: String,
    pub stub: String,
    #[serde(default)]
    pub params: HashMap<String, String>,
}
