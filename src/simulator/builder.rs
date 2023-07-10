use crate::runtime::{RtResult, RuntimeError};
use crate::simulator::config::SimProfile;
use crate::simulator::Simulator;
use std::path::PathBuf;

#[derive(Default)]
pub struct SimulatorBuilder {
    profile: Option<PathBuf>,
    root: Option<PathBuf>,
    main_file: Option<String>,
    main_tree: Option<String>,
}

impl SimulatorBuilder {
    pub fn new() -> Self {
        SimulatorBuilder {
            profile: None,
            root: None,
            main_file: None,
            main_tree: None,
        }
    }

    pub fn profile(&mut self, profile: PathBuf) {
        self.profile = Some(profile);
    }
    pub fn root(&mut self, root: PathBuf) {
        self.root = Some(root);
    }
    pub fn main_file(&mut self, main_file: String) {
        self.main_file = Some(main_file);
    }
    pub fn main_tree(&mut self, main_tree: String) {
        self.main_tree = Some(main_tree);
    }

    pub fn build(&self) -> RtResult<Simulator> {
        if let Some(root) = &self.root {
            let profile = match &self.profile {
                None => SimProfile::default(),
                Some(p) if p.is_relative() => {
                    let mut profile = root.clone();
                    profile.push(p);
                    SimProfile::parse_file(&profile)?
                }
                Some(p) => SimProfile::parse_file(&p)?,
            };

            if let Some(main_file) = &self.main_file {
                Simulator::build(
                    profile,
                    root.clone(),
                    main_file.to_string(),
                    self.main_tree.clone(),
                )
            } else {
                Err(RuntimeError::IOError(format!(
                    "the main file is not selected"
                )))
            }
        } else {
            Err(RuntimeError::IOError(format!(
                "the root directory is not found"
            )))
        }
    }
}
