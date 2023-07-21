use crate::runtime::action::builtin::data::{CheckEq, LockUnlockBBKey, StoreData, StoreTick};
use crate::runtime::action::builtin::http::HttpGet;
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::action::keeper::ActionKeeper;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::builder::builtin::BuilderBuiltInActions;
use crate::runtime::builder::CommonForesterBuilder;
use crate::runtime::env::RtEnv;
use crate::runtime::forester::Forester;
use crate::runtime::rtree::{RuntimeTree, RuntimeTreeStarter};
use crate::runtime::{RtResult, RuntimeError};
use crate::tracer::Tracer;
use crate::tree::project::{FileName, Project, TreeName};
use std::collections::HashMap;
use std::fmt::format;
use std::path::{Path, PathBuf};

pub struct TextForesterBuilder {
    text: Option<String>,
}

impl TextForesterBuilder {
    pub fn new() -> Self {
        Self { text: None }
    }

    /// add script on the fly.
    /// In that scenario, there is no need in the other attributes like files or root.
    ///
    /// Precautions:
    /// Imports and other files still work only with absolute paths.
    pub fn text(&mut self, txt: String) {
        self.text = Some(txt);
    }

    pub fn build(self) -> RtResult<Project> {
        if let Some(t) = self.text.clone() {
            Ok(Project::build_from_text(t)?)
        } else {
            Err(RuntimeError::UnImplementedAction(format!(
                "not enough arguments to initialize the project"
            )))
        }
    }
}
