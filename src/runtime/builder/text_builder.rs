use crate::runtime::{RtResult, RuntimeError};
use crate::tree::project::Project;

/// The builder to build Forester from the given string.
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
        if let Some(t) = self.text {
            Ok(Project::build_from_text(t)?)
        } else {
            Err(RuntimeError::UnImplementedAction(
                "not enough arguments to initialize the project".to_string(),
            ))
        }
    }
}
