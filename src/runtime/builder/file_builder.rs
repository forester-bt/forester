use crate::runtime::{RtResult, RuntimeError};
use crate::tree::project::{FileName, Project, TreeName};
use std::path::PathBuf;

pub struct FileForesterBuilder {
    main_file: Option<FileName>,
    main: Option<TreeName>,
    pub(super) root: Option<PathBuf>,
}

impl FileForesterBuilder {
    pub fn new() -> Self {
        Self {
            main_file: None,
            main: None,
            root: None,
        }
    }

    /// Root folder.
    pub fn root(&mut self, root: PathBuf) {
        self.root = Some(root);
    }
    /// A file that has a main root definition
    pub fn main_file(&mut self, main_file: FileName) {
        self.main_file = Some(main_file);
    }
    /// A name of the main root definition
    pub fn main_tree(&mut self, main_tree: TreeName) {
        self.main = Some(main_tree);
    }

    /// The method to build forester
    pub fn build(self) -> RtResult<Project> {
        match (self.main, self.root.clone(), self.main_file) {
            (None, Some(root), Some(mf)) => Ok(Project::build(mf, root)?),
            (Some(mt), Some(root), Some(mf)) => Ok(Project::build_with_root(mf, mt, root)?),
            _ => Err(RuntimeError::UnImplementedAction(
                "not enough arguments to initialize the project".to_string(),
            )),
        }
    }
}
