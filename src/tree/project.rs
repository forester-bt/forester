pub mod file;
pub mod imports;

use crate::read_file;
use crate::runtime::action::ActionName;
use crate::tree::parser::ast::{FileEntity, Tree};
use crate::tree::parser::Parser;
use crate::tree::project::file::File;
use crate::tree::{cerr, TreeError};
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use crate::runtime::builder::{builtin, ros_core, ros_nav};

pub type FileName = String;
pub type TreeName = String;
pub type AliasName = String;

/// the base structure represents the folder on the disk with some auxiliary info
/// ## Structure
///   - `root` is a root of the project. Every import relates to it.
///   - `main` is a pointer to the file and definition when the tree is started.
///   - `files` is a map of the files
///   - `std` is a set of the standard actions
#[derive(Debug, Default, Clone)]
pub struct Project {
    pub root: PathBuf,
    pub main: (FileName, TreeName),
    pub files: HashMap<FileName, File>,
    pub std: HashSet<ActionName>,
}

impl<'a> Project {
    pub fn find_file(&'a self, f_name: &str) -> Result<&'a File, TreeError> {
        self.files.get(f_name).ok_or(cerr(format!(
            "unexpected error: the file {f_name} not exists"
        )))
    }
    pub fn find_root(&'a self, name: &TreeName, file: &FileName) -> Result<&'a Tree, TreeError> {
        self.find_file(file)?
            .definitions
            .get(name)
            .ok_or(cerr(format!("no root {name} in {file}")))
    }

    pub fn find_tree(&self, file: &FileName, tree: &TreeName) -> Option<&Tree> {
        self.files.get(file).and_then(|f| f.definitions.get(tree))
    }

    /// build the project with the given root and main file
    ///
    /// Suppose we have the following structure:
    /// ```no-run
    /// - root_folder
    ///     - folder    
    ///         - main.tree # root tree_name
    ///     - other.tree
    /// ```
    /// Setting up the rooot as root_folder allows pulling in the other.tree file.
    pub fn build_with_root(
        main_file: FileName,
        main_call: TreeName,
        root: PathBuf,
    ) -> Result<Project, TreeError> {
        debug!(
            target:"ast",
            "built project with root: {:?}, main file: {} and root definition: {} ",
            &root, main_file, main_call
        );
        let mut project = Project {
            root: root.clone(),
            main: ("".to_string(), "".to_string()),
            files: Default::default(),
            std: Default::default(),
        };
        project.main = (main_file.clone(), main_call);
        project.parse_file(root, main_file)?;
        Ok(project)
    }
    /// build the project with the given main file and root.
    /// The root will be found in the main file.
    /// If there are more than one root in the main file, the first one will be used.
    pub fn build(main_file: FileName, root: PathBuf) -> Result<Project, TreeError> {
        let mut project = Project {
            root: root.clone(),
            main: ("".to_string(), "".to_string()),
            files: Default::default(),
            std: Default::default(),
        };

        project.parse_file(root.clone(), main_file.clone())?;

        let main_call = project
            .files
            .get(main_file.as_str())
            .and_then(|file| file.definitions.iter().find(|(_name, t)| t.is_root()))
            .map(|(name, _)| name.to_string())
            .ok_or(TreeError::IOError(format!(
                "no root operation in the file {}",
                main_file.clone()
            )))?;
        debug!(
           target:"ast",
            "built project with root: {:?}, main file: {} and root definition: {} ",
            &root, main_file, main_call
        );
        project.main = (main_file, main_call);
        Ok(project)
    }
    /// build the project with the given text.
    /// The root will be empty.
    ///
    /// # Note
    /// If there are some imports to the other files they will not work
    /// unless the imports are absolute,
    pub fn build_from_text(text: String) -> Result<Project, TreeError> {
        let mut project = Project {
            root: PathBuf::new(),
            main: ("".to_string(), "".to_string()),
            files: Default::default(),
            std: Default::default(),
        };

        project.parse_text(text)?;

        let main_call = project
            .files
            .get("_")
            .and_then(|file| file.definitions.iter().find(|(_name, t)| t.is_root()))
            .map(|(name, _)| name.to_string())
            .ok_or(TreeError::IOError(
                "no root operation in the given text".to_string(),
            ))?;
        debug!(target:"ast","built project from text with root: {}", main_call);
        project.main = ("_".to_string(), main_call);
        Ok(project)
    }

    fn parse_text(&mut self, text: String) -> Result<(), TreeError> {
        let ast_file = Parser::new(text.as_str())?.parse()?;

        let mut file = File::new("_".to_string());
        for ent in ast_file.0.into_iter() {
            match ent {
                FileEntity::Tree(t) => file.add_def(t)?,
                FileEntity::Import(i) => {
                    self.parse_file(PathBuf::new(), i.f_name().to_string())?;
                    file.add_import(i)?
                }
            };
        }

        self.files.insert(file.name.clone(), file);
        Ok(())
    }

    fn parse_file(&mut self, root: PathBuf, file: FileName) -> Result<(), TreeError> {
        let text = file_to_str(root.clone(), file.clone())?;
        let ast_file = Parser::new(text.as_str())?.parse()?;

        if !self.files.contains_key(file.as_str()) {
            let mut file = File::new(file);

            for ent in ast_file.0.into_iter() {
                match ent {
                    FileEntity::Tree(t) => file.add_def(t)?,
                    FileEntity::Import(i) => {
                        self.parse_file(root.clone(), i.f_name().to_string())?;
                        file.add_import(i)?
                    }
                };
            }

            self.files.insert(file.name.clone(), file);
        }
        Ok(())
    }
}
fn file_to_str(root: PathBuf, file: FileName) -> Result<String, TreeError> {

    if file.contains("::"){
        let parts:Vec<_> = file.split("::").collect();
        if parts.len() != 2{
            return Err(TreeError::IOError(format!("invalid file name: {}", file)))
        }else {
            match parts.as_slice() {
                ["std","actions"] => Ok(builtin::builtin_actions_file()),
                ["ros","nav2"] => Ok(ros_nav::ros_actions_file()),
                ["ros","core"] => Ok(ros_core::ros_actions_file()),
                _ => Err(TreeError::IOError(format!("invalid file name: {}", file)))
            }
        }

    } else {
        let mut path = root;
        path.push(file);
        Ok(read_file(&path)?)
    }

}

