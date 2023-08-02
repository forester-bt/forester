use crate::tree::parser::ast::{ImportName, Tree};
use crate::tree::project::{AliasName, File, FileName, Project, TreeName};
use crate::tree::{cerr, TreeError};
use std::collections::{HashMap, HashSet};

/// reordering by tree definition
#[derive(Default)]
pub struct ImportMap {
    pub aliases: HashMap<AliasName, TreeName>,
    pub trees: HashMap<TreeName, FileName>,
    pub files: HashSet<FileName>,
}

impl ImportMap {
    /// processes the imports checking there is no crossing between aliases and definitions
    /// ## Note
    /// For now, when the import of the whole file there is no validations on crossings and other things.
    /// Thus, better off to perform imports only for the used definitions.
    pub fn build(file: &File) -> Result<Self, TreeError> {
        let mut map = ImportMap::default();
        for (file, items) in &file.imports {
            for item in items {
                match item {
                    ImportName::Id(v) => {
                        if map.trees.get(v).filter(|f| f != &file).is_some() {
                            return Err(cerr(format!("the import call {v} is presented twice from several different files")));
                        }
                        if map.aliases.get(v).is_some() {
                            return Err(cerr(format!("the import call {v} is presented as alias")));
                        }
                        map.trees.insert(v.to_string(), file.to_string());
                    }
                    ImportName::Alias(id, alias) => {
                        if map.aliases.get(alias).filter(|idt| *idt != id).is_some() {
                            return Err(cerr(format!(
                                "the import alias {alias} is already defined for another call "
                            )));
                        }
                        map.aliases.insert(alias.to_string(), id.to_string());
                        map.trees.insert(id.to_string(), file.to_string());
                    }
                    ImportName::WholeFile => {
                        map.files.insert(file.to_string());
                    }
                }
            }
        }

        Ok(map)
    }

    pub fn find<'a>(
        &'a self,
        key: &TreeName,
        project: &'a Project,
    ) -> Result<(&'a Tree, &'a FileName), TreeError> {
        if let Some(file) = self.trees.get(key) {
            project
                .find_tree(file, key)
                .map(|t| (t, file))
                .ok_or(cerr(format!(
                    "the call {key} can not be found in the file {file} "
                )))
        } else if let Some(id) = self.aliases.get(key) {
            let file = self
                .trees
                .get(id)
                .ok_or(cerr(format!("the call {id} is not presented")))?;
            project
                .find_tree(file, id)
                .map(|t| (t, file))
                .ok_or(cerr(format!(
                    "the call {key} can not be found in the file {file} "
                )))
        } else {
            self.files
                .iter()
                .flat_map(|f| project.files.get(f))
                .find(|f| f.definitions.contains_key(key))
                .and_then(|f| f.definitions.get(key).map(|t| (t, &f.name)))
                .ok_or(cerr(format!(
                    "the call {key} can not be found among the file in the project"
                )))
        }
    }
}
