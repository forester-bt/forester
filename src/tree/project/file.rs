use std::collections::{HashMap, HashSet};
use crate::tree::parser::ast::{Import, ImportName, Tree};
use crate::tree::project::{FileName, TreeName};
use crate::tree::TreeError;

#[derive(Debug, Default, Clone, PartialEq)]
pub struct File {
    pub name: String,
    pub imports: HashMap<FileName, HashSet<ImportName>>,
    pub definitions: HashMap<TreeName, Tree>,
}


impl File {
    pub fn new(name: FileName) -> Self {
        File {
            name,
            imports: Default::default(),
            definitions: Default::default(),
        }
    }

    pub fn add_import(&mut self, import: Import) -> Result<(), TreeError> {
        self.imports
            .entry(import.0.clone())
            .and_modify(|names| names.extend(import.1.clone()))
            .or_insert(HashSet::from_iter(import.1));
        Ok(())
    }
    pub fn add_def(&mut self, tree: Tree) -> Result<(), TreeError> {
        match self.definitions.get(&tree.name) {
            None => {
                self.definitions.insert(tree.clone().name, tree);
                Ok(())
            }

            Some(_) => {
                Err(TreeError::ParserError(format!("the tree '{}' is already presented", tree.name)))
            }
        }
    }
}