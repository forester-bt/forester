use std::collections::{HashMap, HashSet};
use crate::tree::parser::ast::ImportName;
use crate::tree::{cerr, TreeError};
use crate::tree::project::{AliasName, File, FileName, TreeName};

#[derive(Default)]
pub struct ImportMap {
    pub aliases: HashMap<AliasName, TreeName>,
    pub trees: HashMap<TreeName, FileName>,
    pub files: HashSet<FileName>,
}

impl ImportMap {
    pub fn build(file: &File) -> Result<Self, TreeError> {
        let mut map = ImportMap::default();
        for (file, items) in &file.imports {
            for item in items {
                match item {
                    ImportName::Id(v) => {
                        if map.trees.get(v).filter(|f| f != &file).is_some() {
                            return Err(cerr(format!("the import call {} is presented twice from several different files", v)));
                        }
                        if map.aliases.get(v).is_some() {
                            return Err(cerr(format!("the import call {} is presented as alias", v)));
                        }
                        map.trees.insert(v.to_string(), file.to_string());
                    }
                    ImportName::Alias(id, alias) => {
                        if map.aliases.get(alias).filter(|id| id != id).is_some() {
                            return Err(cerr(format!("the import alias {} is already defined for another call ", alias)));
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
}