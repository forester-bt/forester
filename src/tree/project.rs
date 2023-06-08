use std::borrow::Cow;
use std::collections::{HashMap, HashSet};
use std::fs;
use std::iter::Map;
use std::path::{Path, PathBuf};
use parsit::error::ParseError;
use crate::tree::ast::{AstFile, FileEntity, Key, Import, ImportName, Tree};
use crate::tree::GolError;
use crate::tree::parser::Parser;
use itertools::Itertools;

pub type FileName = String;
pub type TreeName = String;
pub type AliasName = String;

#[derive(Debug, Default, Clone)]
pub struct Project {
    pub root: PathBuf,
    pub main: (FileName,TreeName),
    pub files: HashMap<FileName, File>,
}


pub fn file_to_string(file_path: PathBuf) -> std::io::Result<String> {
    fs::read_to_string(file_path)
}


impl<'a> Project {

    pub fn full_main(&self) -> String {
        let (_,root) = &self.main;
        format!("{}",  root)
    }

    pub fn build_with_root(main_file: FileName, main_call:TreeName, root: PathBuf) -> Result<Project, GolError>{
        let mut project = Project { root: root.clone(), main: ("".to_string(),"".to_string()), files: Default::default() };
        project.main = (main_file.clone(), main_call);
        let _ = project.parse_file(root.clone(), main_file.clone())?;
        Ok(project)
    }

    pub fn build(main_file: FileName, root: PathBuf) -> Result<Project, GolError> {
        let mut project = Project { root: root.clone(), main: ("".to_string(),"".to_string()), files: Default::default() };

        let _ = project.parse_file(root.clone(), main_file.clone())?;

        let main_call =
            project
                .files
                .get(main_file.as_str())
                .and_then(|file| file.definitions.iter().find(|(name, t)| t.is_root()))
                .map(|(name, _)| name.to_string())
                .ok_or(GolError::IOError(format!("no root operation in the file {}", main_file.clone())))?;
        project.main = (main_file, main_call);
        Ok(project)
    }
    fn parse_file(&mut self, mut root: PathBuf, file: FileName) -> Result<(), GolError> {
        let text = file_to_str(root.clone(), file.clone())?;
        let ast_file = Parser::new(text.as_str())?.parse()?;

        if !self.files.contains_key(file.as_str()) {
            let mut file = File::new(file.clone());

            for ent in ast_file.0.into_iter() {
                let _ = match ent {
                    FileEntity::Tree(t) => file.add_def(t)?,
                    FileEntity::Import(i) => {
                        let _ = self.parse_file(root.clone(), i.f_name().to_string())?;
                        file.add_import(i)?
                    }
                };
            }

            self.files.insert(file.name.clone(), file);
        }
        Ok(())
    }
}


pub fn file_to_str<'a>(root: PathBuf, file: FileName) -> Result<String, ParseError<'a>> {
    let mut path = root.clone();
    path.push(file.clone());

    file_to_string(path)
        .map_err(|e| ParseError::ExternalError(format!("error:{}, file:{:?}",e.to_string(),file), 0))
}


#[derive(Debug, Default, Clone, PartialEq)]
pub struct File {
    pub name: String,
    pub imports: HashMap<FileName,HashSet<ImportName>>,
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

    pub fn add_import(&mut self, import: Import) -> Result<(), GolError> {
        self.imports
            .entry(import.0.clone())
            .and_modify(|names|names.extend(import.1.clone()))
            .or_insert(HashSet::from_iter(import.1));
        Ok(())
    }
    pub fn add_def(&mut self, tree: Tree) -> Result<(), GolError> {
        match self.definitions.get(&tree.name.0) {
            None => {
                self.definitions.insert(tree.clone().name.0, tree);
                Ok(())
            }

            Some(_) => {
                Err(GolError::ParserError(format!("the tree '{}' is already presented", tree.name.0)))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::{HashMap, HashSet};
    use std::path::PathBuf;
    use crate::tree::ast::{Argument, Arguments, Call, Calls, Key, Import, ImportName, MesType, Param, Params, Tree, TreeType};
    use crate::tree::project::{File, Project};

    #[test]
    fn smoke() {
        let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        root.push("tree/tests/plain_project");
        let project = Project::build("main.tree".to_string(), root).unwrap();
        assert_eq!(
            project.main, ("main.tree".to_string(),"ball".to_string())
        );

        assert_eq!(
            project.files.get("main.tree"),
            Some(
                &File {
                    name: "main.tree".to_string(),
                    imports:
                    HashMap::from_iter(vec![
                        ("nested/impls.tree".to_string(), HashSet::from_iter(
                            vec![ImportName::Alias("id".to_string(),"idx".to_string()),ImportName::WholeFile])),

                    ]),
                    definitions: HashMap::from_iter(vec![
                        (
                            "ball".to_string(),
                            Tree::new(TreeType::Root, Key("ball".to_string()), Params::default(), Calls::new(vec![
                                Call::lambda(TreeType::Fallback, Calls::new(vec![
                                    Call::invocation("try_to_place_to", Arguments::default()),
                                    Call::invocation("ask_for_help", Arguments::default()),
                                ]))
                            ]))
                        ),
                        (
                            "try_to_place_to".to_string(),
                            Tree::new(TreeType::Sequence, Key("try_to_place_to".to_string()),
                                      Params::new(vec![
                                          Param::new("obj", MesType::Object), Param::new("dest", MesType::Object),
                                      ]),
                                      Calls::new(vec![
                                          Call::lambda(TreeType::Fallback, Calls::new(vec![
                                              Call::invocation("find_ball", Arguments::new(vec![Argument::id("obj")])),
                                          ]))
                                      ]))
                        ),

                        (
                            "find_ball".to_string(),
                            Tree::new(TreeType::Cond, Key("find_ball".to_string()),
                                      Params::new(vec![
                                          Param::new("obj", MesType::Object),
                                      ]),
                                      Calls::default())
                        ),
                    ]),
                }
            )
        );
        assert_eq!(
            project.files.get("nested/impls.tree"),
            Some(
                &File {
                    name: "nested/impls.tree".to_string(),
                    imports: Default::default(),
                    definitions: HashMap::from_iter(vec![
                        (
                            "approach".to_string(),
                            Tree::new(TreeType::Impl, Key("approach".to_string()), Params::new(vec![Param::new("obj", MesType::Object)]), Calls::default())
                        ),
                        (
                            "grasp".to_string(),
                            Tree::new(TreeType::Impl, Key("grasp".to_string()), Params::new(vec![Param::new("obj", MesType::Object)]), Calls::default())
                        ),
                    ]),
                }
            )
        );
    }
}