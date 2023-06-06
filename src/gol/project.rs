use std::borrow::Cow;
use std::collections::{HashMap, HashSet};
use std::fs;
use std::iter::Map;
use std::path::{Path, PathBuf};
use parsit::error::ParseError;
use crate::gol::ast::{AstFile, FileEntity, Id, Import, ImportName, Tree};
use crate::gol::GolError;
use crate::gol::parser::Parser;
use itertools::Itertools;

#[derive(Debug, Default, Clone)]
pub struct Project {
    root: PathBuf,
    main: String,
    files: HashMap<String, File>,
}


pub fn file_to_string(file_path: PathBuf) -> std::io::Result<String> {
    fs::read_to_string(file_path)
}


impl<'a> Project {
    pub fn build(main_file: String, root: PathBuf) -> Result<Project, GolError> {
        let mut project = Project { root: root.clone(), main: "".to_string(), files: Default::default() };

        let _ = project.parse_file(root.clone(), main_file.clone())?;

        let find_root =
            project
                .files
                .get(main_file.as_str())
                .and_then(|file| {
                    file.definitions.iter().find(|(name, t)| t.is_root())
                })
                .map(|(name, _)| name.to_string())
                .ok_or(GolError::IOError(format!("no root operation in the file {}", main_file.clone())))?;
        project.main = find_root;
        Ok(project)
    }
    fn parse_file(&mut self, mut root: PathBuf, file: String) -> Result<(), GolError> {
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


pub fn file_to_str<'a>(root: PathBuf, file: String) -> Result<String, ParseError<'a>> {
    let mut path = root.clone();
    path.push(file);

    file_to_string(path)
        .map_err(|e| ParseError::ExternalError(e.to_string(), 0))
}


#[derive(Debug, Default, Clone, PartialEq)]
pub struct File {
    pub name: String,
    pub imports: HashMap<String,HashSet<ImportName>>,
    pub definitions: HashMap<String, Tree>,
}


impl File {
    pub fn new(name: String) -> Self {
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
    use crate::gol::ast::{Argument, Arguments, Call, Calls, Id, Import, ImportName, MesType, Param, Params, Tree, TreeType};
    use crate::gol::project::{File,  Project};

    #[test]
    fn smoke() {
        let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        root.push("gol/tests/plain_project");

        let project = Project::build("main.gol".to_string(), root).unwrap();
        assert_eq!(
            project.main, "ball".to_string()
        );

        assert_eq!(
            project.files.get("main.gol"),
            Some(
                &File {
                    name: "main.gol".to_string(),
                    imports:
                    HashMap::from_iter(vec![
                        ("nested/impls.gol".to_string(), HashSet::from_iter(
                            vec![ImportName::Alias("id".to_string(),"idx".to_string()),ImportName::WholeFile])),

                    ]),
                    definitions: HashMap::from_iter(vec![
                        (
                            "ball".to_string(),
                            Tree::new(TreeType::Root, Id("ball".to_string()), Params::default(), Calls::new(vec![
                                Call::lambda(TreeType::Fallback, Calls::new(vec![
                                    Call::invocation("try_to_place_to", Arguments::default()),
                                    Call::invocation("ask_for_help", Arguments::default()),
                                ]))
                            ]))
                        ),
                        (
                            "try_to_place_to".to_string(),
                            Tree::new(TreeType::Sequence, Id("try_to_place_to".to_string()),
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
                            Tree::new(TreeType::Cond, Id("find_ball".to_string()),
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
            project.files.get("nested/impls.gol"),
            Some(
                &File {
                    name: "nested/impls.gol".to_string(),
                    imports: Default::default(),
                    definitions: HashMap::from_iter(vec![
                        (
                            "approach".to_string(),
                            Tree::new(TreeType::Impl, Id("approach".to_string()), Params::new(vec![Param::new("obj", MesType::Object)]), Calls::default())
                        ),
                        (
                            "grasp".to_string(),
                            Tree::new(TreeType::Impl, Id("grasp".to_string()), Params::new(vec![Param::new("obj", MesType::Object)]), Calls::default())
                        ),
                    ]),
                }
            )
        );
    }
}