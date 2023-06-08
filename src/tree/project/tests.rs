use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use crate::tree::parser::ast::{Argument, Arguments, Call, Calls, Key, Import, ImportName, MesType, Param, Params, Tree, TreeType};
use crate::tree::project::{File, Project};

#[test]
fn smoke() {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root.push("tree/tests/plain_project");
    let project = Project::build("main.tree".to_string(), root).unwrap();
    assert_eq!(
        project.main, ("main.tree".to_string(), "ball".to_string())
    );

    assert_eq!(
        project.files.get("main.tree"),
        Some(
            &File {
                name: "main.tree".to_string(),
                imports:
                HashMap::from_iter(vec![
                    ("nested/impls.tree".to_string(), HashSet::from_iter(
                        vec![ImportName::Alias("id".to_string(), "idx".to_string()), ImportName::WholeFile])),
                ]),
                definitions: HashMap::from_iter(vec![
                    (
                        "ball".to_string(),
                        Tree::new(TreeType::Root, "ball".to_string(), Params::default(), Calls::new(vec![
                            Call::lambda(TreeType::Fallback, Calls::new(vec![
                                Call::invocation("try_to_place_to", Arguments::default()),
                                Call::invocation("ask_for_help", Arguments::default()),
                            ]))
                        ]))
                    ),
                    (
                        "try_to_place_to".to_string(),
                        Tree::new(TreeType::Sequence, "try_to_place_to".to_string(),
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
                        Tree::new(TreeType::Cond, "find_ball".to_string(),
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
                        Tree::new(TreeType::Impl, "approach".to_string(), Params::new(vec![Param::new("obj", MesType::Object)]), Calls::default())
                    ),
                    (
                        "grasp".to_string(),
                        Tree::new(TreeType::Impl, "grasp".to_string(), Params::new(vec![Param::new("obj", MesType::Object)]), Calls::default())
                    ),
                ]),
            }
        )
    );
}