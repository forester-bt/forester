use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use crate::tree::parser::ast::{Argument, Arguments, Call, Calls, Key, Import, ImportName, MesType, Param, Params, Tree, TreeType, Message};
use crate::tree::project::{File, Project};

#[test]
fn smoke() {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root.push("tree/tests/plain_project");
    let project = Project::build("amr_simple.tree".to_string(), root).unwrap();
    assert_eq!(
        project.main, ("amr_simple.tree".to_string(), "place_ball_to_target".to_string())
    );


    let file = project.files.get("amr_simple.tree").unwrap();

    assert_eq!(
        file.imports,
        HashMap::from_iter(vec![
            ("nested/impls.tree".to_string(), HashSet::from_iter(
                vec![ImportName::Alias("grasp".to_string(), "grasp_ball".to_string()), ImportName::WholeFile])),
        ])
    );

    assert_eq!(file.definitions.len(), 4);

    assert_eq!(file.definitions.get("place_ball_to_target"), Some(
        &Tree::root(
            "place_ball_to_target",
            vec![
                Call::lambda(TreeType::Fallback, Calls::new(vec![
                    Call::invocation("place_to", Arguments::new(vec![
                        Argument::id_mes("obj", Message::object(vec![
                            ("x".to_string(), Message::int(1)),
                        ])),
                        Argument::id_call(
                            "operation",
                            Call::invocation(
                                "place",
                                Arguments::new(vec![
                                    Argument::mes(Message::array(vec![
                                        Message::int(10)
                                    ]))
                                ])
                            )
                        )
                    ])),
                    Call::decorator(
                        TreeType::Retry,
                        Arguments::new(vec![Argument::mes(Message::int(5))]),
                        Call::invocation("ask_for_help", Arguments::default()),
                    ),
                ]))
            ],
        )
    ));

    assert_eq!(file.definitions.get("place_to"), Some(
        &Tree::new(
            TreeType::Sequence,
            "place_to".to_string(),
            Params::new(vec![
                Param::new("what", MesType::Object),
                Param::new("operation", MesType::Tree),
            ]),
            Calls::new(vec![
                Call::lambda(TreeType::Fallback, Calls::new(vec![
                    Call::invocation("is_approachable", Arguments::new(vec![Argument::id("what")])),
                    Call::invocation("do_job",
                                     Arguments::new(
                                         vec![Argument::call(Call::invocation("approach", Arguments::new(vec![Argument::id("what")])))])),
                ])),
                Call::lambda(TreeType::Fallback, Calls::new(vec![
                    Call::invocation("is_graspable", Arguments::new(vec![Argument::id("what")])),
                    Call::invocation("do_job",
                                     Arguments::new(
                                         vec![Argument::call(Call::invocation("approach", Arguments::new(vec![Argument::id("what")])))])),
                ])),
                Call::lambda(TreeType::Sequence, Calls::new(vec![
                    Call::invocation("savepoint", Arguments::default()),
                    Call::invocation_with_capture("operation"),
                ])),
            ]),
        )
    ));
    assert_eq!(file.definitions.get("place"), Some(
        &  Tree::new(
            TreeType::Sequence,
            "place".to_string(),
            Params::new(vec![
                Param::new("where", MesType::Array),
            ]),
            Calls::new(vec![
                Call::invocation("is_valid_place", Arguments::new(vec![Argument::id("where")])),
                Call::invocation("do_job", Arguments::new(vec![Argument::call(
                    Call::invocation("slowly_drop", Arguments::new(vec![
                        Argument::mes(Message::object(vec![(
                            "cord".to_string(), Message::int(1)
                        )]))
                    ]))
                )])),
            ]),
        )
    ));
    assert_eq!(file.definitions.get("do_job"), Some(
        & Tree::new(
            TreeType::Sequence,
            "do_job".to_string(),
            Params::new(vec![
                Param::new("action", MesType::Tree),
            ]),
            Calls::new(vec![
                Call::invocation("savepoint", Arguments::default()),
                Call::invocation_with_capture("action"),
                Call::invocation("savepoint", Arguments::default()),
            ]),
        )
    ));


    assert_eq!(
        project.files.get("nested/impls.tree"),
        Some(
            &File {
                name: "nested/impls.tree".to_string(),
                imports: Default::default(),
                definitions: HashMap::from_iter(vec![
                    (
                        "is_approachable".to_string(),
                        Tree::new(TreeType::Cond, "is_approachable".to_string(),
                                  Params::new(vec![Param::new("obj", MesType::Object)]),
                                  Calls::default())
                    ),
                    (
                        "is_graspable".to_string(),
                        Tree::new(TreeType::Cond, "is_graspable".to_string(),
                                  Params::new(vec![Param::new("obj", MesType::Object)]),
                                  Calls::default())
                    ),
                    (
                        "is_valid_place".to_string(),
                        Tree::new(TreeType::Cond, "is_valid_place".to_string(),
                                  Params::new(vec![Param::new("obj", MesType::Object)]),
                                  Calls::default())
                    ),
                    (
                        "approach".to_string(),
                        Tree::new(TreeType::Impl, "approach".to_string(),
                                  Params::new(vec![Param::new("obj", MesType::Object)]),
                                  Calls::default())
                    ),
                    (
                        "grasp".to_string(),
                        Tree::new(TreeType::Impl, "grasp".to_string(),
                                  Params::new(vec![Param::new("obj", MesType::Object)]),
                                  Calls::default())
                    ),
                    (
                        "ask_for_help".to_string(),
                        Tree::new(TreeType::Impl, "ask_for_help".to_string(),
                                  Params::default(),
                                  Calls::default())
                    ),
                    (
                        "place_to".to_string(),
                        Tree::new(TreeType::Impl, "place_to".to_string(),
                                  Params::new(vec![Param::new("where", MesType::Array)]),
                                  Calls::default())
                    ),
                    (
                        "savepoint".to_string(),
                        Tree::new(TreeType::Impl, "savepoint".to_string(),
                                  Params::default(),
                                  Calls::default())
                    ),
                    (
                        "slowly_drop".to_string(),
                        Tree::new(TreeType::Impl, "slowly_drop".to_string(),
                                  Params::new(vec![Param::new("params", MesType::Object)]),
                                  Calls::default())
                    ),
                ]),
            }
        )
    );
}