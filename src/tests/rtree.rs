#[cfg(test)]
mod tests {
    
    use crate::runtime::args::{RtArgs, RtArgument, RtValue};
    
    use crate::runtime::rtree::rnode::FlowType::{RSequence, Root, Sequence};
    use crate::runtime::rtree::rnode::RNodeName::Name;
    use crate::runtime::rtree::rnode::{FlowType, RNode, RNodeName};
    use crate::runtime::rtree::RuntimeTree;
    
    use crate::tree::parser::ast::call::{Call, Calls};
    use crate::tree::parser::ast::TreeType;
    use crate::tree::project::Project;
    
    
    use std::collections::HashMap;
    use std::path::PathBuf;
    use std::vec;

    pub fn test_tree(root_dir: &str, root_file: &str) -> RuntimeTree {
        let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        root.push("tree/tests");
        root.push(root_dir);
        let project = Project::build(root_file.to_string(), root).unwrap();
        RuntimeTree::build(project).unwrap().tree
    }

    #[test]
    fn ho_op() {
        let tree = test_tree("units/ho", "main.tree");
        assert_eq!(
            tree,
            RuntimeTree {
                root: 1,
                nodes: HashMap::from_iter(vec![
                    (4, RNode::action("say_hi".to_string(), "main.tree".to_string(),RtArgs::default())),
                    (1, RNode::root("main".to_string(), "main.tree".to_string(),vec![2])),
                    (
                        3,
                        RNode::flow(
                            FlowType::Sequence,
                            "wrapper".to_string(),
                            "main.tree".to_string(),
                            RtArgs(vec![RtArgument::new(
                                "operation".to_string(),
                                RtValue::Call(Call::ho_invocation("op")),
                            )]),
                            vec![4]
                        )
                    ),
                    (
                        2,
                        RNode::flow(
                            FlowType::Sequence,
                            "id".to_string(),
                            "main.tree".to_string(),
                            RtArgs(vec![RtArgument::new(
                                "op".to_string(),
                                RtValue::Call(Call::invocation("say_hi", Default::default()))
                            )]),
                            vec![3]
                        )
                    )
                ]),
            }
        )
    }

    #[test]
    fn lambda_ho_op() {
        let tree = test_tree("units/ho_lambda", "main.tree");
        let test_tree = RuntimeTree {
            root: 1,
            nodes: HashMap::from_iter(vec![
                (
                    1,
                    RNode::flow(Root, "main".to_string(), "main.tree".to_string(),RtArgs::default(), vec![2]),
                ),
                (
                    2,
                    RNode::flow(
                        RSequence,
                        "x".to_string(),
                        "main.tree".to_string(),
                        RtArgs(vec![RtArgument::new(
                            "t".to_string(),
                            RtValue::Call(Call::Lambda(
                                TreeType::Sequence,
                                Calls {
                                    elems: vec![Call::invocation("success", Default::default())],
                                },
                            )),
                        )]),
                        vec![3],
                    ),
                ),
                (3, RNode::lambda(Sequence, vec![4])),
                (
                    4,
                    RNode::Leaf(Name("success".to_string(),"std::actions".to_string()), Default::default()),
                ),
            ]),
        };
        assert_eq!(tree, test_tree);
    }

    #[test]
    fn std_action() {
        let tree = test_tree("actions", "std_actions.tree");
        let test_tree = RuntimeTree {
            root: 1,
            nodes: HashMap::from_iter(vec![
                (
                    1,
                    RNode::flow(Root, "main".to_string(), "std_actions.tree".to_string(),RtArgs::default(), vec![2]),
                ),
                (
                    2,
                    RNode::Leaf(
                        RNodeName::Name("fail".to_string(),"std::actions".to_string()),
                        RtArgs(vec![RtArgument::new(
                            "reason".to_string(),
                            RtValue::String("test".to_string()),
                        )]),
                    ),
                ),
            ]),
        };
        assert_eq!(tree, test_tree);
    }

    #[test]
    fn ho_tree() {
        let tree = test_tree("ho_tree", "main.tree");
        let max_id = tree.max_id();
        assert_eq!(max_id, 11);
    }
}
