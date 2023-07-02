#[cfg(test)]
mod tests {
    use crate::runtime::args::{RtArgs, RtArgument, RtValue};
    use crate::runtime::rtree::rnode::RNodeName::Name;
    use crate::runtime::rtree::rnode::{FlowType, RNode};
    use crate::runtime::rtree::RuntimeTree;
    use crate::test_utils::test_tree;
    use crate::tree::parser::ast::call::Call;
    use graphviz_rust::attributes::arrowhead::vee;
    use std::collections::HashMap;

    #[test]
    fn ho_op() {
        let tree = test_tree("units/ho", "main.tree");
        println!("{:?}", tree);
        assert_eq!(
            tree,
            RuntimeTree {
                root: 1,
                nodes: HashMap::from_iter(vec![
                    (4, RNode::action("say_hi".to_string(), RtArgs::default())),
                    (1, RNode::root("main".to_string(), vec![2])),
                    (
                        3,
                        RNode::flow(
                            FlowType::Sequence,
                            "wrapper".to_string(),
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
                            RtArgs(vec![RtArgument::new(
                                "op".to_string(),
                                RtValue::Call(Call::invocation("say_hi", Default::default()))
                            )]),
                            vec![3]
                        )
                    )
                ]),
                std_nodes: Default::default(),
            }
        )
    }

    #[test]
    fn std_action() {
        let tree = test_tree("actions", "std_actions.tree");
        println!("{:?}", tree);
    }

    #[test]
    fn ho_tree() {
        let tree = test_tree("ho_tree", "main.tree");
        println!("{:?}", tree);
    }
    #[test]
    fn smoke() {
        let tree = test_tree("plain_project", "main.tree");
        println!("{:?}", tree);
    }
}
