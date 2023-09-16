pub mod ros_nav;

use crate::runtime::rtree::rnode::{RNode, RNodeId};
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RtOk, RtResult};
use crate::tree::project::Project;

/// The trait is responsible to transform the runtime tree into the specific given format and the vice versa.
pub trait Exporter {
    type Writer;

    /// Export the runtime tree into the specific format.
    fn export(&self, writer: &mut Self::Writer, tree: &RuntimeTree) -> RtOk {
        let mut stack = vec![(tree.root, State::Ready)];

        while let Some((id, st)) = stack.pop() {
            let node = tree.node(&id)?;
            if node.is_action() {
                &self.write_terminal(writer, id, node)?;
            } else {
                match st {
                    State::Ready => {
                        &self.write_interior_start(writer, id, node)?;
                        stack.push((id, State::Started));
                        for child in node.children().iter().rev() {
                            stack.push((*child, State::Ready));
                        }
                    }
                    State::Started => {
                        &self.write_interior_end(writer, id, node)?;
                    }
                }
            }
        }


        Ok(())
    }

    /// Write the terminal node into the specific format. The terminal node is the node that has no children by contract (actions)
    fn write_terminal(&self, w: &mut Self::Writer, id: RNodeId, node: &RNode) -> RtOk;
    /// Write the opening for an interior node into the specific format.
    /// The interior node is the node that has children by contract (flows)
    fn write_interior_start(&self, w: &mut Self::Writer, id: RNodeId, node: &RNode) -> RtOk;

    /// Write the closing for an interior node into the specific format.
    fn write_interior_end(&self, w: &mut Self::Writer, id: RNodeId, node: &RNode) -> RtOk;
}

enum State {
    Ready,
    Started,
}

#[cfg(test)]
mod tests {
    use crate::exporter::Exporter;
    use crate::runtime::RtOk;
    use crate::runtime::rtree::rnode::{RNode, RNodeId};
    use crate::runtime::rtree::RuntimeTree;
    use crate::tree::project::Project;

    struct TestExporter;

    impl Exporter for TestExporter {
        type Writer = Vec<RNodeId>;

        fn write_terminal(&self, w: &mut Self::Writer, id: RNodeId, node: &RNode) -> RtOk {
            w.push(id);
            Ok(())
        }

        fn write_interior_start(&self, w: &mut Self::Writer, id: RNodeId, node: &RNode) -> RtOk {
            w.push(id);
            Ok(())
        }

        fn write_interior_end(&self, w: &mut Self::Writer, id: RNodeId, node: &RNode) -> RtOk {
            w.push(id);
            Ok(())
        }
    }

    #[test]
    fn order() {
        let project = Project::build_from_text(r#"
                import "std::actions"

                root main // 1
                 sequence // 2
                 {
                        first_lvl() // 3
                        first_lvl() // 4
                }

                sequence first_lvl() {
                    success() // 5 8
                    success() // 6 9
                    success() // 7 10
                }
                // order
                // - 1
                // - - 2
                // - - - 3
                // - - - - 5
                // - - - - 6
                // - - - - 7
                // - - - 3
                // - - - 4
                // - - - - 8
                // - - - - 9
                // - - - - 10
                // - - - 4
                // - - 2
                // - 1

            "#.to_string()).unwrap();

        let tree = RuntimeTree::build(project).unwrap().tree;

        let mut order = Vec::new();

        let _ = TestExporter.export(&mut order, &tree);
        assert_eq!(order, vec![1, 2, 3, 5, 6, 7, 3, 4, 8, 9, 10, 4, 2, 1]);
    }
}