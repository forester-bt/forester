mod statements;

use crate::{runtime_tree_default};
use crate::runtime::rtree::rnode::{RNode, RNodeId};
use crate::runtime::rtree::RuntimeTree;


use crate::tree::TreeError;
use crate::visualizer::statements::ToStmt;
use graphviz_rust::cmd::{CommandArg, Format};
use graphviz_rust::dot_generator::*;
use graphviz_rust::dot_structures::*;
use graphviz_rust::printer::PrinterContext;
use graphviz_rust::{exec, print};
use std::collections::VecDeque;
use std::path::PathBuf;

/// The struct to visualize the given runtime tree to graphviz format.
pub struct Visualizer;

impl Visualizer {
    fn build_graph(runtime_tree: &RuntimeTree) -> Result<Graph, TreeError> {
        let mut graph = graph!(strict di id!(""));
        let mut stack: VecDeque<RNodeId> = VecDeque::new();
        stack.push_back(runtime_tree.root);

        while let Some(id) = stack.pop_front() {
            if let Some(node) = runtime_tree.nodes.get(&id) {
                graph.add_stmt(node.to_stmt(id.to_string()));
                match node {
                    RNode::Leaf(_, _) => {}
                    RNode::Flow(_, _, _, children) => {
                        for c in children {
                            graph.add_stmt(stmt!(edge!(node_id!(id) => node_id!(c))));
                            stack.push_back(*c);
                        }
                    }
                    RNode::Decorator(_, _, child) => {
                        graph.add_stmt(stmt!(edge!(node_id!(id) => node_id!(child))));
                        stack.push_back(*child);
                    }
                }
            } else {
                return Err(TreeError::VisualizationError(format!(
                    "the node with id {id} is not in the tree"
                )));
            }
        }

        Ok(graph)
    }

    pub fn dot(runtime_tree: &RuntimeTree) -> Result<String, TreeError> {
        debug!(target:"visualizer","visualize a given tree ");

        Ok(print(
            Visualizer::build_graph(runtime_tree)?,
            &mut PrinterContext::default(),
        ))
    }
    pub fn project_svg_to_file(
        root: PathBuf,
        file: Option<&String>,
        tree: Option<&String>,
        output: Option<&String>,
    ) -> Result<String, TreeError> {
        let (rts, output_pb) = runtime_tree_default(root, file, tree, output,"svg".to_string())?;
        debug!(target:"visualizer","visualize a given project to a file {:?}", &output_pb);
        Visualizer::rt_tree_svg_to_file(&rts.tree, output_pb)
    }
    pub fn rt_tree_svg_to_file(
        runtime_tree: &RuntimeTree,
        path: PathBuf,
    ) -> Result<String, TreeError> {
        let g = Visualizer::build_graph(runtime_tree)?;
        let p = path.to_str().ok_or(TreeError::VisualizationError(format!(
            "{:?} is not applicable",
            &path
        )))?;

        exec(
            g,
            &mut PrinterContext::default(),
            vec![Format::Svg.into(), CommandArg::Output(p.to_string())],
        )
            .map_err(|e| TreeError::VisualizationError(e.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use crate::runtime::rtree::RuntimeTree;
    use crate::tree::project::Project;
    use crate::visualizer::Visualizer;

    #[test]
    fn smoke() {
        let p = Project::build_from_text(
            r#"
        
        impl a1();
        fallback one(a:tree){
            a1()
            a(..)
        }
        
        root main sequence {
            one(a1())
            a1()
        }
        
        "#
                .to_string(),
        )
            .unwrap();
        let tree = RuntimeTree::build(p).unwrap().tree;

        let result = Visualizer::dot(&tree).unwrap();

        assert_eq!(
            result,
            r#"strict digraph  {
    1[label="(1) root
main ",shape=rect,color=black]
    1 -> 2 
    2[label="(2) sequence",shape=rect,color=darkred]
    2 -> 3 
    2 -> 4 
    3[label="(3) fallback
one (a=a1(<>))",shape=rect,color=blue]
    3 -> 5 
    3 -> 6 
    4[label="(4) a1 ",shape=component,color=green]
    5[label="(5) a1 ",shape=component,color=green]
    6[label="(6) a1 ",shape=component,color=green]
}"#
        );
    }
}
