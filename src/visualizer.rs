mod statements;

use crate::get_pb;
use crate::runtime::rtree::rnode::{RNode, RNodeId};
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::RtOk;
use crate::tree::parser::ast::arg::{Arguments, Params};
use crate::tree::parser::ast::call::Call;
use crate::tree::parser::ast::{ImportName, Key, Tree};
use crate::tree::project::file::File;
use crate::tree::project::imports::ImportMap;
use crate::tree::project::{AliasName, FileName, Project, TreeName};
use crate::tree::{cerr, TreeError};
use crate::visualizer::statements::ToStmt;
use graphviz_rust::cmd::{CommandArg, Format};
use graphviz_rust::dot_generator::*;
use graphviz_rust::dot_structures::*;
use graphviz_rust::printer::PrinterContext;
use graphviz_rust::{exec, exec_dot, print};
use itertools::Itertools;
use serde_json::to_string;
use std::collections::{HashMap, HashSet, VecDeque};
use std::fmt::format;
use std::path::PathBuf;

/// The struct to visualize the given runtime tree to graphviz format.
pub struct Visualizer;

impl<'a> Visualizer {
    pub fn visualize_to_file(
        root: PathBuf,
        file: Option<&String>,
        tree: Option<&String>,
        output: Option<&String>,
    ) -> Result<String, TreeError> {
        let project = match (file, tree) {
            (Some(file), Some(tree)) => {
                Project::build_with_root(file.to_string(), tree.to_string(), root)
            }
            (Some(file), None) => Project::build(file.to_string(), root),
            _ => Project::build("main.tree".to_string(), root),
        }?;

        let output_pb = match output {
            Some(path) => get_pb(path, project.root.clone()),
            None => {
                let mut output_name = PathBuf::from(project.main.0.clone());
                let _ = output_name.set_extension("svg");
                let mut new_output = project.root.clone();
                new_output.push(output_name);
                new_output
            }
        };

        let rt = RuntimeTree::build(project)?.tree;
        Visualizer::svg_file(&rt, output_pb)
    }

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
        Ok(print(
            Visualizer::build_graph(runtime_tree)?,
            &mut PrinterContext::default(),
        ))
    }

    pub fn svg_file(runtime_tree: &RuntimeTree, path: PathBuf) -> Result<String, TreeError> {
        let mut g = Visualizer::build_graph(runtime_tree)?;
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
