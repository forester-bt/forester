mod statements;
#[cfg(test)]
mod tests;

use std::collections::{HashMap, HashSet, VecDeque};
use std::fmt::format;
use std::path::PathBuf;
use graphviz_rust::cmd::{CommandArg, Format};
use graphviz_rust::dot_generator::*;
use graphviz_rust::dot_structures::*;
use graphviz_rust::{exec, print};
use graphviz_rust::printer::PrinterContext;
use itertools::Itertools;
use crate::tree::parser::ast::{Arguments, Call, ImportName, Key, Params, Tree};
use crate::tree::{cerr, TreeError};
use crate::tree::project::imports::ImportMap;
use crate::tree::project::{AliasName, FileName, Project, TreeName};
use crate::tree::project::file::File;
use crate::tree::project::invocation::Invocation;
use crate::tree::project::params::find_arg;
use crate::tree::visualizer::statements::ToStmt;


struct VizItemParent {
    id: String,
    params: Params,
    args: Arguments,
}

struct VizItem {
    call: Call,
    parent: VizItemParent,
    file_name: String,

}

#[derive(Default)]
struct State {
    gen: usize,
    pub stack: VecDeque<VizItem>,

}

impl State {
    fn next(&mut self) -> String {
        self.gen += 1;
        self.curr()
    }
    fn curr(&self) -> String {
        self.gen.to_string()
    }
    fn push(&mut self, call: Call, parent_id: String, params: Params, args: Arguments, file: String) {
        self.stack.push_back(
            VizItem {
                call,
                parent: VizItemParent { id: parent_id, params, args },
                file_name: file,
            }
        )
    }
    fn push_front(&mut self, call: Call, parent_id: String, params: Params, args: Arguments, file: String) {
        self.stack.push_front(
            VizItem {
                call,
                parent: VizItemParent { id: parent_id, params, args },
                file_name: file,
            }
        )
    }
    fn pop(&mut self) -> Option<VizItem> {
        self.stack.pop_front()
    }
}

struct Visualizer<'a> {
    project: &'a Project,
}


impl<'a> Visualizer<'a> {
    fn init_with_root(&self) -> Result<&Tree, TreeError> {
        let (main_file, root) = &self.project.main;

        self.project.files
            .get(main_file)
            .ok_or(cerr(format!("no main file {}", main_file)))?
            .definitions.get(root)
            .ok_or(cerr(format!("no root {} in {}", root, main_file)))
    }
    fn get_file(&self, file: &String) -> Result<&File, TreeError> {
        self.project
            .files
            .get(file.as_str())
            .ok_or(cerr(format!("unexpected error: the file {} not exists", &file)))
    }
    fn build_graph(&self) -> Result<Graph, TreeError> {
        let (file, name) = &self.project.main;
        let mut graph = graph!(strict di id!(name));
        let root = self.init_with_root()?;

        let mut state = State::default();

        graph.add_stmt(root.to_inv().to_stmt(state.next()));

        for call in &root.calls.elems {
            state.push(
                call.clone(),
                state.curr(),
                root.params.clone(),
                Arguments::default(),
                file.clone());
        }

        while let Some(item) = state.pop() {
            let VizItem {
                call,
                parent: VizItemParent { id, params, args },
                file_name
            } = item;

            let curr_file = &self.get_file(&file_name)?;
            let import_map = ImportMap::build(curr_file)?;

            let node = match call {
                Call::Lambda(tpe, calls) => {
                    let stmt = tpe.to_stmt(state.next());
                    for call in &calls.elems {
                        state.push(
                            call.clone(),
                            state.curr(),
                            params.clone(),
                            args.clone(),
                            file_name.clone());
                    }
                    stmt
                }
                Call::InvocationCapturedArgs(key) => {
                    let rhs = find_arg(&key, &params, &args)?;
                    let call =
                        rhs
                            .get_call()
                            .ok_or(cerr(format!("the argument {} should be tree", key)))?;
                    let k = call.key().ok_or(
                        cerr(format!("the param {} should have a name",key))
                    )?;
                    state.push_front(Call::Invocation(k, call.arguments()), id, params, args, file_name.clone());
                    continue;
                }
                Call::Invocation(name, args) => {
                    if let Some(tree) = curr_file.definitions.get(&name) {
                        let stmt = tree.to_inv_args(args.clone()).to_stmt(state.next());
                        for call in &tree.calls.elems {
                            state.push(
                                call.clone(),
                                state.curr(),
                                tree.params.clone(),
                                args.clone(),
                                file.clone());
                        }
                        stmt
                    } else {
                        let tree = import_map.find(&name, self.project)?;
                        let stmt = if &tree.name != &name {
                            Invocation::new_with_alias(&tree, name.clone(), args.clone())
                                .to_stmt(state.next())
                        } else {
                            Invocation::new(&tree, args.clone()).to_stmt(state.next())
                        };

                        for call in &tree.calls.elems {
                            state.push(call.clone(),
                                       state.curr(),
                                       tree.params.clone(),
                                       args.clone(),
                                       file.clone());
                        }
                        stmt
                    }
                }
                Call::Decorator(tpe, decor_args, call) => {
                    let stmt = (tpe, decor_args).to_stmt(state.next());
                    state.push(
                        *call,
                        state.curr(),
                        params.clone(),
                        args.clone(),
                        file.clone());
                    stmt
                }
            };
            let edge = stmt!(edge!(node_id!(id) => node_id!(state.curr())));
            graph.add_stmt(node);
            graph.add_stmt(edge);
        }


        Ok(graph)
    }


    pub fn to_svg_file(&mut self, path: String) -> Result<String, TreeError> {
        let mut g = self.build_graph()?;
        println!("{}", print(g.clone(), &mut PrinterContext::default()));
        exec(
            g,
            &mut PrinterContext::default(),
            vec![
                Format::Svg.into(),
                CommandArg::Output(path),
            ],
        ).map_err(|e| TreeError::VisualizationError(e.to_string()))
    }
}


impl<'a> Visualizer<'a> {
    pub fn new(project: &'a Project) -> Self {
        Self { project }
    }
}
