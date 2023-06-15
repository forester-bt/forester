use crate::runtime::action::{ActionKeeper, ActionName};
use crate::runtime::args::RtArgs;
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::rnode::{RNode, RNodeId, RNodeState, RNodeType, RTreeType, TreeName};
use crate::runtime::RuntimeErrorCause;
use crate::tree::parser::ast::{Arguments, Call, Calls, Param, Params, Tree};
use crate::tree::project::file::File;
use crate::tree::project::imports::ImportMap;
use crate::tree::project::params::find_arg;
use crate::tree::project::{FileName, Project};
use crate::tree::TreeError;
use std::collections::{HashMap, HashSet, VecDeque};

pub struct TreeContext {
    bb: BlackBoard,
}

#[derive(Default)]
pub struct RuntimeTree {
    pub root: RNodeId,
    pub nodes: HashMap<RNodeId, RNode>,
}

fn find_file<'a>(project: &'a Project, f_name: &str) -> Result<&'a File, RuntimeErrorCause> {
    project
        .files
        .get(f_name)
        .ok_or(RuntimeErrorCause::io(format!(
            "unexpected error: the file {} not exists",
            f_name
        )))
}
fn find_root<'a>(
    project: &'a Project,
    name: &TreeName,
    file: &FileName,
) -> Result<&'a Tree, RuntimeErrorCause> {
    project
        .files
        .get(file)
        .ok_or(RuntimeErrorCause::io(format!("no main file {}", file)))?
        .definitions
        .get(name)
        .ok_or(RuntimeErrorCause::io(format!(
            "no root {} in {}",
            name, file
        )))
}

impl RuntimeTree {
    pub fn build(project: Project) -> Result<RuntimeTree, TreeError> {
        let (file, name) = &project.main;
        let root = find_root(&project, name, file)?;

        let mut builder = Builder::default();
        let mut r_tree = RuntimeTree::default();

        // add root
        let root_id = builder.curr();
        let children = builder.push_vec(
            root.calls.clone(),
            Params::default(),
            Arguments::default(),
            file.clone(),
        );
        let root_node = RNode::root(root.name.to_string(), children);
        r_tree.root = root_id;
        r_tree.nodes.insert(root_id, root_node);

        while let Some(item) = builder.pop() {
            let Item {
                id,
                call,
                parent: Parent { params, args },
                file_name: f,
            } = item;

            let curr_file = find_file(&project, f.as_str())?;
            let import_map = ImportMap::build(curr_file)?;

            match call {
                Call::Lambda(tpe, calls) => {
                    let r_tpe: RNodeType = tpe.into();
                    let children = builder.push_vec(calls, params.clone(), args.clone(), f.clone());

                    if let RNodeType::Tree(r_t) = r_tpe {
                        r_tree.nodes.insert(id, RNode::lambda(r_t, children));
                    } else {
                        return Err(
                            RuntimeErrorCause::un(format!("unexpected type for lambda")).into()
                        );
                    }
                }
                Call::InvocationCapturedArgs(key) => {
                    let rhs = find_arg(&key, &params, &args)?;
                    let call = rhs.get_call().ok_or(RuntimeErrorCause::un(format!(
                        "the argument {} should be tree",
                        key
                    )))?;

                    let k = call.key().ok_or(RuntimeErrorCause::un(format!(
                        "the param {} should have a name",
                        key
                    )))?;

                    builder.push_front(
                        Call::Invocation(k, call.arguments()),
                        params,
                        args,
                        f.clone(),
                    );
                    continue;
                }
                Call::Decorator(tpe, decor_args, call) => {}
                Call::Invocation(_, _) => {}
            }
        }

        Ok(r_tree)
    }
}
#[derive(Clone)]
struct Parent {
    params: Params,
    args: Arguments,
}

struct Item {
    id: usize,
    call: Call,
    parent: Parent,
    file_name: String,
}

#[derive(Default)]
struct Builder {
    gen: usize,
    stack: VecDeque<Item>,
}

impl Builder {
    fn next(&mut self) -> usize {
        self.gen += 1;
        self.curr()
    }
    fn curr(&self) -> usize {
        self.gen
    }

    fn push(&mut self, call: Call, params: Params, args: Arguments, file_name: String) -> usize {
        let parent = Parent { params, args };
        let id = self.next();
        self.stack.push_back(Item {
            id,
            call,
            parent,
            file_name,
        });
        self.curr()
    }
    fn push_vec(
        &mut self,
        calls: Calls,
        params: Params,
        args: Arguments,
        file_name: String,
    ) -> Vec<usize> {
        let mut children = vec![];
        for call in calls.elems {
            children.push(self.push(call, params.clone(), args.clone(), file_name.clone()));
        }

        children
    }
    fn push_front(
        &mut self,
        call: Call,
        params: Params,
        args: Arguments,
        file_name: String,
    ) -> usize {
        let parent = Parent { params, args };
        let id = self.next();
        self.stack.push_front(Item {
            id,
            call,
            parent,
            file_name,
        });
        self.curr()
    }
    fn pop(&mut self) -> Option<Item> {
        self.stack.pop_front()
    }
}
