mod builder;
mod tests;

use crate::runtime::action::{ActionKeeper, ActionName};
use crate::runtime::args::{decorator_args, to_rt_args, RtArgs};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::rnode::{DecoratorType, FlowType, Name, RNode, RNodeId, RNodeState};
use crate::runtime::rtree::builder::{Builder, StackItem};
use crate::runtime::RuntimeErrorCause;
use crate::tree::parser::ast::{Argument, Arguments, Call, Calls, Param, Params, Tree};
use crate::tree::project::file::File;
use crate::tree::project::imports::ImportMap;
use crate::tree::project::params::find_rhs_arg;
use crate::tree::project::{FileName, Project};
use crate::tree::TreeError;
use std::collections::{HashMap, HashSet, VecDeque};

pub struct TreeContext {
    bb: BlackBoard,
}

#[derive(Default, Debug)]
pub struct RuntimeTree {
    pub root: RNodeId,
    pub nodes: HashMap<RNodeId, RNode>,
}

impl RuntimeTree {
    pub fn build(project: Project) -> Result<RuntimeTree, TreeError> {
        let (file, name) = &project.main;
        let root = &project.find_root(name, file)?;
        let mut builder = Builder::default();
        let mut r_tree = RuntimeTree::default();

        let root_id = builder.next();
        builder.add_chain_root(root_id);

        let children = builder.push_vec(root.calls.clone(), root_id, file.clone());
        let root_node = RNode::root(root.name.to_string(), children);
        r_tree.root = root_id;
        r_tree.nodes.insert(root_id, root_node);

        while let Some(item) = builder.pop() {
            let StackItem {
                id,
                call,
                parent_id,
                file_name,
            } = item;
            let curr_file = &project.find_file(file_name.as_str())?;
            let import_map = ImportMap::build(curr_file)?;

            match call {
                Call::Lambda(tpe, calls) => {
                    let children = builder.push_vec(calls, id, file_name.clone());
                    builder.add_chain_lambda(id, parent_id);
                    r_tree
                        .nodes
                        .insert(id, RNode::lambda(tpe.try_into()?, children));
                }
                Call::InvocationCapturedArgs(key) => {
                    let (parent_args, parent_params) =
                        builder.get_chain_skip_lambda(&parent_id)?.get_args();
                    let rhs = find_rhs_arg(&key, &parent_params, &parent_args)?;
                    let err_cause = format!("the argument {} should be tree", key);
                    let call = rhs.get_call().ok_or(RuntimeErrorCause::un(err_cause))?;

                    let err_cause = format!("the param {} should have a name", key);
                    let k = call.key().ok_or(RuntimeErrorCause::un(err_cause))?;
                    builder.push_front(
                        id,
                        Call::Invocation(k, call.arguments()),
                        id,
                        file_name.clone(),
                    );
                }
                Call::Decorator(tpe, decor_args, call) => {
                    let (parent_args, parent_params) =
                        builder.get_chain_skip_lambda(&parent_id)?.get_args();
                    builder.add_chain(id, parent_id, parent_args.clone(), parent_params.clone());
                    let child = builder.push(*call, id, file.clone());
                    let d_tpe: DecoratorType = tpe.try_into()?;
                    let rt_args = decorator_args(&d_tpe, decor_args)?;
                    r_tree
                        .nodes
                        .insert(id, RNode::decorator(d_tpe, rt_args, child));
                }
                Call::Invocation(name, args) => {
                    if let Some(tree) = curr_file.definitions.get(&name) {
                        let rt_args = to_rt_args(name.as_str(), args.clone(), tree.params.clone())?;
                        builder.add_chain(id, parent_id, args.clone(), tree.params.clone());
                        if tree.tpe.is_action() {
                            r_tree.nodes.insert(id, RNode::action(name, rt_args));
                        } else {
                            let children =
                                builder.push_vec(tree.calls.clone(), id, file_name.clone());
                            r_tree.nodes.insert(
                                id,
                                RNode::flow(tree.tpe.try_into()?, name, rt_args, children),
                            );
                        }
                    } else {
                        let tree = import_map.find(&name, &project)?;
                        let checked_args =
                            to_rt_args(name.as_str(), args.clone(), tree.params.clone())?;
                        builder.add_chain(id, parent_id, args.clone(), tree.params.clone());
                        let children = builder.push_vec(tree.calls.clone(), id, file_name.clone());

                        if &tree.name != &name {
                            if tree.tpe.is_action() {
                                r_tree.nodes.insert(
                                    id,
                                    RNode::action_alias(tree.name.clone(), name, checked_args),
                                );
                            } else {
                                r_tree.nodes.insert(
                                    id,
                                    RNode::flow_alias(
                                        tree.tpe.try_into()?,
                                        tree.name.clone(),
                                        name,
                                        checked_args,
                                        children,
                                    ),
                                );
                            }
                        } else {
                            if tree.tpe.is_action() {
                                r_tree.nodes.insert(id, RNode::action(name, checked_args));
                            } else {
                                r_tree.nodes.insert(
                                    id,
                                    RNode::flow(tree.tpe.try_into()?, name, checked_args, children),
                                );
                            }
                        };
                    }
                }
            }
        }

        Ok(r_tree)
    }
}
