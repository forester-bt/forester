pub mod builder;
pub mod macros;
pub mod rnode;
mod transform;

use crate::runtime::action::ActionName;
use crate::runtime::args::transform::{to_dec_rt_args, to_rt_args};

use crate::runtime::rtree::rnode::{DecoratorType, RNode, RNodeId};
use crate::runtime::rtree::transform::{StackItem, Transformer};
use crate::runtime::{RtResult, RuntimeError};
use crate::tree::parser::ast::call::Call;

use crate::tree::project::imports::ImportMap;
use crate::tree::project::Project;
use crate::tree::{cerr, TreeError};
use std::collections::{HashMap, HashSet};

pub struct RuntimeTreeStarter {
    pub tree: RuntimeTree,
    // an action with the same name but the import will be from the diff place and to distinguish we keep track here
    pub std_actions: HashSet<ActionName>,
    pub actions: HashSet<ActionName>,
}

/// The runtime tree is a representation of the compilation tree supplemented with some runtime information.
#[derive(Default, Debug, PartialEq)]
pub struct RuntimeTree {
    pub root: RNodeId,
    pub nodes: HashMap<RNodeId, RNode>,
}

impl RuntimeTree {
    pub fn build(project: Project) -> Result<RuntimeTreeStarter, TreeError> {
        let (file, name) = &project.main;
        let root = project.find_root(name, file)?;
        let mut builder = Transformer::default();
        let mut r_tree = RuntimeTree::default();
        let mut std_actions = HashSet::new();
        let mut actions = HashSet::new();

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
                // for lambda there is not many actions since it does not have arguments so just grab a type and children
                Call::Lambda(tpe, calls) => {
                    let children = builder.push_vec(calls, id, file_name.clone());
                    builder.add_chain_lambda(id, parent_id);
                    r_tree
                        .nodes
                        .insert(id, RNode::lambda(tpe.try_into()?, children));
                }
                // for higher order invocation there are two possible cases:
                // - the invocation is passed as an argument from the parent (this chain can be long up)
                //   So we need to find the initially passed call.
                // - since we found it we transform it into a simple invocation call and process it at the next step.
                // - if it is lambda we found it
                Call::HoInvocation(key) => {
                    let call = builder.find_ho_call(&parent_id, &key)?;
                    if call.is_lambda() {
                        builder.push_front(id, call, id, file_name.clone());
                    } else {
                        let k = call
                            .key()
                            .ok_or(cerr(format!("the call {:?} does not have a name", call)))?;

                        builder.push_front(
                            id,
                            Call::invocation(&k, call.arguments()),
                            id,
                            file_name.clone(),
                        );
                    }
                }
                // just take the arguments and transform them into runtime args and push further
                Call::Decorator(tpe, decor_args, call) => {
                    let (_, parent_args, parent_params) =
                        builder.get_chain_skip_lambda(&parent_id)?.get_tree();
                    builder.add_chain(id, parent_id, parent_args.clone(), parent_params.clone());
                    let child = builder.push(*call, id, file.clone());
                    let d_tpe: DecoratorType = tpe.try_into()?;
                    let rt_args = to_dec_rt_args(&d_tpe, decor_args)?;
                    r_tree
                        .nodes
                        .insert(id, RNode::decorator(d_tpe, rt_args, child));
                }
                // firstly we need to find the definition either in the file or in the imports
                // with a consideration of a possible alias and transform the args
                Call::Invocation(name, args) => match curr_file.definitions.get(&name) {
                    Some(tree) => {
                        let rt_args = to_rt_args(name.as_str(), args.clone(), tree.params.clone())?;
                        builder.add_chain(id, parent_id, args.clone(), tree.params.clone());
                        if tree.tpe.is_action() {
                            r_tree.nodes.insert(id, RNode::action(name, rt_args));
                            actions.insert(tree.name.clone());
                        } else {
                            let children =
                                builder.push_vec(tree.calls.clone(), id, file_name.clone());
                            r_tree.nodes.insert(
                                id,
                                RNode::flow(tree.tpe.try_into()?, name, rt_args, children),
                            );
                        }
                    }
                    None => {
                        let (tree, file) = import_map.find(&name, &project)?;
                        if file == "std::actions" {
                            std_actions.insert(tree.name.clone());
                        }
                        let rt_args = to_rt_args(name.as_str(), args.clone(), tree.params.clone())?;
                        builder.add_chain(id, parent_id, args.clone(), tree.params.clone());
                        let children = builder.push_vec(tree.calls.clone(), id, file_name.clone());

                        if tree.name != name {
                            if tree.tpe.is_action() {
                                actions.insert(tree.name.clone());
                                r_tree.nodes.insert(
                                    id,
                                    RNode::action_alias(tree.name.clone(), name, rt_args),
                                );
                            } else {
                                r_tree.nodes.insert(
                                    id,
                                    RNode::flow_alias(
                                        tree.tpe.try_into()?,
                                        tree.name.clone(),
                                        name,
                                        rt_args,
                                        children,
                                    ),
                                );
                            }
                        } else if tree.tpe.is_action() {
                            r_tree
                                .nodes
                                .insert(id, RNode::action(name.clone(), rt_args));
                            actions.insert(name);
                        } else {
                            r_tree.nodes.insert(
                                id,
                                RNode::flow(tree.tpe.try_into()?, name, rt_args, children),
                            );
                        };
                    }
                },
            }
        }

        Ok(RuntimeTreeStarter {
            tree: r_tree,
            std_actions,
            actions,
        })
    }
    pub fn node(&self, id: &RNodeId) -> RtResult<&RNode> {
        self.nodes.get(id).ok_or(RuntimeError::uex(format!(
            "the node {id} is not found in the rt tree"
        )))
    }

    pub fn max_id(&self) -> RNodeId {
        self.nodes.keys().max().cloned().unwrap_or_default()
    }
}
