pub mod analyzer;
pub mod builder;
pub mod iter;
pub mod macros;
pub mod rnode;
pub mod transform;

use crate::runtime::action::ActionName;
use crate::runtime::args::transform::{to_dec_rt_args, to_rt_args};

use crate::runtime::rtree::rnode::{DecoratorType, RNode, RNodeId};
use crate::runtime::rtree::transform::{StackItem, Transformer};
use crate::runtime::{RtOk, RtResult, RuntimeError};
use crate::tree::parser::ast::call::Call;

use crate::runtime::rtree::analyzer::RtTreeAnalyzer;
use crate::runtime::rtree::iter::RtTreeBfsIter;
use crate::tree::project::imports::ImportMap;
use crate::tree::project::{FileName, Project};
use crate::tree::{cerr, TreeError};
use std::collections::{HashMap, HashSet, VecDeque};
use std::path::PathBuf;
use crate::converter::Converter;
use crate::converter::to_nav::ToRosNavConverter;

/// The auxiliary structure that encapsulates the runtime tree
/// and some additional information about actions
pub struct RuntimeTreeStarter {
    pub tree: RuntimeTree,
    // the separate tables for standard and all actions
    // the reason for this is that the user actions can have the same name as the standard ones
    // and we need to distinguish them
    pub std_actions: HashSet<(ActionName, FileName)>,
    pub actions: HashSet<ActionName>,
}

/// The runtime tree is a representation of the compilation tree supplemented with some runtime information.
#[derive(Default, Debug, PartialEq)]
pub struct RuntimeTree {
    pub root: RNodeId,
    pub nodes: HashMap<RNodeId, RNode>,
}

impl RuntimeTree {
    /// Returns bfs iterator over the runtime tree
    pub fn iter(&self) -> RtTreeBfsIter<'_> {
        RtTreeBfsIter {
            queue: VecDeque::from(vec![self.root]),
            tree: &self,
        }
    }
    /// Returns the analyzer for the runtime tree
    /// which provides methods to analyze the tree
    /// and find nodes by some criteria
    ///
    /// # Example
    ///
    /// ```
    ///     use forester_rs::runtime::args::RtArgs;
    ///     use forester_rs::runtime::rtree::builder::RtNodeBuilder;
    ///     use forester_rs::runtime::rtree::builder::RtTreeBuilder;
    ///     use forester_rs::runtime::rtree::rnode::FlowType;
    ///     use forester_rs::runtime::rtree::rnode::RNodeName;
    ///     use forester_rs::*;
    ///
    ///     #[test]
    ///     fn analyzer() {
    ///         let mut rtb = RtTreeBuilder::new();
    ///
    ///         let flow = flow!(fallback node_name!("root"), args!();
    ///             flow!(sequence node_name!("seq"), args!();
    ///                  action!(node_name!("action1"))
    ///             ),
    ///            action!(node_name!("action2"))
    ///         );
    ///
    ///         rtb.add_as_root(flow);
    ///         let tree = rtb.build().unwrap().0;
    ///
    ///         let analyzer = tree.analyze();
    ///
    ///         let a1 = analyzer.find_by(|n| n.is_name("action1")).unwrap();
    ///         let a2 = analyzer.find_by(|n| n.is_name("action2")).unwrap();
    ///         let root = analyzer.find_by(|n| n.is_name("root")).unwrap();
    ///         let seq = analyzer.find_by(|n| n.is_name("seq")).unwrap();
    ///
    ///         assert_eq!(analyzer.parent(&a1), Some(&seq));
    ///         assert_eq!(analyzer.parent(&a2), Some(&root));
    ///         assert_eq!(analyzer.parent(&root), None);
    ///     }
    /// ```
    pub fn analyze(&self) -> RtTreeAnalyzer<'_> {
        RtTreeAnalyzer::new(self)
    }
    /// Builds the runtime tree from the project
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
        let root_node = RNode::root(root.name.to_string(), file.clone(), children);
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
                // - if it is lambda we already found it
                Call::HoInvocation(key) => {
                    let (p_id, _parent_args, _parent_params) =
                        builder.get_chain_skip_lambda(&parent_id)?.get_tree();
                    let call = builder.find_ho_call(&parent_id, &key)?;
                    if call.is_lambda() || call.is_decorator() {
                        builder.push_front(id, call, p_id, file_name.clone());
                    } else {
                        let k = call
                            .key()
                            .ok_or(cerr(format!("the call {:?} does not have a name. Therefore, it is no possible to invoke it by name.", call)))?;

                        builder.push_front(
                            id,
                            Call::invocation(&k, call.arguments()),
                            p_id,
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
                    let rt_args = to_dec_rt_args(&d_tpe, decor_args, parent_args, parent_params)?;
                    r_tree
                        .nodes
                        .insert(id, RNode::decorator(d_tpe, rt_args, child));
                }
                // firstly we need to find the definition either in the file or in the imports
                // with a consideration of a possible alias and transform the args
                Call::Invocation(name, args) => {
                    let (_, parent_args, parent_params) = builder
                        .get_chain_skip_lambda(&parent_id)
                        .map(|e| e.get_tree())
                        .unwrap_or_default();
                    match curr_file.definitions.get(&name) {
                        Some(tree) => {
                            let (rt_args,upd_args) = to_rt_args(
                                name.as_str(),
                                args.clone(),
                                tree.params.clone(),
                                parent_args,
                                parent_params,
                            )?;
                            builder.add_chain(id, parent_id, upd_args, tree.params.clone());
                            if tree.tpe.is_action() {
                                r_tree.nodes.insert(id, RNode::action(name, curr_file.name.clone(), rt_args));
                                actions.insert(tree.name.clone());
                            } else {
                                let children =
                                    builder.push_vec(tree.calls.clone(), id, file_name.clone());
                                r_tree.nodes.insert(
                                    id,
                                    RNode::flow(tree.tpe.try_into()?, name, curr_file.name.clone(), rt_args, children),
                                );
                            }
                        }
                        None => {
                            let (tree, file) = import_map.find(&name, &project)?;
                            if file.contains("::") {
                                std_actions.insert((tree.name.clone(), file.clone()));
                            }
                            let (rt_args,upd_args)  = to_rt_args(
                                name.as_str(),
                                args.clone(),
                                tree.params.clone(),
                                parent_args,
                                parent_params,
                            )?;
                            builder.add_chain(id, parent_id, upd_args, tree.params.clone());
                            let children =
                                builder.push_vec(tree.calls.clone(), id, file_name.clone());

                            if tree.name != name {
                                if tree.tpe.is_action() {
                                    actions.insert(tree.name.clone());
                                    r_tree.nodes.insert(
                                        id,
                                        RNode::action_alias(tree.name.clone(), file.clone(), name, rt_args),
                                    );
                                } else {
                                    r_tree.nodes.insert(
                                        id,
                                        RNode::flow_alias(
                                            tree.tpe.try_into()?,
                                            tree.name.clone(),
                                            file.clone(),
                                            name,
                                            rt_args,
                                            children,
                                        ),
                                    );
                                }
                            } else if tree.tpe.is_action() {
                                r_tree
                                    .nodes
                                    .insert(id, RNode::action(name.clone(), file.clone(), rt_args));
                                actions.insert(name);
                            } else {
                                r_tree.nodes.insert(
                                    id,
                                    RNode::flow(tree.tpe.try_into()?, name, file.clone(), rt_args, children),
                                );
                            };
                        }
                    }
                }
            }
        }

        Ok(RuntimeTreeStarter {
            tree: r_tree,
            std_actions,
            actions,
        })
    }
    /// Returns the node by id
    pub fn node(&self, id: &RNodeId) -> RtResult<&RNode> {
        self.nodes.get(id).ok_or(RuntimeError::uex(format!(
            "the node {id} is not found in the rt tree"
        )))
    }

    /// find the max given id in the tree
    pub fn max_id(&self) -> RNodeId {
        self.nodes.keys().max().cloned().unwrap_or_default()
    }

    /// Converts the runtime tree into the ROS navigation xml file
    pub fn to_ros_nav(&self, xml: PathBuf) -> RtOk {
        ToRosNavConverter::new(&self, xml).convert()
    }
}

#[cfg(test)]
mod tests {
    use crate::runtime::args::{RtArgs, RtArgument, RtValue};
    use crate::runtime::rtree::rnode::FlowType::{Fallback, Root, Sequence};
    use crate::runtime::rtree::rnode::RNode::{Flow, Leaf};
    use crate::runtime::rtree::rnode::RNodeName::{Lambda, Name};
    use crate::runtime::rtree::RuntimeTree;
    use crate::tree::project::Project;
    use std::collections::{HashSet};
    use itertools::Itertools;


    #[test]
    fn smoke() {
        let project = Project::build_from_text(
            r#"
          import "std::actions"
          impl action();
          root main fallback{
                sequence {
                    action()
                    success()
                }
            }        
        "#
                .to_string(),
        )
            .unwrap();

        let st_tree = RuntimeTree::build(project).unwrap();

        assert_eq!(
            st_tree.std_actions,
            HashSet::from_iter(vec![("success".to_string(), "std::actions".to_string())])
        );
        assert_eq!(
            st_tree.actions,
            HashSet::from_iter(vec!["action".to_string(), "success".to_string()])
        );
        assert_eq!(st_tree.tree.nodes.len(), 5);
        assert_eq!(st_tree.tree.root, 1);
        assert_eq!(st_tree.tree.max_id(), 5);
        let items: Vec<_> = st_tree.tree.nodes.iter().sorted_by_key(|e| e.0).collect();
        assert_eq!(
            items,
            vec![
                (
                    &1usize,
                    &Flow(Root, Name("main".to_string(), "_".to_string()), RtArgs(vec![]), vec![2])
                ),
                (&2usize, &Flow(Fallback, Lambda, RtArgs(vec![]), vec![3])),
                (&3usize, &Flow(Sequence, Lambda, RtArgs(vec![]), vec![4, 5])),
                (&4usize, &Leaf(Name("action".to_string(), "_".to_string()), RtArgs(vec![]))),
                (&5usize, &Leaf(Name("success".to_string(), "std::actions".to_string()), RtArgs(vec![]))),
            ]
        );
    }

    #[test]
    fn decorator_lambda() {
        let project = Project::build_from_text(
            r#"
          impl action();
          root main f(t = retry(1) action())
          sequence f(t:tree) t(..)
        "#
                .to_string(),
        )
            .unwrap();

        let st_tree = RuntimeTree::build(project).unwrap().tree;

        assert_eq!(st_tree.nodes.len(), 4)
    }

    #[test]
    fn params() {
        let project = Project::build_from_text(
            r#"
        impl consumer(arg:any);

        root main test(1)

        sequence test(a:any){
            test2(a)
        }

        sequence test2(a1:num){
            consumer(a1)
        }
        "#
                .to_string(),
        )
            .unwrap();

        let st_tree = RuntimeTree::build(project).unwrap().tree;

        let items: Vec<_> = st_tree.nodes.iter().sorted_by_key(|e| e.0).collect();
        assert_eq!(
            items,
            vec![
                (
                    &1usize,
                    &Flow(
                        Root,
                        Name("main".to_string(), "_".to_string()),
                        RtArgs(vec![]),
                        vec![2],
                    )
                ),
                (
                    &2usize,
                    &Flow(
                        Sequence,
                        Name("test".to_string(), "_".to_string()),
                        RtArgs(vec![RtArgument::new(
                            "a".to_string(),
                            RtValue::int(1))]),
                        vec![3],
                    )
                ),
                (
                    &3usize,
                    &Flow(
                        Sequence,
                        Name("test2".to_string(), "_".to_string()),
                        RtArgs(vec![RtArgument::new(
                            "a1".to_string(),
                            RtValue::int(1))]),
                        vec![4],
                    )
                ),
                (
                    &4usize,
                    &Leaf(
                        Name("consumer".to_string(), "_".to_string()),
                        RtArgs(vec![RtArgument::new(
                            "arg".to_string(),
                            RtValue::int(1))]),
                    )
                ),
            ]
        );
    }

    #[test]
    fn params2() {
        let project = Project::build_from_text(
            r#"
        impl consumer(arg0:any,arg:any);
        root main test("-b-",1)

        sequence test(b:any,a:any){
            test2(a,b)
        }

        sequence test2(b1:num,a1:any){
            consumer(b1,a1)
        }
        "#
                .to_string(),
        )
            .unwrap();

        let st_tree = RuntimeTree::build(project).unwrap().tree;

        let item  = st_tree.nodes.iter().find(|(id,_)|**id == 4).unwrap().1.args();
        assert_eq!(
            item,
            RtArgs(vec![
                RtArgument::new("arg0".to_string(), RtValue::int(1) ),
                RtArgument::new("arg".to_string(), RtValue::str("-b-".to_string()))
            ])
        );
    }
}
