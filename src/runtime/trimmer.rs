pub mod task;
pub mod validator;

use crate::runtime::action::{Action, ActionName};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::context::{RNodeState, Timestamp};
use crate::runtime::rtree::builder::RtTreeBuilder;
use crate::runtime::rtree::rnode::RNodeId;
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::trimmer::task::TrimTask;
use itertools::Itertools;
use std::collections::{HashMap, HashSet, VecDeque};
use std::fmt::{Debug, Formatter};
use std::sync::{Arc, Mutex};

/// The queue to deliver the modifications for the current state.
/// The major intention is to be able to modify the runtime tree although, it can be used to add some extra information
/// to bb(but it is not a good way to do it since it is hard to control)
/// or perform some validations
/// ```rust
///
/// use forester_rs::*;
/// use forester_rs::runtime::forester::Forester;
/// use forester_rs::runtime::rtree::builder::RtTreeBuilder;
/// use forester_rs::runtime::RtResult;
/// use forester_rs::runtime::trimmer::task::{RtTreeTrimTask, TrimTask};
/// use forester_rs::runtime::trimmer::{RequestBody, TreeSnapshot, TrimRequest};
/// use forester_rs::runtime::rtree::builder::RtNodeBuilder;
/// use forester_rs::runtime::rtree::rnode::RNodeName;
/// use forester_rs::runtime::args::RtArgs;
///
/// fn smoke(mut forester: Forester) {
///
///     forester.add_trim_task(TrimTask::rt_tree(Test));
///     let result = forester.run_until(Some(100)).unwrap();
///     println!("{result}");
/// }
///
/// struct Test;
///
/// impl RtTreeTrimTask for Test {
///     fn process(&self, snapshot: TreeSnapshot<'_>) -> RtResult<TrimRequest> {
///         if snapshot.tick < 90 {
///             Ok(TrimRequest::Skip)
///         } else {
///             let tree = snapshot.tree;
///             let id = tree
///                 .nodes
///                 .iter()
///                 .find(|(_, n)| {
///                     n.name()
///                         .and_then(|n| n.name().ok())
///                         .filter(|n| n.as_str() == "fail_empty")
///                         .is_some()
///                 })
///                 .map(|(id, _)| id)
///                 .unwrap();
///
///             let mut rtb = RtTreeBuilder::new_from(tree.max_id() + 1);
///             rtb.set_as_root(action!(node_name!("success")), id.clone());
///
///             Ok(TrimRequest::attempt(RequestBody::new(
///                 rtb,
///                 Default::default(),
///             )))
///         }
///     }
/// }
///
/// ```
#[derive(Default)]
pub struct TrimmingQueue {
    tasks: VecDeque<TrimTask>,
}
impl TrimmingQueue {
    /// how many tasks are in the queue
    pub fn len(&self) -> usize {
        self.tasks.len()
    }

    /// add a task to the queue
    pub fn push(&mut self, task: TrimTask) {
        self.tasks.push_back(task);
    }

    /// add group of tasks in the queue
    pub fn push_all(&mut self, tasks: Vec<TrimTask>) {
        self.tasks.extend(tasks.into_iter());
    }

    /// pull out the tasks from the queue
    pub fn pop(&mut self) -> Option<TrimTask> {
        self.tasks.pop_front()
    }
}

/// Snapshot represents a current state of execution.
/// It is used to calculate the reason for trimming
#[derive(Debug, Clone)]
pub struct TreeSnapshot<'a> {
    pub tick: Timestamp,
    pub bb: Arc<Mutex<BlackBoard>>,
    pub tree: &'a RuntimeTree,
    /// from the ctx
    pub tree_state: &'a HashMap<RNodeId, RNodeState>,
    /// current actions. This field is important when we want to replace one action to another,
    /// that is not in the tree and thus we need to add it manually.
    ///This field helps us to check this out.
    pub actions: HashSet<&'a ActionName>,
}

impl<'a> TreeSnapshot<'a> {
    pub fn new(
        tick: Timestamp,
        bb: Arc<Mutex<BlackBoard>>,
        tree: &'a RuntimeTree,
        tree_state: &'a HashMap<RNodeId, RNodeState>,
        actions: HashSet<&'a ActionName>,
    ) -> Self {
        Self {
            tick,
            bb,
            tree,
            tree_state,
            actions,
        }
    }
}

/// The request to proceed or not.
/// There are three possible ways:
///  defer to the next tick, reject this task at all or try to proceed.
#[derive(Debug)]
pub enum TrimRequest {
    /// To reject current trim request.
    /// It can be useful when other request changed the needed data already.  
    Reject,

    /// The conditions are not suitable for the changes. Better to wait until it will be better.
    Skip,

    /// Tries to proceed.
    Attempt(RequestBody),
}

impl TrimRequest {
    pub fn attempt(request: RequestBody) -> TrimRequest {
        TrimRequest::Attempt(request)
    }
}

/// The information to change.
pub struct RequestBody {
    /// The subtree that needs to be replaced.
    /// Basically the major important change appears in the root node of the builder.
    /// The other nodes are just supporters.
    pub tree_b: RtTreeBuilder,

    /// If we deliver a new action or we change an implementation of the actions this field helps to handle it.
    /// Eventually all actions enlisted here will be delivered to the main tree.  
    pub actions: HashMap<ActionName, Action>,
}

impl RequestBody {
    pub fn new(tree_b: RtTreeBuilder, actions: HashMap<ActionName, Action>) -> Self {
        Self { tree_b, actions }
    }
}

impl Debug for RequestBody {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let mut str = f.debug_struct("body");
        str.field("tree_b", &self.tree_b);
        let actions = self.actions.keys().join(",");

        str.field("actions", &actions.as_str());

        Ok(())
    }
}
