pub mod task;
pub mod validator;

use crate::runtime::action::keeper::ActionImpl;
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

/// The queue to deliver the modifications for the runtime tree
#[derive(Default)]
pub struct TrimmingQueue {
    tasks: VecDeque<TrimTask>,
}
impl TrimmingQueue {
    pub fn len(&self) -> usize {
        self.tasks.len()
    }
    pub fn push(&mut self, task: TrimTask) {
        self.tasks.push_back(task);
    }
    pub fn push_all(&mut self, tasks: Vec<TrimTask>) {
        self.tasks.extend(tasks.into_iter());
    }

    pub fn pop(&mut self) -> Option<TrimTask> {
        self.tasks.pop_front()
    }
}

#[derive(Debug, Clone)]
pub struct TreeSnapshot<'a> {
    pub tick: Timestamp,
    pub bb: Arc<Mutex<BlackBoard>>,
    pub tree: &'a RuntimeTree,
    pub tree_state: &'a HashMap<RNodeId, RNodeState>,
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

#[derive(Debug)]
pub enum TrimRequest {
    Reject,
    Skip,
    Attempt(RequestBody),
}

impl TrimRequest {
    pub fn attempt(request: RequestBody) -> TrimRequest {
        TrimRequest::Attempt(request)
    }
}

pub struct RequestBody {
    pub tree_b: RtTreeBuilder,
    pub actions: HashMap<ActionName, Action>,
}

impl RequestBody {
    pub fn new(tree_b: RtTreeBuilder, actions: HashMap<ActionName, Action>) -> Self {
        Self { tree_b, actions }
    }
    pub fn new_only_tree(tree_b: RtTreeBuilder) -> Self {
        Self {
            tree_b,
            actions: Default::default(),
        }
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
