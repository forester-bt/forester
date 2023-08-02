use crate::runtime::context::TreeContext;
use crate::runtime::forester::Forester;
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::RtResult;
use std::collections::VecDeque;

/// The queue to deliver the modifications for the runtime tree
#[derive(Default)]
pub struct ModificationQueue {
    tasks: VecDeque<ModificationTask>,
}

pub enum ModState {
    Defer,
    Reject,
    Proceed,
}

pub enum ModificationTask {
    RtTree(Box<dyn RtTreeTask>),
}

impl ModificationTask {
    pub fn rt_tree<T>(task: T) -> Self
    where
        T: RtTreeTask + 'static,
    {
        ModificationTask::RtTree(Box::new(task))
    }
}

impl ModificationQueue {
    pub fn push(&mut self, task: ModificationTask) {
        self.tasks.push_back(task);
    }
    pub fn push_all(&mut self, tasks: Vec<ModificationTask>) {
        self.tasks.extend(tasks.into_iter());
    }

    pub fn pop(&mut self) -> Option<ModificationTask> {
        self.tasks.pop_front()
    }
}

/// The task that takes the original tree and returns a part of the tree that needs to be modified.
pub trait RtTreeTask {
    fn modification(&self, original: &RuntimeTree) -> RtResult<RuntimeTree>;
}
