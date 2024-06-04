use crate::runtime::trimmer::{TreeSnapshot, TrimRequest};
use crate::runtime::RtResult;


/// The task to trim the tree.
pub enum TrimTask {
    RtTree(Box<dyn RtTreeTrimTask>),
}

impl TrimTask {
    pub fn rt_tree<T>(task: T) -> Self
    where
        T: RtTreeTrimTask + 'static,
    {
        TrimTask::RtTree(Box::new(task))
    }

    pub fn process(&self, snapshot: TreeSnapshot<'_>) -> RtResult<TrimRequest> {
        match self {
            TrimTask::RtTree(t) => t.process(snapshot),
        }
    }
}

/// The task that takes the original tree and returns a part of the tree that needs to be modified.
pub trait RtTreeTrimTask: Send {
    fn process(&self, snapshot: TreeSnapshot<'_>) -> RtResult<TrimRequest>;
}
