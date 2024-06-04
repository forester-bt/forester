use crate::runtime::blackboard::BBRef;
use crate::runtime::context::{TracerRef, TreeContext, TreeContextRef};

/// The context for the daemon.
#[derive(Clone, Default, Debug)]
pub struct DaemonContext {
    pub bb: BBRef,
    pub tracer: TracerRef,
}

impl DaemonContext {
    pub fn new(bb: BBRef, tracer: TracerRef) -> Self {
        Self { bb, tracer }
    }
}

impl From<TreeContext> for DaemonContext {
    fn from(mut value: TreeContext) -> Self {
        DaemonContext {
            bb: value.bb(),
            tracer: value.tracer(),

        }
    }
}impl From<TreeContextRef> for DaemonContext {
    fn from(value: TreeContextRef) -> Self {
        DaemonContext {
            bb: value.bb(),
            tracer: value.tracer(),

        }
    }
}