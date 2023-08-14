use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::context::TreeContextRef;

pub trait Remote {}

pub struct RemoteHttpAction {
    url: String,
}

impl Remote for RemoteHttpAction {}

impl Impl for RemoteHttpAction {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        todo!()
    }
}

impl RemoteHttpAction {
    pub fn new(url: String) -> Self {
        Self { url }
    }
}
