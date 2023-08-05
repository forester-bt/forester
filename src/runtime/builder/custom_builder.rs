use crate::runtime::rtree::builder::{RtNodeBuilder, RtTreeBuilder};
use crate::runtime::rtree::rnode::RNodeId;
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::RtResult;
use std::collections::HashSet;

pub struct CustomForesterBuilder {
    rtb: RtTreeBuilder,
}

impl CustomForesterBuilder {
    pub fn new() -> Self {
        Self {
            rtb: RtTreeBuilder::new(),
        }
    }

    pub fn add_rt_node(&mut self, node_b: RtNodeBuilder) -> RNodeId {
        self.rtb.add(node_b)
    }

    /// The method to build forester
    pub fn build(self) -> RtResult<(RuntimeTree, HashSet<String>)> {
        self.rtb.build()
    }
}
