use crate::runtime::action::builtin::data::{CheckEq, LockUnlockBBKey, StoreData, StoreTick};
use crate::runtime::action::builtin::http::HttpGet;
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::action::keeper::ActionKeeper;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::builder::builtin::BuilderBuiltInActions;
use crate::runtime::builder::CommonForesterBuilder;
use crate::runtime::env::RtEnv;
use crate::runtime::forester::Forester;
use crate::runtime::rtree::builder::{RtNodeBuilder, RtTreeBuilder};
use crate::runtime::rtree::rnode::{RNodeId, RNodeName};
use crate::runtime::rtree::{RuntimeTree, RuntimeTreeStarter};
use crate::runtime::{RtResult, RuntimeError};
use crate::tracer::Tracer;
use crate::tree::project::{FileName, Project, TreeName};
use std::collections::{HashMap, HashSet};
use std::fmt::format;
use std::path::{Path, PathBuf};

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
    pub fn build(self) -> (RuntimeTree, HashSet<String>) {
        self.rtb.build()
    }
}
