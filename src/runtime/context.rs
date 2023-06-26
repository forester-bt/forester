use crate::runtime::action::Tick;
use crate::runtime::args::RtArgs;
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::rtree::rnode::RNodeId;
use crate::runtime::{RtOk, RtResult, RuntimeError, TickResult};
use graphviz_rust::attributes::bb;
use graphviz_rust::attributes::color_name::lawngreen;
use std::collections::{HashMap, VecDeque};

pub type Timestamp = usize;

pub struct TreeContext<'a> {
    bb: &'a BlackBoard,
    stack: VecDeque<RNodeId>,
    state: HashMap<RNodeId, RNodeState>,
    ts_map: HashMap<RNodeId, Timestamp>,
    curr_ts: Timestamp,
}

impl<'a> TreeContext<'a> {
    pub fn new(bb: &'a BlackBoard) -> Self {
        Self {
            bb,
            stack: Default::default(),
            state: Default::default(),
            ts_map: Default::default(),
            curr_ts: 1,
        }
    }
}

impl<'a> TreeContext<'a> {
    pub(crate) fn shift(&mut self) -> RtOk {
        self.curr_ts += 1;
        Ok(())
    }

    pub(crate) fn root_state(&self, root: RNodeId) -> Tick {
        self.state
            .get(&root)
            .ok_or(RuntimeError::uex(format!("the root node {root} is absent")))
            .and_then(|s| match s {
                RNodeState::Finished { res, .. } => Ok(res.clone()),
                s => Err(RuntimeError::uex(format!(
                    "the root is in unexpected state {:?}",
                    s
                ))),
            })
    }

    pub(crate) fn is_curr_ts(&self, id: &RNodeId) -> bool {
        self.ts_map
            .get(id)
            .map(|e| *e == self.curr_ts)
            .unwrap_or(false)
    }
    pub fn curr_ts(&self) -> RtResult<Timestamp> {
        Ok(self.curr_ts)
    }

    pub(crate) fn push(&mut self, id: RNodeId) -> RtOk {
        self.stack.push_back(id);
        self.state.insert(id, RNodeState::Ready);
        Ok(())
    }
    pub(crate) fn pop(&mut self) -> RtResult<Option<RNodeId>> {
        Ok(self.stack.pop_back())
    }
    pub(crate) fn peek(&self) -> RtResult<Option<&RNodeId>> {
        if self.stack.is_empty() {
            Ok(None)
        } else {
            Ok(self.stack.back())
        }
    }

    pub(crate) fn new_state(
        &mut self,
        id: RNodeId,
        state: RNodeState,
    ) -> RtResult<Option<RNodeState>> {
        self.ts_map.insert(id, self.curr_ts);
        Ok(self.state.insert(id, state))
    }
    pub(crate) fn get_state_in_cts(&self, id: RNodeId) -> RNodeState {
        if self.is_curr_ts(&id) {
            self.state
                .get(&id)
                .map(|s| s.clone())
                .unwrap_or(RNodeState::Ready)
        } else {
            RNodeState::Ready
        }
    }
}

pub type ChildIndex = usize;

#[derive(Clone, Debug)]
pub enum RNodeState {
    Ready,
    Running {
        run_args: RtArgs,
        cursor: ChildIndex,
        len: usize,
    },
    Finished {
        res: TickResult,
        cursor: ChildIndex,
        len: usize,
    },
    Leaf(TickResult),
}

impl RNodeState {
    pub fn leaf(tick_r: TickResult) -> RNodeState {
        RNodeState::Leaf(tick_r)
    }
    pub fn fin(res: TickResult, cursor: ChildIndex, len: usize) -> Self {
        RNodeState::Finished { res, cursor, len }
    }
}
impl RNodeState {
    pub(crate) fn run(children: usize) -> RNodeState {
        RNodeState::Running {
            cursor: 0,
            len: children,
            run_args: RtArgs::default(),
        }
    }
    pub(crate) fn running(cursor: ChildIndex, children: usize) -> RNodeState {
        RNodeState::Running {
            cursor,
            len: children,
            run_args: RtArgs::default(),
        }
    }

    pub fn is_running(&self) -> bool {
        match self {
            RNodeState::Running { .. } => true,
            _ => false,
        }
    }
    pub fn is_ready(&self) -> bool {
        match self {
            RNodeState::Ready => true,
            _ => false,
        }
    }
    pub fn is_finished(&self) -> bool {
        match self {
            RNodeState::Finished { .. } => true,
            _ => false,
        }
    }
}
