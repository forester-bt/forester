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
    bb: &'a mut BlackBoard,
    stack: VecDeque<RNodeId>,
    state: HashMap<RNodeId, RNodeState>,
    ts_map: HashMap<RNodeId, Timestamp>,
    curr_ts: Timestamp,
}

impl<'a> TreeContext<'a> {
    pub fn bb(&mut self) -> &mut BlackBoard {
        self.bb
    }
    pub fn new(bb: &'a mut BlackBoard) -> Self {
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
    pub(crate) fn next_tick(&mut self) -> RtOk {
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
    pub(crate) fn state_in_ts(&self, id: RNodeId) -> RNodeState {
        let actual_state = self
            .state
            .get(&id)
            .map(|s| s.clone())
            .unwrap_or(RNodeState::Ready(RtArgs::default()));
        if self.is_curr_ts(&id) {
            actual_state
        } else {
            RNodeState::Ready(actual_state.tick_args())
        }
    }
}

pub type ChildIndex = usize;

#[derive(Clone, Debug)]
pub enum RNodeState {
    Ready(RtArgs),
    Running { tick_args: RtArgs },
    Finished { res: TickResult, tick_args: RtArgs },
    Leaf(TickResult),
}

impl RNodeState {
    pub fn tick_args(&self) -> RtArgs {
        match self {
            RNodeState::Ready(tick_args) => tick_args.clone(),
            RNodeState::Running { tick_args, .. } => tick_args.clone(),
            RNodeState::Finished { tick_args, .. } => tick_args.clone(),
            RNodeState::Leaf(_) => RtArgs::default(),
        }
    }

    pub(crate) fn leaf(tick_r: TickResult) -> RNodeState {
        RNodeState::Leaf(tick_r)
    }
    pub(crate) fn fin(res: TickResult, tick_args: RtArgs) -> Self {
        RNodeState::Finished { res, tick_args }
    }
    pub(crate) fn run(tick_args: RtArgs) -> RNodeState {
        RNodeState::Running { tick_args }
    }
}
impl RNodeState {
    pub fn is_running(&self) -> bool {
        match self {
            RNodeState::Running { .. } => true,
            _ => false,
        }
    }
    pub fn is_ready(&self) -> bool {
        match self {
            RNodeState::Ready(_) => true,
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
