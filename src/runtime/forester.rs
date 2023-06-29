use crate::runtime::action::flow::{read_cursor, tick_run_with, CURSOR, LEN, P_CURSOR};
use crate::runtime::action::keeper::ActionKeeper;
use crate::runtime::action::{decorator, flow, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::rtree::rnode::{FlowType, Name, RNode};
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RtOk, RtResult, RuntimeError, TickResult};
use crate::tree::project::Project;
use tracing::{debug, info};

pub struct Forester {
    pub tree: RuntimeTree,
    pub bb: BlackBoard,
    pub keeper: ActionKeeper,
}

impl Forester {
    pub fn new(tree: RuntimeTree, bb: BlackBoard, keeper: ActionKeeper) -> Self {
        Self { tree, bb, keeper }
    }
    /// Main loop executing the code
    pub fn start(&mut self) -> Tick {
        let mut ctx = TreeContext::new(&mut self.bb);
        ctx.push(self.tree.root)?;

        while let Some(id) = ctx.peek()? {
            let id = *id;
            match self.tree.node(&id)? {
                RNode::Flow(tpe, n, args, children) => match ctx.state_in_ts(id) {
                    RNodeState::Ready(tick_args) if children.is_empty() => {
                        ctx.new_state(
                            id,
                            RNodeState::fin(TickResult::Success, tick_run_with(tick_args, 0, 0)),
                        )?;
                        ctx.pop()?;
                    }
                    RNodeState::Ready(tick_args) => {
                        ctx.new_state(
                            id,
                            RNodeState::run(
                                tick_args
                                    .with(CURSOR, RtValue::int(0))
                                    .with(LEN, RtValue::int(children.len() as i64)),
                            ),
                        )?;
                    }
                    RNodeState::Running { tick_args } => {
                        let cursor = read_cursor_as_usize(tick_args.clone())?;
                        let child = children[cursor];
                        match ctx.state_in_ts(child) {
                            RNodeState::Ready(..) => {
                                ctx.push(child)?;
                            }
                            RNodeState::Running { .. } => {
                                if tpe.is_root() {
                                    ctx.next_tick()?;
                                    ctx.push(child)?;
                                } else {
                                    ctx.pop()?;
                                }
                            }
                            RNodeState::Finished { res, .. } | RNodeState::Leaf(res) => {
                                let new_state = flow::process(
                                    tpe,
                                    args.clone(),
                                    tick_args.clone(),
                                    res,
                                    &mut ctx,
                                )?;
                                ctx.new_state(id, new_state)?;
                            }
                        }
                    }
                    RNodeState::Finished { .. } => {
                        ctx.pop()?;
                    }
                    RNodeState::Leaf(_) => return type_err("flow"),
                },
                RNode::Decorator(tpe, init_args, child) => match ctx.state_in_ts(id) {
                    RNodeState::Ready(tick_args) => {
                        let new_state =
                            decorator::before(tpe, init_args.clone(), tick_args, &mut ctx)?;
                        ctx.new_state(id, new_state)?;
                    }
                    RNodeState::Running { tick_args, .. } => match ctx.state_in_ts(*child) {
                        RNodeState::Ready(..) => {
                            ctx.push(*child)?;
                        }
                        RNodeState::Running { .. } => {
                            let new_state =
                                decorator::during(tpe, init_args.clone(), tick_args, &mut ctx)?;
                            ctx.new_state(id, new_state)?;
                            ctx.pop()?;
                        }
                        RNodeState::Finished { res, .. } | RNodeState::Leaf(res) => {
                            let new_state =
                                decorator::after(tpe, tick_args, init_args.clone(), res, &mut ctx)?;
                            ctx.new_state(id, new_state)?;
                            ctx.pop()?;
                        }
                    },
                    RNodeState::Finished { .. } => {
                        ctx.pop()?;
                    }
                    RNodeState::Leaf(_) => return type_err("decorator"),
                },
                RNode::Leaf(f_name, args) => {
                    if ctx.state_in_ts(id).is_ready() {
                        let mut action = self.keeper.get(f_name.name()?)?;
                        let tick_res = action.tick(args.clone(), &mut ctx)?;
                        ctx.new_state(id, RNodeState::leaf(tick_res))?;
                    }
                    ctx.pop()?;
                }
            }
        }

        ctx.root_state(self.tree.root)
    }
}

fn type_err(tpe: &str) -> Tick {
    Err(RuntimeError::uex(format!("the node {tpe} can't be a leaf")))
}
fn read_cursor_as_usize(args: RtArgs) -> RtResult<usize> {
    usize::try_from(read_cursor(args)?)
        .map_err(|e| RuntimeError::uex(format!("cursor is not usize")))
}
