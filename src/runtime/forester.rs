use crate::runtime::action::flow::handle_flow;
use crate::runtime::action::keeper::ActionKeeper;
use crate::runtime::action::{decorator, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::rtree::rnode::{FlowType, Name, RNode};
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RtOk, RtResult, RuntimeError, TickResult};
use crate::tree::project::Project;

pub struct Forester {
    pub tree: RuntimeTree,
    pub bb: BlackBoard,
    pub keeper: ActionKeeper,
}

impl Forester {
    pub fn new(tree: RuntimeTree, bb: BlackBoard, keeper: ActionKeeper) -> Self {
        Self { tree, bb, keeper }
    }

    pub fn perform(&mut self) -> Tick {
        let mut ctx = TreeContext::new(&mut self.bb);
        ctx.push(self.tree.root)?;

        while let Some(id) = ctx.peek()? {
            let id = *id;
            match self.tree.node(&id)? {
                RNode::Leaf(f_name, args) => {
                    if ctx.get_state_in_cts(id).is_ready() {
                        let mut action = self.keeper.get(f_name.name()?)?;
                        let tick_res = action.tick(args.clone(), &mut ctx)?;
                        ctx.new_state(id, RNodeState::leaf(tick_res))?;
                    }
                    ctx.pop()?;
                }
                RNode::Decorator(tpe, init_args, child) => match ctx.get_state_in_cts(id) {
                    RNodeState::Ready => {
                        let new_state = decorator::before(tpe, init_args.clone(), &mut ctx)?;
                        ctx.new_state(id, new_state)?;
                        ctx.push(*child)?;
                    }
                    RNodeState::Running { run_args, .. } => match ctx.get_state_in_cts(*child) {
                        RNodeState::Ready => {
                            ctx.push(*child)?;
                        }
                        RNodeState::Running { .. } => {
                            let new_state =
                                decorator::during(tpe, init_args.clone(), run_args, &mut ctx)?;
                            if new_state.is_running() {
                                ctx.push(*child)?;
                            }
                            ctx.new_state(id, new_state)?;
                        }
                        RNodeState::Finished { res, .. } | RNodeState::Leaf(res) => {
                            let new_state =
                                decorator::after(tpe, run_args, init_args.clone(), res, &mut ctx)?;
                            ctx.new_state(id, new_state)?;
                            ctx.pop()?;
                        }
                    },
                    RNodeState::Finished { .. } => {
                        ctx.pop()?;
                    }
                    RNodeState::Leaf(_) => return type_err("decorator"),
                },
                RNode::Flow(tpe, n, args, children) => match ctx.get_state_in_cts(id) {
                    RNodeState::Ready if children.is_empty() => {
                        ctx.new_state(
                            id,
                            RNodeState::Finished {
                                res: TickResult::Success,
                                cursor: 0,
                                len: 0,
                            },
                        )?;
                        ctx.pop()?;
                    }
                    RNodeState::Ready => {
                        ctx.new_state(id, RNodeState::run(children.len()))?;
                        ctx.push(children[0])?;
                    }
                    RNodeState::Running { cursor, len, .. } => {
                        let child = children[cursor];
                        match ctx.get_state_in_cts(child) {
                            RNodeState::Ready | RNodeState::Running { .. } => {
                                ctx.push(child)?;
                            }
                            RNodeState::Finished { res, .. } | RNodeState::Leaf(res) => {
                                let new_state =
                                    handle_flow(tpe, args.clone(), res, cursor, len, &mut ctx)?;
                                ctx.new_state(id, new_state)?;
                                ctx.pop()?;
                            }
                        }
                    }
                    RNodeState::Finished { .. } => {
                        ctx.pop()?;
                    }
                    RNodeState::Leaf(_) => return type_err("flow"),
                },
            }
        }

        ctx.root_state(self.tree.root)
    }
}

fn type_err(tpe: &str) -> Tick {
    Err(RuntimeError::uex(format!("the node {tpe} can't be a leaf")))
}
