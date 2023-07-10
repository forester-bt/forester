use crate::runtime::action::flow::{read_cursor, run_with, CURSOR, LEN, P_CURSOR};
use crate::runtime::action::keeper::ActionKeeper;
use crate::runtime::action::{decorator, flow, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::context::{RNodeState, TreeContext};
use crate::runtime::rtree::rnode::{FlowType, Name, RNode};
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RtOk, RtResult, RuntimeError, TickResult};
use crate::tracer::Tracer;
use crate::tree::project::Project;
use graphviz_rust::attributes::target;
use log::debug;
use std::path::PathBuf;

pub struct Forester {
    pub tree: RuntimeTree,
    pub bb: BlackBoard,
    pub keeper: ActionKeeper,
    pub tracer: Tracer,
}

impl Forester {
    pub(crate) fn new(
        tree: RuntimeTree,
        bb: BlackBoard,
        keeper: ActionKeeper,
        tracer: Tracer,
    ) -> RtResult<Self> {
        Ok(Self {
            tree,
            bb,
            keeper,
            tracer,
        })
    }

    pub fn run(&mut self) -> Tick {
        self.run_until(None)
    }

    pub fn run_until(&mut self, max_tick: Option<usize>) -> Tick {
        // The ctx has a call stack to manage the flow.
        // When the flow goes up it pops the current element and leaps to the parent.
        let mut ctx =
            TreeContext::new(&mut self.bb, &mut self.tracer, max_tick.unwrap_or_default());
        ctx.push(self.tree.root)?;
        // starts from root and pops up the element when either it is finished
        // or the root needs to make a new tick
        while let Some(id) = ctx.peek()? {
            let id = *id;
            debug!(target:"loop", "node = {}, tick = {}", id,ctx.curr_ts());
            match self.tree.node(&id)? {
                RNode::Flow(tpe, n, args, children) => match ctx.state_in_ts(id) {
                    // do nothing, since there are no children
                    RNodeState::Ready(tick_args) if children.is_empty() => {
                        debug!(target:"flow[ready]", "tick:{}, {tpe}. The children are empty. Go to the fin state", ctx.curr_ts());
                        ctx.new_state(id, RNodeState::Success(run_with(tick_args, 0, 0)))?;
                        ctx.pop()?;
                    }
                    // since it is ready we need to zero cursor for the children
                    // for some memory nodes we can switch it after.
                    // But then we do nothing but switch the state to running in the current tick.
                    RNodeState::Ready(tick_args) => {
                        let new_state =
                            RNodeState::Running(run_with(tick_args, 0, children.len() as i64));
                        debug!(target:"flow[ready]", "tick:{}, {tpe}. Just switch to the new_state:{:?}",ctx.curr_ts(),&new_state);
                        ctx.new_state(id, new_state)?;
                    }
                    // the flow can arrive here in 2 possible cases:
                    // - when we are about to start child
                    // - when we have the child finished (with any state)
                    // to figure out where we are, we analyze the child at the current cursor.
                    RNodeState::Running(tick_args) => {
                        let cursor = read_cursor_as_usize(tick_args.clone())?;
                        let child = children[cursor];
                        match ctx.state_in_ts(child) {
                            // we are about to kick off the child.
                            // Just pass the control to the child
                            RNodeState::Ready(..) => {
                                debug!(target:"flow[run]", "tick:{}, {tpe}. The '{child}' is ready, push it on the stack",ctx.curr_ts());
                                ctx.push(child)?;
                            }
                            // child is already running and since the flow is here in the parent,
                            // he decided that it is a final state for the tick,
                            // thus pass the control to the parent
                            RNodeState::Running(_) => {
                                // root does not have parent so, just proceed to the next tick
                                if tpe.is_root() {
                                    debug!(target:"flow[run]", "tick:{}, {tpe}. The '{child}' is running, tick up the flow. ",ctx.curr_ts());
                                    ctx.next_tick()?;
                                    ctx.push(child)?;
                                } else {
                                    let next_state =
                                        flow::monitor(tpe, args.clone(), tick_args, &mut ctx)?;
                                    debug!(target:"flow[run]", "tick:{}, {tpe}. Go up with the new state: {:?}",ctx.curr_ts(),&next_state);
                                    ctx.new_state(id, next_state)?;
                                    ctx.pop()?;
                                }
                            }
                            // child is finished, thus the node needs to make a decision how to proceed.
                            // this stage just updates the status and depending on the status,
                            // the flow goes further or stays on the node but on the next loop of while.
                            s @ (RNodeState::Failure(_) | RNodeState::Success(_)) => {
                                let new_state = flow::finalize(
                                    tpe,
                                    args.clone(),
                                    tick_args.clone(),
                                    s.clone().try_into()?,
                                    &mut ctx,
                                )?;
                                debug!(target:"flow[run]", "tick:{}, {tpe}. The '{}' is finished as {:?}, the new state: {:?} ",ctx.curr_ts(),child,s, &new_state);
                                ctx.new_state(id, new_state)?;
                            }
                        }
                    }
                    // the node is finished. pass the control further or if it is root,
                    // finish the whole procedure
                    RNodeState::Failure(_) | RNodeState::Success(_) => {
                        debug!(target:"flow[fin]", "tick:{},{tpe} gets popped up",ctx.curr_ts());
                        ctx.pop()?;
                    }
                },
                // similar to the flow except we don't need to handle more than 1 child.
                RNode::Decorator(tpe, init_args, child) => match ctx.state_in_ts(id) {
                    // since it is ready we need to prepare decorator to start
                    // But then we do nothing but switch the state to running in the current tick.
                    RNodeState::Ready(tick_args) => {
                        let new_state =
                            decorator::prepare(tpe, init_args.clone(), tick_args, &mut ctx)?;
                        debug!(target:"decorator[ready]", "tick:{}, the new_state: {:?}",ctx.curr_ts(),&new_state);
                        ctx.new_state(id, new_state)?;
                    }
                    // the flow can arrive here in 2 possible cases:
                    // - when we are about to start child
                    // - when we have the child finished (with any state)
                    // to figure out where we are, we analyze the child at the current cursor.
                    RNodeState::Running(tick_args) => match ctx.state_in_ts(*child) {
                        // we are about to kick off the child.
                        // Just pass the control to the child
                        RNodeState::Ready(..) => {
                            debug!(target:"decorator[run]", "tick:{}, The '{}' is ready, push it on the stack",ctx.curr_ts(),&child);
                            ctx.push(*child)?;
                        }
                        // child is already running and since the flow is here in the parent,
                        // he decided that it is a final state for the tick,
                        // thus pass the control to the parent
                        // we can use this to monitor the progress and make a decision
                        // (for Timeout for example)
                        RNodeState::Running { .. } => {
                            let new_state =
                                decorator::monitor(tpe, init_args.clone(), tick_args, &mut ctx)?;
                            debug!(target:"decorator[run]", "tick:{},The '{}' is running, the new state: {:?} ",ctx.curr_ts(),child, &new_state);
                            ctx.new_state(id, new_state)?;
                            ctx.pop()?;
                        }
                        // child is finished, thus the node needs to make a decision how to proceed.
                        // this stage just updates the status and depending on the status,
                        // the flow goes further or stays on the node but on the next loop of while.
                        s @ (RNodeState::Success(_) | RNodeState::Failure(_)) => {
                            let new_state = decorator::finalize(
                                tpe,
                                tick_args,
                                init_args.clone(),
                                s.to_tick_result()?,
                                &mut ctx,
                            )?;
                            debug!(target:"decorator[run]", "tick:{},The '{}' is finished, the new state: {:?} ",ctx.curr_ts(),child, &new_state);
                            ctx.new_state(id, new_state)?;
                            ctx.pop()?;
                        }
                    },
                    // the node is finished. pass the control further or if it is root,
                    // finish the whole procedure
                    RNodeState::Success(_) | RNodeState::Failure(_) => {
                        debug!(target:"decorator[fin]", "tick:{}, it gets popped up",ctx.curr_ts());
                        ctx.pop()?;
                    }
                },
                // The leaf nodes process atomically.
                RNode::Leaf(f_name, args) => {
                    debug!(target:"leaf","args :{:?}",args);
                    if ctx.state_in_ts(id).is_ready() {
                        let mut action = self.keeper.get(f_name.name()?)?;
                        let res = action.tick(args.clone(), &mut ctx)?;
                        let new_state = RNodeState::from(args.clone(), res);
                        debug!(target:"leaf", "tick:{}, the new state: {:?}",ctx.curr_ts(),&new_state);
                        ctx.new_state(id, new_state)?;
                    }
                    ctx.pop()?;
                }
            }
        }

        ctx.root_state(self.tree.root)
    }

    pub fn bb_dump(&self, file: PathBuf) -> RtOk {
        self.bb.dump(file)
    }
}

fn type_err(tpe: &str) -> Tick {
    Err(RuntimeError::uex(format!("the node {tpe} can't be a leaf")))
}
fn read_cursor_as_usize(args: RtArgs) -> RtResult<usize> {
    usize::try_from(read_cursor(args)?)
        .map_err(|e| RuntimeError::uex(format!("cursor is not usize")))
}
