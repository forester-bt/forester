pub mod decorator;
pub mod flow;
pub mod serv;

use crate::runtime::action::keeper::ActionKeeper;
use crate::runtime::action::{recover, Tick};
use crate::runtime::args::RtArgs;
use crate::runtime::blackboard::BlackBoard;
use crate::runtime::context::{RNodeState, TreeContext, TreeContextRef};
use crate::runtime::env::RtEnv;
use crate::runtime::forester::flow::{read_cursor, run_with, run_with_par, FlowDecision};
use crate::runtime::forester::serv::ServInfo;
use crate::runtime::rtree::rnode::RNode;
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::trimmer::task::TrimTask;
use crate::runtime::trimmer::validator::TrimValidationResult;
use crate::runtime::trimmer::{RequestBody, TreeSnapshot, TrimRequest, TrimmingQueue};
use crate::runtime::{trimmer, RtOk, RtResult, RuntimeError};
use crate::tracer::{Event, Tracer};
use log::debug;
use std::sync::{Arc, Mutex};
use tokio::task::JoinHandle;

/// The entry point to process execution.
/// It works with the following components:
/// - Runtime tree to get the current element of the parsing tree.
/// - Blackboard as a memory layer
/// - ActionKeeper to execute the actions
/// - Tracer to store the tracing information
/// - Optimizer holds tasks to modify the tree or other components on the fly
///
///# Note:
/// Better to use `ForesterBuilder` to create a Forester.
pub struct Forester {
    pub tree: RuntimeTree,
    pub bb: Arc<Mutex<BlackBoard>>,
    pub tracer: Arc<Mutex<Tracer>>,
    pub keeper: ActionKeeper,
    pub env: Arc<Mutex<RtEnv>>,
    pub trimmer: Arc<Mutex<TrimmingQueue>>,
    serv: Option<ServInfo>,
}

impl Forester {
    pub(crate) fn new(
        tree: RuntimeTree,
        bb: Arc<Mutex<BlackBoard>>,
        tracer: Arc<Mutex<Tracer>>,
        keeper: ActionKeeper,
        env: Arc<Mutex<RtEnv>>,
        serv: Option<ServInfo>,
    ) -> RtResult<Self> {
        let trimmer = Arc::new(Mutex::new(TrimmingQueue::default()));
        Ok(Self {
            tree,
            bb,
            keeper,
            tracer,
            env,
            trimmer,
            serv,
        })
    }

    /// The function to trim the tree or perform other procedures.
    /// Initially, the intention is to have an ability to change some components of the current execution on a fly.
    /// The trimming procedure performs only one task in a tick. Others are either declined or postponed.
    ///
    /// The task can decide to decline or postpone itself.
    fn trim(&mut self, ctx: &TreeContext) -> RtOk {
        let mut mq_guard = self.trimmer.lock()?;
        debug!(
            target:"trim","trying to trim the tree, the number of tasks in the q is {}",
            mq_guard.len()
        );
        // the tasks that have decided to skip this tick or they have not passed the validations.
        let mut deferred_tasks = vec![];
        'top: while let Some(t) = mq_guard.pop() {
            let snapshot = TreeSnapshot::new(
                ctx.curr_ts(),
                self.bb.clone(),
                self.tracer.clone(),
                &self.tree,
                &ctx.state(),
                self.keeper.actions(),
            );
            match t.process(snapshot.clone())? {
                TrimRequest::Reject => {
                    debug!(target:"trim","a trimming request has rejected by itself");
                }
                TrimRequest::Skip => {
                    debug!(target:"trim","a trimming request has decided to skip this tick");
                    deferred_tasks.push(t);
                }
                TrimRequest::Attempt(r) => {
                    debug!(target:"trim","validating the trim request {:?}", r);
                    match trimmer::validator::validate(&snapshot, &r) {
                        TrimValidationResult::Defer(reason) => {
                            debug!(target:"trim","the request is deferred, The reason is '{reason}'");
                        }
                        TrimValidationResult::Reject(reason) => {
                            debug!(target:"trim","the request is rejected, The reason is '{reason}'")
                        }
                        TrimValidationResult::Proceed => {
                            let RequestBody { tree_b, actions } = r;
                            for (nid, node) in tree_b.nodes {
                                let new = format!("{:?}", node);
                                let old = self.tree.nodes.insert(nid, node);
                                debug!(target:"trim","The node {nid} is replaced. The previous node {:?}, the new node {new}", old);
                                self.tracer.lock()?.trace(
                                    ctx.curr_ts(),
                                    Event::Trim(nid.clone(), format!("{:?} >>> {new}", old)),
                                )?;
                            }
                            for (an, action) in actions {
                                self.keeper.register(an, action)?;
                            }
                            break 'top;
                        }
                    }
                }
            }
        }

        mq_guard.push_all(deferred_tasks);
        Ok(())
    }

    pub fn add_trim_task(&mut self, task: TrimTask) -> RtResult<JoinHandle<RtOk>> {
        let arc = self.trimmer.clone();
        let env = &self.env.lock()?;
        Ok(env.runtime.spawn(async move {
            arc.lock()
                .map(|mut t| t.push(task))
                .map_err(|e| RuntimeError::TrimmingError(e.to_string()))
        }))
    }

    /// Runs the execution.
    /// Traverse the tree either until the root transits into either Failure or Success or some Exception will be thrown.
    pub fn run(&mut self) -> Tick {
        self.run_until(None)
    }

    /// Runs the execution but with the limitation on the ticks
    /// Traverse the tree either until the root transits into either Failure or Success
    /// or some Exception will be thrown or the limit on ticks is exceeded.
    pub fn run_until(&mut self, max_tick: Option<usize>) -> Tick {
        // The ctx has a call stack to manage the flow.
        // When the flow goes up it pops the current element and leaps to the parent.
        let mut ctx = TreeContext::new(
            self.bb.clone(),
            self.tracer.clone(),
            max_tick.unwrap_or_default(),
            self.env.clone(),
        );
        ctx.push(self.tree.root)?;
        // starts from root and pops up the element when either it is finished
        // or the root needs to make a new tick
        while let Some(id) = ctx.peek()? {
            let id = *id;
            debug!(target:"loop", "node = {}, tick = {}", id,ctx.curr_ts());
            match self.tree.node(&id)? {
                RNode::Flow(tpe, _n, init_args, children) => match ctx.state_in_ts(&id) {
                    // do nothing, since there are no children
                    RNodeState::Ready(tick_args) if children.is_empty() => {
                        debug!(target:"flow[ready]", "tick:{}, {tpe}. The children are empty. Go to the fin state", ctx.curr_ts());
                        ctx.new_state(id, RNodeState::Success(run_with(tick_args, 0, 0)))?;
                        ctx.pop()?;
                    }
                    // since it is ready we need to zero cursor for the children
                    // for some memory nodes we can switch it after.
                    // But then we do nothing but switch the state to running in the current tick.
                    // the parallel case is processed a bit differently since we need to memorize the children state, see run_with_par
                    RNodeState::Ready(tick_args) => {
                        let len = children.len() as i64;
                        debug!(target:"flow[ready]", "tick:{}, {tpe}. Start node",ctx.curr_ts());
                        let new_state = if tpe.is_par() {
                            RNodeState::Running(run_with_par(tick_args, len))
                        } else {
                            RNodeState::Running(run_with(tick_args, 0, len))
                        };

                        debug!(target:"flow[ready]", "tick:{}, {tpe}. Switch to the new_state:{}",ctx.curr_ts(),&new_state);
                        ctx.new_state(id, new_state)?;
                    }
                    // the flow can arrive here in 2 possible cases:
                    // - when we are about to start child
                    // - when we have the child finished (with any state)
                    // to figure out where we are, we analyze the child at the current cursor.
                    RNodeState::Running(tick_args) => {
                        let cursor = read_cursor_as_usize(tick_args.clone())?;
                        let child = children[cursor];
                        debug!(target:"flow[run]", "tick:{}, {tpe}. Running child {child}, cursor:{cursor}",ctx.curr_ts());
                        match ctx.state_in_ts(&child) {
                            // we are about to kick off the child.
                            // Just pass the control to the child
                            RNodeState::Ready(..) => {
                                debug!(target:"flow[run]", "tick:{}, {tpe}. The '{child}' is ready, push it on the stack",ctx.curr_ts());
                                ctx.push(child)?;
                            }
                            // child is already running and since the flow is here in the parent,
                            // he decided that it is a final state for the tick,
                            // thus pass the control to the parent.
                            // here, we need to decide whether we need to go up or not.
                            // In the most cases, we go up by popping the current element.
                            // Only with parallel nodes, we need to check if we have other elements to kick off
                            RNodeState::Running(_) => {
                                // root does not have parent so, just proceed to the next tick
                                if tpe.is_root() {
                                    debug!(target:"flow[run]", "tick:{}, {tpe}. The '{child}' is running, tick up the flow. ",ctx.curr_ts());
                                    ctx.next_tick()?;
                                    debug!(target:"trim","attempt to trim is  {:?}", self.trim(&ctx));
                                    ctx.push(child)?;
                                } else {
                                    debug!(target:"flow[run]", "tick:{}, {tpe}. The '{child}' is running, decide to go up or stay here.",ctx.curr_ts());
                                    // for parallel node we need to proceed with other children regardless of the current result
                                    match flow::monitor(
                                        tpe,
                                        init_args.clone(),
                                        tick_args,
                                        &mut ctx,
                                    )? {
                                        FlowDecision::PopNode(ns) => {
                                            debug!(target:"flow[run]", "tick:{}, {tpe}. Go up with the new state: {}",ctx.curr_ts(),&ns);
                                            ctx.new_state(id, ns)?;
                                            ctx.pop()?;
                                        }
                                        FlowDecision::Stay(ns) => {
                                            debug!(target:"flow[run]", "tick:{}, {tpe}. stay with the new state: {}",ctx.curr_ts(),&ns);
                                            ctx.new_state(id, ns)?;
                                        }
                                        FlowDecision::Halt(new_state, halting_child_cursor) => {
                                            // A reactively-checked child returning running will halt any other running child.
                                            // Pop ourselves then halt the previously running child.
                                            let halting_child_id = children[halting_child_cursor];
                                            debug!(target:"flow[run]", "tick:{}, {tpe}. Reactively checked child '{child}' has returned running. Halting previously running child '{halting_child_id}', then go up with new state: {}.", ctx.curr_ts(), new_state);

                                            ctx.new_state(id, new_state)?;
                                            ctx.pop()?;

                                            ctx.force_to_halting_state(halting_child_id)?;
                                            ctx.push(halting_child_id)?;
                                        }
                                    }
                                }
                            }
                            // Halting must be an atomic process, it is never spread over multiple ticks so should never be seen in this flow.
                            RNodeState::Halting(_) => {
                                return Err(RuntimeError::Unexpected(
                                    "A flow node child should never return a state of Halting."
                                        .to_string(),
                                ));
                            }
                            // child is finished, thus the node needs to make a decision how to proceed.
                            // this stage just updates the status and depending on the status,
                            // the flow goes further or stays on the node but on the next loop of while.
                            s @ (RNodeState::Failure(_) | RNodeState::Success(_)) => {
                                debug!(target:"flow[run]", "tick:{}, {tpe}. The '{child}' is finished, decide to go up or stay here.",ctx.curr_ts());
                                let decision = flow::finalize(
                                    tpe,
                                    init_args.clone(),
                                    tick_args.clone(),
                                    s.clone().try_into()?,
                                    &mut ctx,
                                )?;

                                match decision {
                                    FlowDecision::PopNode(ns) => {
                                        debug!(target:"flow[run]", "tick:{}, {tpe}. The '{}' is finished as {}, the new state: {}. Pop the node.",ctx.curr_ts(),child,s, &ns);
                                        ctx.new_state(id, ns)?;
                                        ctx.pop()?;
                                    }
                                    FlowDecision::Stay(ns) => {
                                        debug!(target:"flow[run]", "tick:{}, {tpe}. The '{}' is finished as {}, the new state: {}. Stay at this node. ",ctx.curr_ts(),child,s, &ns);
                                        ctx.new_state(id, ns)?;
                                    }
                                    FlowDecision::Halt(new_state, halting_child_cursor) => {
                                        // Normally we would fall through to Failure or Success next tick (e.g. Stay decision), but we need to pass control to the child so it can halt properly.
                                        // The current node can continue as normal once the child is halted.
                                        ctx.new_state(id, new_state.clone())?;
                                        // Force the state of the child to be halting, so it will interrupt itself on the next tick.
                                        let halting_child_id = children[halting_child_cursor];
                                        debug!(target:"flow[run]", "tick:{}, {tpe}. The '{}' is finished as {}, the new state: {}. Halting child '{}'. ",ctx.curr_ts(),child,s, &new_state, &halting_child_id);
                                        ctx.force_to_halting_state(halting_child_id)?;
                                        ctx.push(halting_child_id)?;
                                    }
                                }
                            }
                        }
                    }
                    // The node's parent has commanded us to halt.
                    RNodeState::Halting(tick_args) => {
                        debug!(target:"flow[halt]", "tick:{}, {tpe}. Checking for running children to halt.",ctx.curr_ts());
                        let (new_state, halting_child_cursor) = flow::halt(tpe, tick_args.clone());
                        // Halting is a one-way process, pop ourselves then push any halting children.
                        ctx.new_state(id, new_state)?;
                        ctx.pop()?;
                        if let Some(halting_child_cursor) = halting_child_cursor {
                            let halting_child_id = children[halting_child_cursor];
                            ctx.force_to_halting_state(halting_child_id)?;
                            ctx.push(halting_child_id)?;
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
                RNode::Decorator(tpe, init_args, child) => match ctx.state_in_ts(&id) {
                    // since it is ready we need to prepare decorator to start
                    // But then we do nothing but switch the state to running in the current tick.
                    RNodeState::Ready(tick_args) => {
                        let prev_state = ctx.state_last_set(&id);
                        debug!(target:"decorator[ready]", "tick:{}, {tpe}. Start decorator({init_args}), child args({tick_args}) and prev state({prev_state})",ctx.curr_ts());
                        // Decorators only prepare themselves if they didn't return Running in the previous tick
                        let new_state = match prev_state {
                            RNodeState::Running(_) => {
                                RNodeState::Running(run_with(tick_args, 0, 1))
                            }
                            _ => decorator::prepare(tpe, init_args.clone(), tick_args, &mut ctx)?,
                        };
                        debug!(target:"decorator[ready]", "tick:{}, the new_state: {}",ctx.curr_ts(),&new_state);
                        ctx.new_state(id, new_state)?;
                    }
                    // the flow can arrive here in 2 possible cases:
                    // - when we are about to start child
                    // - when we have the child finished (with any state)
                    // to figure out where we are, we analyze the child at the current cursor.
                    RNodeState::Running(tick_args) => match ctx.state_in_ts(child) {
                        // we are about to kick off the child.
                        // Just pass the control to the child
                        RNodeState::Ready(..) => {
                            debug!(target:"decorator[run]", "tick:{}, The decorator({init_args}) has the '{}' ready, push it on the stack",ctx.curr_ts(),&child);
                            ctx.push(*child)?;
                        }
                        // child is already running and since the flow is here in the parent,
                        // he decided that it is a final state for the tick,
                        // thus pass the control to the parent
                        // we can use this to monitor the progress and make a decision
                        // (for Timeout for example)
                        RNodeState::Running { .. } => {
                            debug!(target:"decorator[run]", "tick:{}, {tpe}. Running decorator",ctx.curr_ts());
                            let new_state =
                                decorator::monitor(tpe, init_args.clone(), tick_args, &mut ctx)?;
                            debug!(target:"decorator[run]", "tick:{},The '{}' is running, the new state: {} ",ctx.curr_ts(),child, &new_state);
                            ctx.new_state(id, new_state)?;
                            ctx.pop()?;
                        }
                        // Halting must be an atomic process, it is never spread over multiple ticks so should never be seen in this flow.
                        RNodeState::Halting(_) => {
                            return Err(RuntimeError::Unexpected(
                                "A decorator node child should never return a state of Halting."
                                    .to_string(),
                            ));
                        }
                        // child is finished, thus the node needs to make a decision how to proceed.
                        // this stage just updates the status and depending on the status,
                        // the flow goes further or stays on the node but on the next loop of while.
                        s @ (RNodeState::Success(_) | RNodeState::Failure(_)) => {
                            debug!(target:"decorator[run]", "tick:{}, {tpe}. Running a child of the decorator({tick_args}), child args({})",ctx.curr_ts(),s.args());
                            let new_state = decorator::finalize(
                                tpe,
                                tick_args,
                                init_args.clone(),
                                s.to_tick_result()?,
                                &mut ctx,
                            )?;
                            debug!(target:"decorator[run]", "tick:{},The '{}' is finished, the new state: {} ",ctx.curr_ts(),child, &new_state);
                            ctx.new_state(id, new_state)?;
                            ctx.pop()?;
                        }
                    },
                    // The node's parent has commanded us to halt.
                    // The only resetting a decorator needs to do is trigger a prepare next time it's ticked and halt its child.
                    RNodeState::Halting(_) => {
                        debug!(target:"decorator[halt]", "tick:{}, {tpe}. Halting child.",ctx.curr_ts());
                        // Halting is a one-way process, pop ourselves then halt and push the child.
                        ctx.new_state(id, RNodeState::Ready(ctx.state_last_set(&id).args()))?;
                        ctx.pop()?;
                        ctx.force_to_halting_state(*child)?;
                        ctx.push(*child)?;
                    }
                    // the node is finished. pass the control further or if it is root,
                    // finish the whole procedure
                    RNodeState::Success(_) | RNodeState::Failure(_) => {
                        debug!(target:"decorator[fin]", "tick:{}, The decorator({init_args}) has a child finished. it gets popped up",ctx.curr_ts());
                        ctx.pop()?;
                    }
                },
                // The leaf nodes process atomically.
                RNode::Leaf(f_name, args) => match ctx.state_last_set(&id) {
                    RNodeState::Halting(tick_args) => {
                        debug!(target:"leaf[halt]", "tick:{}, args :{:?}",ctx.curr_ts(), args);
                        let ctx_ref = TreeContextRef::from_ctx(&ctx, self.trimmer.clone());
                        self.keeper.halt(
                            self.env.clone(),
                            f_name.name()?,
                            args.clone(),
                            ctx_ref,
                            &self.serv,
                        )?;
                        ctx.new_state(id, RNodeState::Ready(tick_args))?;
                        ctx.pop()?;
                    }
                    _ => {
                        debug!(target:"leaf[run]","tick:{}, args :{:?}",ctx.curr_ts(), args);
                        if ctx.state_in_ts(&id).is_ready() {
                            let ctx_ref = TreeContextRef::from_ctx(&ctx, self.trimmer.clone());
                            let res = recover(self.keeper.on_tick(
                                self.env.clone(),
                                f_name.name()?,
                                args.clone(),
                                ctx_ref,
                                &self.serv,
                            ))?;
                            let new_state = RNodeState::from(args.clone(), res);
                            debug!(target:"leaf", "tick:{}, the new state: {}",ctx.curr_ts(),&new_state);
                            ctx.new_state(id, new_state)?;
                        }
                        ctx.pop()?;
                    }
                },
            }
        }
        // clean up the tree
        self.stop_http();
        self.env.lock().map(|mut e| e.stop_all_daemons())?;

        ctx.root_state(self.tree.root)
    }

    /// stops the http server
    pub fn stop_http(&mut self) {
        if let Some(serv) = self.serv.take() {
            match serv.stop() {
                Ok(_) => {
                    debug!(target:"forester","Tree is finished. The server for remote actions is stopped");
                }
                Err(_) => {
                    debug!(target:"forester", "Tree is finished. The server for remote actions is stopped with an error");
                }
            }
        }
    }
}

fn read_cursor_as_usize(args: RtArgs) -> RtResult<usize> {
    usize::try_from(read_cursor(args)?)
        .map_err(|_e| RuntimeError::uex("cursor is not usize".to_string()))
}
