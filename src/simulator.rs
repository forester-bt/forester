use crate::runtime::action::Action as RtAction;
use crate::runtime::action::Tick;
use crate::runtime::builder::ForesterBuilder;
use crate::runtime::forester::Forester;
use crate::runtime::{RtOk, RtResult};
use crate::simulator::actions::SimAction;
use crate::simulator::config::{Action, SimProfile, SimProfileConfig};
use crate::tracer::{Tracer, TracerConfiguration};
use crate::visualizer::Visualizer;
use std::path::{Path, PathBuf};
pub mod actions;
pub mod builder;
pub mod config;

pub struct Simulator {
    pub root: PathBuf,
    pub profile: SimProfile,
    pub forester: Forester,
}

impl Simulator {
    pub fn build(
        profile: SimProfile,
        root: PathBuf,
        main_file: String,
        main_tree: Option<String>,
    ) -> RtResult<Self> {
        let mut fb = ForesterBuilder::new();
        let pr = profile.clone();
        fb.root(root.clone());
        fb.main_file(main_file);

        if let Some(m_tree) = main_tree {
            fb.main_tree(m_tree);
        }

        if let Some(trace_dump_file) = profile.config.trace {
            fb.tracer(Tracer::create(TracerConfiguration::in_file(get_pb(
                &trace_dump_file,
                root.clone(),
            )))?)
        }

        if let Some(bb_load_path) = profile.config.bb.load {
            fb.bb_load(bb_load_path);
        }

        for action in profile.actions.iter() {
            fb.register_action(
                action.name.as_str(),
                RtAction::sync(SimAction::create(
                    action.stub.as_str(),
                    action
                        .params
                        .get("delay")
                        .map(|s| s.parse::<usize>().unwrap_or_default())
                        .unwrap_or_default(),
                )?),
            )
        }

        let forester = fb.build()?;

        Ok(Simulator {
            root,
            profile: pr,
            forester,
        })
    }

    pub fn run(&mut self) -> Tick {
        let cfg = &self.profile.config;
        let max = cfg.max_ticks;

        if let Some(viz_file) = &cfg.graph {
            let tree = &self.forester.tree;
            Visualizer::svg_file(tree, get_pb(viz_file, self.root.clone()))?;
        }

        let result = self.forester.run_until(max);

        if let Some(bb_dump) = &cfg.bb.dump {
            self.forester.bb.dump(get_pb(bb_dump, self.root.clone()))?;
        }

        result
    }
}

pub(crate) fn get_pb(file: &String, root: PathBuf) -> PathBuf {
    let file_pb = PathBuf::from(file);
    if file_pb.is_relative() {
        let mut full_path = root;
        full_path.push(file_pb);
        full_path
    } else {
        file_pb
    }
}
