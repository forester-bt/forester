use crate::get_pb;
use crate::runtime::builder::ForesterBuilder;
use crate::runtime::env::RtEnv;
use crate::runtime::forester::Forester;
use crate::runtime::{RtResult, RuntimeError};
use crate::simulator::actions::SimAction;
use crate::simulator::config::SimProfile;
use crate::simulator::RtAction;
use crate::simulator::Simulator;
use crate::tracer::{Tracer, TracerConfiguration};
use std::fmt::format;
use std::path::{Path, PathBuf};

/// The builder to create a simulator process.
///
/// # Examples
/// ```no_run
/// use std::path::PathBuf;
/// use forester_rs::runtime::builder::ForesterBuilder;
/// use forester_rs::simulator::builder::SimulatorBuilder;
///
/// fn smoke() {
///     let mut sb = SimulatorBuilder::new();
///
///     let root = PathBuf::from("simulator/smoke");
///
///     sb.root(root.clone());
///     sb.profile(PathBuf::from("sim.yaml"));
///     
///     let mut fb = ForesterBuilder::from_file_system();
///
///     fb.main_file("main.tree".to_string());
///     fb.root(root);
///
///     sb.forester_builder(fb);
///     
///     let mut sim = sb.build().unwrap();
///     sim.run().unwrap();
/// }
///
/// fn smoke_from_text() {
///     let mut sb = SimulatorBuilder::new();
///
///     let sim = PathBuf::from("simulator/smoke/sim.yaml");
///     let mut fb = ForesterBuilder::from_text();
///     sb.profile(sim);
///     
///     fb.text(
///         r#"
/// import "std::actions"
///
/// root main sequence {
///     store_str("info1", "initial")
///     retryer(task(config = obj), success())
///     store_str("info2","finish")
/// }
///
/// fallback retryer(t:tree, default:tree){
///     retry(5) t(..)
///     fail("just should fail")
///     default(..)
/// }
///
/// impl task(config: object);
///     "#
///         .to_string(),
///     );    
///     sb.forester_builder(fb);
///     let mut sim = sb.build().unwrap();
///     sim.run().unwrap();
/// }
/// ```
#[derive(Default)]
pub struct SimulatorBuilder {
    profile: Option<PathBuf>,
    root: Option<PathBuf>,
    fb: Option<ForesterBuilder>,
}

impl SimulatorBuilder {
    pub fn new() -> Self {
        SimulatorBuilder {
            profile: None,
            fb: None,
            root: None,
        }
    }
    /// Add a simulator profile.
    pub fn profile(&mut self, profile: PathBuf) {
        self.profile = Some(profile);
    }

    /// Add an ForesterBuilder instance.
    /// Use `ForesterBuilder` for that.
    pub fn forester_builder(&mut self, fb: ForesterBuilder) {
        self.fb = Some(fb);
    }
    /// Add the root directory.
    pub fn root(&mut self, root: PathBuf) {
        self.root = Some(root);
    }

    /// Build
    pub fn build(&mut self) -> RtResult<Simulator> {
        let mut fb = self
            .fb
            .take()
            .ok_or(RuntimeError::uex(format!("the forester builder is absent")))?;

        let profile = if let Some(p) = &self.profile {
            SimProfile::parse_file(&get_pb(p, &self.root.clone())?)?
        } else {
            SimProfile::default()
        };

        let pr = profile.clone();

        if let Some(trace_dump_file) = profile.config.trace {
            fb.tracer(Tracer::create(TracerConfiguration::in_file(get_pb(
                &PathBuf::from(trace_dump_file),
                &self.root.clone(),
            )?))?)
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
        Ok(Simulator::new(self.root.take(), pr, forester))
    }
}
