pub mod config;

use crate::{
    get_pb,
    runner::config::{RemoteAction, RunProfile, RunUntil, ServerAPI},
    runtime::{
        action::builtin::remote::RemoteHttpAction, action::Tick, builder::ForesterBuilder,
        forester::Forester, RtResult, RuntimeError,
    },
    tracer::{Tracer, TracerConfig},
};
use std::path::PathBuf;

/// Runner is a wrapper above [`Forester`] that runs a tree from the file system
/// configured with a [`RunProfile`].
///
/// The profile drives the blackboard (initial load and final dump), the tracer,
/// the http server and the remote actions, so the tree can be run
/// without coding the setup, just switching the profiles.
///
/// # Example
/// ```no_run
/// use std::path::PathBuf;
/// use forester_rs::runner::Runner;
/// use forester_rs::runner::config::RunProfile;
///
/// fn smoke() {
///     let profile = RunProfile::from_file("runner/smoke/profile.yaml").unwrap();
///     let mut runner = Runner::build(
///         PathBuf::from("runner/smoke/main.tree"),
///         "main".to_string(),
///         profile,
///     )
///     .unwrap();
///
///     runner.run().unwrap();
/// }
/// ```
pub struct Runner {
    /// The root folder of the project,
    /// the relative paths in the profile are resolved against it.
    pub root: PathBuf,
    /// The profile the runner was built with.
    pub profile: RunProfile,
    /// The underlying Forester instance.
    pub forester: Forester,
}

impl Runner {
    /// Builds the runner from the main tree file and the profile.
    ///
    /// The `main_file` path is split into the root folder (the parent directory)
    /// and the file name that are passed to [`ForesterBuilder`];
    /// `main` is the name of the root tree in that file.
    ///
    /// From the profile:
    /// - `tracer` sets up the [`Tracer`];
    /// - `bb.load` loads the initial blackboard data from the file before the start;
    /// - `api` starts the http server for the remote actions;
    /// - `actions` registers the remote actions by name.
    pub fn build(main_file: PathBuf, main: String, profile: RunProfile) -> RtResult<Self> {
        let file = main_file
            .file_name()
            .and_then(|f| f.to_str())
            .map(|f| f.to_string())
            .ok_or(RuntimeError::IOError(format!(
                "the path {} does not point to a file",
                main_file.display()
            )))?;
        let root = main_file
            .parent()
            .map(|p| p.to_path_buf())
            .unwrap_or_default();

        let mut fb = ForesterBuilder::from_fs();
        fb.root(root.clone());
        fb.main_file(file);
        fb.main_tree(main);

        if let Some(tracer_cfg) = &profile.tracer_config {
            let mut cfg = TracerConfig::default();
            if let Some(indent) = tracer_cfg.indent {
                cfg.indent = indent;
            }
            cfg.time_format = tracer_cfg.time_format.clone();
            cfg.to_file = tracer_cfg
                .to_file
                .as_ref()
                .map(|f| get_pb(f, &Some(root.clone())))
                .transpose()?;
            fb.tracer(Tracer::create(cfg)?);
        }

        if let Some(load) = profile.bb_config.as_ref().and_then(|bb| bb.load.clone()) {
            fb.bb_load(load);
        }

        if profile.server_api.is_some() {
            let ServerAPI::Http { host, port } = profile.server_api.clone().unwrap_or_default();
            fb.http_serv(
                host.unwrap_or_else(|| "127.0.0.1".to_string()),
                port.unwrap_or(0),
            );
        }

        for action in profile.actions.iter() {
            match action {
                RemoteAction::HttpAction { name, url } => {
                    fb.register_remote_action(name, RemoteHttpAction::new(url.clone()));
                }
            }
        }

        Ok(Self {
            forester: fb.build()?,
            root,
            profile,
        })
    }

    /// Runs the tree until the root finishes or the `run_until` tick limit is reached,
    /// and afterwards dumps the blackboard to the `bb.dump` file if it is set.
    pub fn run(&mut self) -> Tick {
        let max = match self.profile.run_until.clone().unwrap_or_default() {
            RunUntil::NoLimit => None,
            RunUntil::Limit(ticks) => Some(ticks),
        };

        let result = self.forester.run_until(max);

        if let Some(dump) = self
            .profile
            .bb_config
            .as_ref()
            .and_then(|bb| bb.dump.clone())
        {
            self.forester
                .bb
                .lock()?
                .dump(get_pb(&PathBuf::from(dump), &Some(self.root.clone()))?)?;
        }

        result
    }
}
