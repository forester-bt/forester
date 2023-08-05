use crate::runtime::context::RNodeState;
use crate::runtime::rtree::rnode::RNodeId;
use crate::runtime::{RtOk, RtResult};
use chrono::{DateTime, Utc};
use std::fmt::{Display, Formatter};
use std::fs;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::PathBuf;

#[cfg(windows)]
pub const LINE_ENDING: &str = "\r\n";
#[cfg(not(windows))]
pub const LINE_ENDING: &'static str = "\n";

/// The structure that keep trace of execution and store it either in memory ot in a file.
/// The tracer uses printable representation with the space indentation.
///
/// #Example of output
/// ```plain
/// [1]  1 : Running(cursor=0,len=1)
/// [1]    2 : Running(cursor=0,len=3)
/// [1]      3 : Success(key=info,value=initial)
/// [1]    2 : Running(cursor=1,len=3)
/// [1]      4 : Running(cursor=0,len=3)
/// [1]        6 : Running(len=1)
/// [1]          9 : Failure(config=[],reason=)
/// [1]        6 : Running(arg=2,cursor=0,len=1)
/// [1]      4 : Running(cursor=0,len=3,prev_cursor=0)
/// [1]    2 : Running(cursor=1,len=3,prev_cursor=1)
/// [2]  next tick
/// [2]    2 : Running(cursor=0,len=3,prev_cursor=1)
/// [2]      4 : Running(cursor=0,len=3,prev_cursor=0)
/// [2]        6 : Running(arg=2,cursor=0,len=1)
/// [2]          9 : Failure(config=[],reason=)
/// [2]        6 : Running(arg=3,cursor=0,len=1)
/// [2]      4 : Running(cursor=0,len=3,prev_cursor=0)
/// [2]    2 : Running(cursor=0,len=3,prev_cursor=1)
/// [2]  1 : Running(cursor=0,len=1)
/// [3]  next tick
/// [3]    2 : Running(cursor=0,len=3,prev_cursor=1)
/// [3]      4 : Running(cursor=0,len=3,prev_cursor=0)
/// [3]        6 : Running(arg=3,cursor=0,len=1)
/// [3]          9 : Failure(config=[],reason=)
/// [3]        6 : Running(arg=4,cursor=0,len=1)
/// [3]      4 : Running(cursor=0,len=3,prev_cursor=0)
/// [3]    2 : Running(cursor=0,len=3,prev_cursor=1)
/// [3]  1 : Running(cursor=0,len=1)
/// [4]  next tick
/// ```
#[derive(Debug)]
pub enum Tracer {
    Noop,
    InMemory {
        events: Vec<Trace>,
        level: usize,
        cfg: TracerConfig,
    },
    InFile {
        level: usize,
        cfg: TracerConfig,
        file: PathBuf,
    },
}

impl Default for Tracer {
    fn default() -> Self {
        Tracer::create(TracerConfig::default()).unwrap()
    }
}

impl Display for Tracer {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Tracer::InMemory { events, cfg, .. } => {
                for e in events {
                    f.write_str(&e.to_string(cfg.time_format.clone()))?;
                }
            }
            Tracer::InFile { .. } => (),
            Tracer::Noop => {
                let _ = f.write_str(
                    " the noop implementation. does not have any records. \
                Please, check the configuration, and ensure the tracer option is turned on.",
                );
            }
        }

        Ok(())
    }
}

impl Tracer {
    /// shift the level of tracing back to one level. Better to avoid using this method.
    pub fn left(&mut self) {
        match self {
            Tracer::Noop => {}
            Tracer::InMemory { level, cfg, .. } | Tracer::InFile { level, cfg, .. } => {
                *level -= cfg.indent
            }
        };
    }
    /// shift the level of tracing forward to one level. Better to avoid using this method.
    pub fn right(&mut self) {
        match self {
            Tracer::Noop => {}
            Tracer::InMemory { level, cfg, .. } | Tracer::InFile { level, cfg, .. } => {
                *level += cfg.indent
            }
        }
    }
    /// to add the information about the event.
    pub fn trace(&mut self, tick: usize, ev: Event) -> RtOk {
        match self {
            Tracer::Noop => Ok(()),
            Tracer::InMemory { events, level, cfg } => {
                let trace = if cfg.time_format.is_some() {
                    Trace::new_with_dt(*level, tick, ev)
                } else {
                    Trace::new(*level, tick, ev)
                };
                events.push(trace);
                Ok(())
            }
            Tracer::InFile { file, level, cfg } => {
                let trace = if cfg.time_format.is_some() {
                    Trace::new_with_dt(*level, tick, ev)
                } else {
                    Trace::new(*level, tick, ev)
                };
                let mut file = OpenOptions::new()
                    .append(true)
                    .create(true)
                    .open(file.clone())?;

                file.write_all(trace.to_string(cfg.time_format.clone()).as_bytes())?;
                Ok(())
            }
        }
    }
    pub fn noop() -> Self {
        Tracer::Noop
    }
    pub fn create(cfg: TracerConfig) -> RtResult<Self> {
        debug!("create new tracer from {:?}", cfg);
        match &cfg.to_file {
            None => Ok(Tracer::InMemory {
                events: vec![],
                level: 0,
                cfg,
            }),
            Some(file) => {
                if !&file.exists() {
                    if let Some(parent) = file.parent() {
                        let _ = fs::create_dir_all(parent);
                        fs::File::create(file.clone())?;
                    }
                }
                Ok(Tracer::InFile {
                    level: 0,
                    file: file.clone(),
                    cfg,
                })
            }
        }
    }
}

#[derive(Debug)]
pub struct TracerConfig {
    pub indent: usize,
    pub time_format: Option<String>,
    pub to_file: Option<PathBuf>,
}

impl TracerConfig {
    pub fn default_dt_fmt() -> String {
        "%d %H:%M:%S%.3f".to_string()
    }

    pub fn in_file(file: PathBuf, dt_fmt: Option<String>) -> TracerConfig {
        TracerConfig {
            indent: 2,
            time_format: dt_fmt,
            to_file: Some(file),
        }
    }
    pub fn in_memory(dt_fmt: Option<String>) -> TracerConfig {
        TracerConfig {
            indent: 2,
            time_format: dt_fmt,
            to_file: None,
        }
    }
    pub fn time_format(&mut self, f: &str) {
        self.time_format = Some(f.to_string());
    }
}

impl Default for TracerConfig {
    fn default() -> Self {
        TracerConfig {
            indent: 2,
            time_format: None,
            to_file: None,
        }
    }
}

/// The type of events that Tracer can store.
#[derive(Debug, Clone)]
pub enum Event {
    /// The next tick occurs
    NextTick,
    /// The current node updates its state
    NewState(RNodeId, RNodeState),
    /// The custom user information.
    Custom(String),

    Trim(RNodeId, String),
}

impl Display for Event {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Event::NextTick => {
                f.write_str(format!("next tick").as_str())?;
            }
            Event::NewState(id, s) => {
                f.write_str(format!("{} : {}", id, s).as_str())?;
            }
            Event::Custom(s) => {
                f.write_str(format!("custom: {s}").as_str())?;
            }
            Event::Trim(id, txt) => {
                f.write_str(format!("trim {id} : {txt}").as_str())?;
            }
        }

        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct Trace {
    pub level: usize,
    pub tick: usize,
    pub ev: Event,
    dts: Option<DateTime<Utc>>,
}

impl Trace {
    pub fn new(level: usize, tick: usize, ev: Event) -> Self {
        Self {
            level,
            tick,
            ev,
            dts: None,
        }
    }
    pub fn new_with_dt(level: usize, tick: usize, ev: Event) -> Self {
        Self {
            level,
            tick,
            ev,
            dts: Some(Utc::now()),
        }
    }
    pub fn to_string(&self, dtf: Option<String>) -> String {
        let dt = dtf
            .and_then(|f| self.dts.map(|dts| dts.format(f.as_str()).to_string()))
            .unwrap_or_default();

        format!(
            "{dt} [{tick}]{:indent$}{v}",
            "",
            tick = self.tick,
            indent = self.level,
            v = format!("{}{}", self.ev, LINE_ENDING),
        )
    }
}
