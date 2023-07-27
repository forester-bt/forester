use crate::runtime::context::RNodeState;
use crate::runtime::rtree::rnode::RNodeId;
use crate::runtime::{RtOk, RtResult};
use std::fmt::{Display, Formatter};
use std::fs;
use std::fs::OpenOptions;
use std::io::Write;
use std::ops::Range;
use std::path::PathBuf;
use std::ptr::write;

#[cfg(windows)]
pub const LINE_ENDING: &'static str = "\r\n";
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
        indent: usize,
    },
    InFile {
        level: usize,
        indent: usize,
        file: PathBuf,
    },
}

impl Default for Tracer {
    fn default() -> Self {
        Tracer::create(TracerConfiguration::default()).unwrap()
    }
}

impl Display for Tracer {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Tracer::InMemory { events, .. } => {
                for e in events {
                    f.write_str(&e.to_string())?;
                }
            }
            Tracer::InFile { .. } => (),
            Tracer::Noop => {
                f.write_str(
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
            Tracer::InMemory { level, indent, .. } | Tracer::InFile { level, indent, .. } => {
                *level -= *indent
            }
        };
    }
    /// shift the level of tracing forward to one level. Better to avoid using this method.
    pub fn right(&mut self) {
        match self {
            Tracer::Noop => {}
            Tracer::InMemory { level, indent, .. } | Tracer::InFile { level, indent, .. } => {
                *level += *indent
            }
        }
    }
    /// to add the infromation about the event.
    pub fn trace(&mut self, tick: usize, ev: Event) -> RtOk {
        match self {
            Tracer::Noop => Ok(()),
            Tracer::InMemory { events, level, .. } => {
                let trace = Trace {
                    level: *level,
                    tick,
                    ev,
                };
                events.push(trace);
                Ok(())
            }
            Tracer::InFile { file, level, .. } => {
                let trace = Trace {
                    level: *level,
                    tick,
                    ev,
                };
                let mut file = OpenOptions::new()
                    .append(true)
                    .create(true)
                    .open(file.clone())?;

                file.write(trace.to_string().as_bytes())?;
                Ok(())
            }
        }
    }
    pub fn noop() -> Self {
        Tracer::Noop
    }
    pub fn create(cfg: TracerConfiguration) -> RtResult<Self> {
        debug!("create new tracer from {:?}", cfg);
        match cfg.to_file {
            None => Ok(Tracer::InMemory {
                events: vec![],
                level: 0,
                indent: cfg.indent,
            }),
            Some(file) => {
                if !&file.exists() {
                    if let Some(parent) = file.parent() {
                        fs::create_dir_all(parent);
                        fs::File::create(file.clone())?;
                    }
                }
                Ok(Tracer::InFile {
                    level: 0,
                    indent: cfg.indent,
                    file,
                })
            }
        }
    }
}

#[derive(Debug)]
pub struct TracerConfiguration {
    pub indent: usize,
    pub to_file: Option<PathBuf>,
}

impl TracerConfiguration {
    pub fn in_file(file: PathBuf) -> TracerConfiguration {
        TracerConfiguration {
            indent: 2,
            to_file: Some(file),
        }
    }
    pub fn in_memory() -> TracerConfiguration {
        TracerConfiguration {
            indent: 2,
            to_file: None,
        }
    }
}

impl Default for TracerConfiguration {
    fn default() -> Self {
        TracerConfiguration {
            indent: 2,
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
}

impl Display for Event {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Event::NextTick => {
                f.write_str("next tick")?;
            }
            Event::NewState(id, s) => {
                f.write_str(format!("{} : {}", id, s).as_str())?;
            }
            Event::Custom(s) => {
                f.write_str(s)?;
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
}

impl Display for Trace {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let v = format!(
            "[{tick}]{:indent$}{v}",
            "",
            tick = self.tick,
            indent = self.level,
            v = format!("{}{}", self.ev, LINE_ENDING),
        );
        f.write_str(v.as_str())?;
        Ok(())
    }
}
