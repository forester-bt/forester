use crate::runtime::context::RNodeState;
use crate::runtime::rtree::rnode::RNodeId;
use crate::runtime::RtResult;
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
    pub fn left(&mut self) {
        match self {
            Tracer::Noop => {}
            Tracer::InMemory { level, indent, .. } | Tracer::InFile { level, indent, .. } => {
                *level -= *indent
            }
        };
    }
    pub fn right(&mut self) {
        match self {
            Tracer::Noop => {}
            Tracer::InMemory { level, indent, .. } | Tracer::InFile { level, indent, .. } => {
                *level += *indent
            }
        }
    }
    pub fn trace(&mut self, tick: usize, ev: Event) {
        match self {
            Tracer::Noop => {}
            Tracer::InMemory { events, level, .. } => {
                let trace = Trace {
                    level: *level,
                    tick,
                    ev,
                };
                events.push(trace);
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
                    .open(file.clone())
                    .map_err(|e| error!("error in writing to file {} {:?}", e, file))
                    .unwrap();

                file.write(trace.to_string().as_bytes())
                    .map_err(|e| error!("error in writing to file {} {:?}", e, file));
            }
        }
    }
    pub fn noop() -> Self {
        Tracer::Noop
    }
    pub fn create(cfg: TracerConfiguration) -> RtResult<Self> {
        match cfg.to_file {
            None => Ok(Tracer::InMemory {
                events: vec![],
                level: 0,
                indent: cfg.indent,
            }),
            Some(file) => {
                // println!(" >> {:?}", file);
                // if !&file.exists() {
                //     if let Some(parent) = file.parent() {
                //         fs::create_dir_all(parent);
                //         fs::File::create(file.clone())?;
                //     }
                // }

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

#[derive(Debug, Clone)]
pub enum Event {
    NextTick,
    NewState(RNodeId, RNodeState),
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
