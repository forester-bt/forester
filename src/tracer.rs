use crate::runtime::context::RNodeState;
use crate::runtime::rtree::rnode::RNodeId;
use std::fmt::{Display, Formatter, Write};
use std::ops::Range;
use std::ptr::write;

#[cfg(windows)]
const LINE_ENDING: &'static str = "\r\n";
#[cfg(not(windows))]
const LINE_ENDING: &'static str = "\n";

#[derive(Debug, Default)]
pub struct Tracer {
    pub events: Vec<Trace>,
    level: usize,
}

impl Tracer {
    pub fn create(cfg: TracerConfiguration) -> Self {
        Tracer::default()
    }

    pub fn trace(&mut self, tick: usize, ev: Event) {
        self.events.push(Trace {
            level: self.level,
            tick,
            ev,
        });
    }

    pub fn left(&mut self) {
        self.level -= 2;
    }
    pub fn right(&mut self) {
        self.level += 2;
    }
}

#[derive(Debug, Default)]
pub struct TracerConfiguration {}

#[derive(Debug)]
pub enum Event {
    NextTick,
    NewState(RNodeId, RNodeState),
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
        }

        Ok(())
    }
}

#[derive(Debug)]
pub struct Trace {
    pub level: usize,
    pub tick: usize,
    pub ev: Event,
}

impl Display for Tracer {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        for e in &self.events {
            let v = format!(
                "[{tick}]{:indent$}{v}",
                "",
                tick = e.tick,
                indent = e.level,
                v = format!("{}{}", e.ev, LINE_ENDING),
            );
            f.write_str(v.as_str())?;
        }

        Ok(())
    }
}
