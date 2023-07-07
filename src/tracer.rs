pub struct Tracer {}

pub enum EventType {}

pub struct Event {
    pub id: usize,
    pub ctx: usize,
    pub tpe: EventType,
    pub txt: String,
}

impl Tracer {
    pub fn create(cfg: TracerConfiguration) {}
}

pub struct TracerConfiguration {}
