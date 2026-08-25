use serde::{Deserialize, Serialize};

#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
pub struct RunProfile {
    run_until: Ticks,

    bb_dump: Option<String>,
    bb_load: Option<String>,

    tracer: Option<String>
}

#[derive(Debug, Default, PartialEq, Clone, Serialize, Deserialize)]
enum Ticks {
    #[default]
    Unlim,
    Lim(usize),
}
