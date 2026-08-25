pub mod config;

use crate::{runner::config::RunProfile, runtime::forester::Forester};
use std::path::PathBuf;

struct Runner {
    pub root: Option<PathBuf>,
    pub main: Option<String>,
    pub profile: RunProfile,
    pub forester: Forester,
}
