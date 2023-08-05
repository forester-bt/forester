use crate::runtime::trimmer::{RequestBody, TreeSnapshot, TrimRequest};

pub enum TrimValidationResult {
    Defer(String),
    Reject(String),
    Proceed,
}

impl TrimValidationResult {
    pub fn should_proceed(&self) -> bool {
        match self {
            TrimValidationResult::Proceed => true,
            _ => false,
        }
    }
}

pub trait TrimValidation {
    fn validate(&self, snapshot: &TreeSnapshot, request: &RequestBody) -> TrimValidationResult;
}

pub fn validate(snapshot: &TreeSnapshot, request: &RequestBody) -> TrimValidationResult {
    for validation in validations() {
        let r = validation.validate(snapshot, request);
        if !r.should_proceed() {
            return r;
        }
    }
    TrimValidationResult::Proceed
}

fn validations() -> Vec<Box<impl TrimValidation>> {
    vec![Box::new(RunningVal)]
}

struct RunningVal;
impl TrimValidation for RunningVal {
    fn validate(&self, snapshot: &TreeSnapshot, request: &RequestBody) -> TrimValidationResult {
        let state = snapshot.tree_state;
        for k in request.tree_b.nodes.keys() {
            if state.get(k).map(|v| v.is_running()).unwrap_or(false) {
                return TrimValidationResult::Defer(format!("the node {k} is running."));
            }
        }

        TrimValidationResult::Proceed
    }
}
