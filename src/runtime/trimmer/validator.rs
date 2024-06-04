use crate::runtime::trimmer::{RequestBody, TreeSnapshot};

/// Validation function. Takes all validators and go through them.
/// The validators are typically works on the following tasks:
/// - check if some subtasks that are about to replace are running
/// - check if the given subtree (starting from root) is not torn
/// and all nodes have the root tree as a parent or grandparent on some level.
///
/// Typically this is a set of guards preventing or trying to prevent the tree to be broken.
pub fn validate(snapshot: &TreeSnapshot, request: &RequestBody) -> TrimValidationResult {
    for validation in validations() {
        let r = validation.validate(snapshot, request);
        if !r.should_proceed() {
            return r;
        }
    }
    TrimValidationResult::Proceed
}

/// a list of validations
fn validations() -> Vec<Box<impl TrimValidation>> {
    vec![Box::new(RunningVal)]
}

/// How to handle the request after it gets validated
pub enum TrimValidationResult {
    /// to next tick
    Defer(String),
    /// reject and forget
    Reject(String),
    /// try to execute
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

/// Check if some subtasks that are about to replace are running
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
