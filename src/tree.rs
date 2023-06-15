use crate::runtime::RuntimeErrorCause;
use parsit::error::ParseError;
use parsit::step::Step;

pub mod parser;
pub mod project;

pub fn cerr(v: String) -> TreeError {
    TreeError::CompileError(v)
}

#[derive(Debug)]
pub enum TreeError {
    ParseError(String),
    CompileError(String),
    RuntimeError(RuntimeErrorCause),
    VisualizationError(String),
    IOError(String),
}

impl From<ParseError<'_>> for TreeError {
    fn from(value: ParseError) -> Self {
        TreeError::ParseError(value.to_string())
    }
}
impl From<RuntimeErrorCause> for TreeError {
    fn from(value: RuntimeErrorCause) -> Self {
        TreeError::RuntimeError(value)
    }
}
