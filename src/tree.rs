use crate::runtime::RuntimeError;
use parsit::error::ParseError;

pub mod parser;
pub mod project;

pub fn cerr(v: String) -> TreeError {
    TreeError::CompileError(v)
}

#[derive(Debug, PartialEq)]
pub enum TreeError {
    ParseError(String),
    CompileError(String),
    VisualizationError(String),
    IOError(String),
}

impl From<ParseError<'_>> for TreeError {
    fn from(value: ParseError) -> Self {
        TreeError::ParseError(value.to_string())
    }
}
impl From<RuntimeError> for TreeError {
    fn from(value: RuntimeError) -> Self {
        TreeError::IOError(format!("{:?}", value))
    }
}
