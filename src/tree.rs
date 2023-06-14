use parsit::error::ParseError;
use parsit::step::Step;
use crate::runtime::RuntimeError;

pub mod parser;
pub mod project;
pub mod visualizer;

pub fn cerr(v: String) -> TreeError {
    TreeError::CompileError(v)
}

#[derive(Debug)]
pub enum TreeError {
    ParseError(String),
    CompileError(String),
    VisualizationError(String),
    IOError(String)
}

impl From<ParseError<'_>> for TreeError {
    fn from(value: ParseError) -> Self {
        TreeError::ParseError(value.to_string())
    }
}


