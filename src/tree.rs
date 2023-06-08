use parsit::error::ParseError;
use parsit::step::Step;

mod parser;
mod project;
mod visualizer;

pub fn cerr(v: String) -> TreeError {
    TreeError::CompileError(v)
}

#[derive(Debug)]
pub enum TreeError {
    ParserError(String),
    CompileError(String),
    VisualizationError(String),
    IOError(String)
}

impl From<ParseError<'_>> for TreeError {
    fn from(value: ParseError) -> Self {
        TreeError::ParserError(value.to_string())
    }
}


