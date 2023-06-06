use parsit::error::ParseError;
use parsit::step::Step;

mod ast;
mod parser;
mod lexer;
mod project;
mod visualizer;

#[derive(Debug)]
pub enum GolError{
    ParserError(String),
    CompileError(String),
    VisualizationError(String),
    IOError(String)
}

impl From<ParseError<'_>> for GolError {
    fn from(value: ParseError) -> Self {
        GolError::ParserError(value.to_string())
    }
}


