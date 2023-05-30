use parsit::error::ParseError;
use parsit::parser::ParseIt;
use parsit::step::Step;
use parsit::token;
use crate::gol::ast::{Definitions, Id};
use crate::gol::lexer::Token;

struct Parser<'a> {
    inner: ParseIt<'a, Token<'a>>,
}

impl<'a> Parser<'a> {
    fn id(&self, pos: usize) -> Step<'a, Id<'a>> {
        token!(self.token(pos) => Token::Id(v) => Id(v) )
    }
}

impl<'a> Parser<'a> {
    pub fn new(src: &'a str) -> Result<Self, ParseError> {
        Ok(Parser {
            inner: ParseIt::new(src)?,
        })
    }
    fn token(&self, pos: usize) -> Result<(&Token<'a>, usize), ParseError<'a>> {
        self.inner.token(pos)
    }

    pub fn parse(src: &'a str) -> Result<Definitions, ParseError<'a>> {
        let parser = Parser::new(src)?;
        parser
            .inner
            .validate_eof(parser.id(0))
            .into()
    }
}