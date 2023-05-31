use std::collections::HashMap;
use parsit::error::ParseError;
use parsit::parser::{EmptyToken, ParseIt};
use parsit::step::Step;
use parsit::{seq, token, wrap};
use crate::gol::ast::{Bool, Call, Definitions, Id, Message, MesType, Number, Param, Params, StringLit};
use crate::gol::lexer::Token;

struct Parser<'a> {
    inner: ParseIt<'a, Token<'a>>,
}

impl<'a> Parser<'a> {
    fn l_pr(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::LParen )
    }
    fn r_pr(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::RParen )
    }
    fn l_brc(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::LBrace )
    }
    fn r_brc(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::RBrace )
    }
    fn l_br(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::LBrack )
    }
    fn r_br(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::RBrack )
    }

    fn comma(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::Comma )
    }
    fn colon(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::Colon )
    }

    fn id(&self, pos: usize) -> Step<'a, Id<'a>> {
        token!(self.token(pos) => Token::Id(v) => Id(v) )
    }
    fn str(&self, pos: usize) -> Step<'a, StringLit<'a>> {
        token!(self.token(pos) => Token::StringLit(v) => StringLit(v) )
    }
    fn num(&self, pos: usize) -> Step<'a, Number> {
        token!(self.token(pos) => Token::Digit(n) => n.clone() )
    }
    fn bool(&self, pos: usize) -> Step<'a, Bool> {
        token!(self.token(pos) =>
                Token::True => Bool::True ,
                Token::False => Bool::False
        )
    }


    fn array(&self, pos: usize) -> Step<'a, Vec<Message<'a>>> {
        let l = |p| self.l_br(p);
        let r = |p| { self.r_br(p) };
        let comma = |p| { self.comma(p) };
        let message = |p| { self.message(p) };
        let elems = |p| seq!(p => message, comma,);
        let no_elems = vec![];

        wrap!(pos => l; elems or no_elems; r)
    }
    fn object(&self, pos: usize) -> Step<'a, HashMap<String, Message<'a>>> {
        let l = |p| { self.l_brc(p) };
        let r = |p| { self.r_brc(p) };
        let comma = |p| { self.comma(p) };

        let pair = |p| {
            self.str(p)
                .map(|l| l.0.to_string())
                .then_skip(|p| self.colon(p))
                .then_zip(|p| self.message(p))
        };

        let elems = |p| seq!(pos => pair, comma,).map(|v| HashMap::from_iter(v));
        let def = HashMap::new();

        wrap!(pos => l; elems or def; r)
    }
    fn params(&self, pos: usize) -> Step<'a, Params<'a>> {
        let l = |p| { self.l_pr(p) };
        let r = |p| { self.r_pr(p) };
        let comma = |p| { self.comma(p) };

        let param = |p| {
            self.id(p)
                .then_skip(|p| self.colon(p))
                .then_zip(|p| self.mes_type(p))
                .map(|(name, tpe)| Param { name, tpe })
        };

        let elems = |p| seq!(pos => param, comma,).map(|params| Params { params });
        let def = Params{ params: vec![] };

        wrap!(pos => l; elems or def; r)
    }
    fn call(&self, pos: usize) -> Step<'a, Call> {
        Step::Fail(1)
    }

    fn tree_type(&self, pos: usize) -> Step<'a, Call> {
        Step::Fail(1)
    }
    fn mes_type(&self, pos: usize) -> Step<'a, MesType> {
        token!(self.token(pos) =>
                Token::StringT => MesType::String ,
                Token::NumT => MesType::Num,
                Token::ArrayT => MesType::Array,
                Token::BoolT => MesType::Bool,
                Token::TreeT => MesType::Tree
        )
    }

    fn message(&self, pos: usize) -> Step<'a, Message<'a>> {
        self.str(pos).map(Message::String)
            .or_from(pos)
            .or(|p| self.num(p).map(Message::Num))
            .or(|p| self.bool(p).map(Message::Bool))
            .or(|p| self.array(p).map(Message::Array))
            .or(|p| self.object(p).map(Message::Object))
            .or(|p| self.call(p).map(Message::Call))
            .into()
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