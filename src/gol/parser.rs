mod tests;

use std::collections::HashMap;
use std::env::Args;
use std::str::FromStr;
use parsit::error::ParseError;
use parsit::parser::{EmptyToken, ParseIt};
use parsit::step::Step;
use parsit::{seq, token, wrap};
use crate::gol::ast::{Argument, Arguments, Bool, Call, Calls, Id, Message, MesType, Number, Param, Params, StringLit, Tree, Trees, TreeType, validate_lambda};
use crate::gol::lexer::Token;

pub struct Parser<'a> {
    inner: ParseIt<'a, Token<'a>>,
}

impl<'a> Parser<'a> {
    fn assign(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::Assign )
    }
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


    fn array(&'a self, pos: usize) -> Step<'a, Vec<Message<'a>>> {
        let l = |p| self.l_br(p);
        let r = |p| { self.r_br(p) };
        let comma = |p| { self.comma(p) };
        let message = |p| { self.message(p) };
        let elems = |p| seq!(p => message, comma,);
        let no_elems = vec![];

        wrap!(pos => l; elems or no_elems; r)
    }
    fn object(&'a self, pos: usize) -> Step<'a, HashMap<String, Message<'a>>> {
        let l = |p| { self.l_brc(p) };
        let r = |p| { self.r_brc(p) };
        let comma = |p| { self.comma(p) };

        let pair = |p| {
            self.str(p)
                .map(|l| l.0.to_string())
                .then_skip(|p| self.colon(p))
                .then_zip(|p| self.message(p))
        };

        let elems = |p|
            seq!(p => pair, comma,)
                .map(|v| HashMap::from_iter(v));

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

        let elems = |p| {
            seq!(p => param, comma,)
                .map(|params| Params { params })
        };
        let def = Params { params: vec![] };

        wrap!(pos => l; elems or def; r)
    }

    fn arg(&'a self, pos: usize) -> Step<'a, Argument<'a>> {
        let assign = |p| { self.assign(p) };
        let assigned = |p| { self.id(p).then_skip(assign) };
        let assign_id = |p| {
            assigned(p)
                .then_zip(|p| self.id(p))
                .map(|(a, b)| Argument::AssignedId(a, b))
        };
        let assign_mes = |p| {
            assigned(p)
                .then_zip(|p| self.message(p))
                .map(|(a, b)| Argument::AssignedMes(a, b))
        };
        let mes = |p| self.message(p).map(Argument::Mes);
        let id = |p| self.id(p).map(Argument::Id);

        assign_mes(pos)
            .or_from(pos)
            .or(assign_id)
            .or(mes)
            .or(id)
            .into()
    }

    fn args(&'a self, pos: usize) -> Step<'a, Arguments<'a>> {
        let l = |p| { self.l_pr(p) };
        let r = |p| { self.r_pr(p) };
        let comma = |p| { self.comma(p) };

        let arg = |p| self.arg(p);

        let elems = |p| {
            seq!(p => arg, comma,)
                .map(|args| Arguments { args })
        };

        let def = Arguments { args: vec![] };

        wrap!(pos => l; elems or def; r)
    }
    fn tree_type(&self, pos: usize) -> Step<'a, TreeType> {
        self.id(pos)
            .flat_map(
                |p| TreeType::from_str(p.0),
                |pe| Step::Fail(pos),
            )
    }

    fn mes_type(&self, pos: usize) -> Step<'a, MesType> {
        token!(self.token(pos) =>
                Token::StringT => MesType::String ,
                Token::NumT => MesType::Num,
                Token::ArrayT => MesType::Array,
                Token::BoolT => MesType::Bool,
                Token::TreeT => MesType::Tree,
                Token::ObjectT => MesType::Object
        )
    }

    fn message(&'a self, pos: usize) -> Step<'a, Message<'a>> {
        self.str(pos).map(Message::String)
            .or_from(pos)
            .or(|p| self.num(p).map(Message::Num))
            .or(|p| self.bool(p).map(Message::Bool))
            .or(|p| self.array(p).map(Message::Array))
            .or(|p| self.object(p).map(Message::Object))
            .or(|p| self.call(p).map(Message::Call))
            .into()
    }

    fn call(&'a self, pos: usize) -> Step<'a, Call> {
        let inv = |p| {
            self.id(p).then_zip(|p| self.args(p))
                .map(|(id, args)| Call::Invocation(id, args))
        };

        let anon = |p| {
            self.tree_type(p)
                .then_or_default_zip(|p| self.args(p))
                .then_zip(|p| self.calls(p))
                .validate(|((t, args), calls)| validate_lambda(t,args, calls))
                .map(|((t, args), calls)| {
                    if t.is_decorator() {
                        Call::decorator(t, args, calls.elems[0].clone())
                    }else {
                        Call::lambda(t, calls)
                    }
                })
        };

        anon(pos).or_from(pos).or(inv).into()
    }
    fn calls(&'a self, pos: usize) -> Step<'a, Calls> {
        let calls = |p| {
            let l = |p| { self.l_brc(p) };
            let r = |p| { self.r_brc(p) };
            let calls = |p| self.inner.zero_or_more(p, |p| self.call(p));
            wrap!(p => l;calls;r).map(Calls::new)
        };

        calls(pos)
            .or_from(pos)
            .or(|p| self.call(p)
                .map(|c| Calls { elems: vec![c] }))
            .into()
    }

    fn tree(&'a self, pos: usize) -> Step<'a, Tree> {
        self
            .tree_type(pos)
            .then_zip(|p| self.id(p))
            .then_or_default_zip(|p| self.params(p))
            .then_or_default_zip(|p| self.calls(p))
            .map(|(((tpe, name), params), calls)| Tree {
                tpe,
                name,
                params,
                calls,
            })
    }
    fn trees(&'a self, pos: usize) -> Step<'a, Trees> {
        let make_tree = |elems: Vec<Tree<'a>>| {
            let it = elems.into_iter().map(|e| (e.name.0.to_string(), e));
            Trees(HashMap::from_iter(it))
        };
        self.inner.zero_or_more(pos, |p| self.tree(p)).map(make_tree)
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

    pub fn parse(&'a self) -> Result<Trees, ParseError<'a>> {
        self.inner.validate_eof(self.trees(0))
            .into()
    }
}

