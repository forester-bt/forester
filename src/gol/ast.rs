use std::collections::HashMap;
use std::fmt::{Display, Formatter};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct EmptyToken {}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Number {
    Int(i64),
    Float(f64),
    Hex(i64),
    Binary(isize),
}

#[derive(Debug, Copy, Clone, PartialEq,Hash)]
pub struct Id<'a>(pub &'a str);

pub struct StringLit<'a>(pub &'a str);

pub enum Bool {
    True,
    False,
}


pub enum Message<'a> {
    Num(Number),
    String(StringLit<'a>),
    Bool(Bool),
    Array(Vec<Message<'a>>),
    Object(HashMap<String, Message<'a>>),
    Call(Call),
}

pub struct Call {}

pub struct Param<'a>{
    pub name:Id<'a>,
    pub tpe: MesType,
}
pub struct Params<'a>{
    pub params:Vec<Param<'a>>
}

pub enum TreeType {
    Root,
    Par,
    Seq,
    MSeq,
    RSeq,
    Fall,
    RFall,
    // decorators
    Inv,
    FSuccess,
    FFail,
    Repeat,
    Retry,
    Timeout,
    // actions
    Impl,
    Wait,
    Success,
    Fail,
    // conditions
    Cond,
    Equal,
    Gt,
    Ls,
    Test,
    TestAll,
    TestOne,
}

pub enum MesType {
    Num,
    Array,
    Object,
    String,
    Bool,
    Tree,
}

pub struct Tree {
    pub tpe: TreeType,
}


pub struct Trees(HashMap<String, Tree>);