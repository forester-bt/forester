use serde::{Deserialize, Serialize};
use crate::runtime::RuntimeError;

pub type BBKey = String;


pub struct BlackBoard {}

impl BlackBoard {
    pub fn lock(&self, key: BBKey) -> Result<(), RuntimeError> {
        unimplemented!()
    }
    pub fn unlock(&self, key: BBKey) -> Result<(), RuntimeError> {
        unimplemented!()
    }

    pub fn get<T: for<'a> Deserialize<'a>>(&self, key: BBKey) -> Result<&T, RuntimeError> {
        unimplemented!()
    }
    pub fn take<T: for<'a> Deserialize<'a>>(&self, key: BBKey) -> Result<T, RuntimeError> {
        unimplemented!()
    }
    pub fn check<T: for<'a> Deserialize<'a>>(&self, key: BBKey) -> Result<bool, RuntimeError> {
        unimplemented!()
    }

    pub fn put<T: Serialize>(&mut self, key: BBKey, value: T) -> Result<(), RuntimeError> {
        unimplemented!()
    }
    pub fn put_if_new<T: Serialize>(&mut self, key: BBKey, value: T) -> Result<(), RuntimeError> {
        unimplemented!()
    }
    pub fn replace<T: Serialize, D: for<'a> Deserialize<'a>>(&mut self, key: BBKey, value: T) -> Result<D, RuntimeError> {
        unimplemented!()
    }
}

