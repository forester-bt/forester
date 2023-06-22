use crate::runtime::RuntimeErrorCause;
use serde::{Deserialize, Serialize};

pub type BBKey = String;

pub struct BlackBoard {}

impl BlackBoard {
    pub fn lock(&self, key: BBKey) -> Result<(), RuntimeErrorCause> {
        unimplemented!()
    }
    pub fn unlock(&self, key: BBKey) -> Result<(), RuntimeErrorCause> {
        unimplemented!()
    }

    pub fn get<T: for<'a> Deserialize<'a>>(&self, key: BBKey) -> Result<&T, RuntimeErrorCause> {
        unimplemented!()
    }
    pub fn take<T: for<'a> Deserialize<'a>>(&self, key: BBKey) -> Result<T, RuntimeErrorCause> {
        unimplemented!()
    }
    pub fn check<T: for<'a> Deserialize<'a>>(&self, key: BBKey) -> Result<bool, RuntimeErrorCause> {
        unimplemented!()
    }

    pub fn put<T: Serialize>(&mut self, key: BBKey, value: T) -> Result<(), RuntimeErrorCause> {
        unimplemented!()
    }
    pub fn put_if_absent<T: Serialize>(
        &mut self,
        key: BBKey,
        value: T,
    ) -> Result<(), RuntimeErrorCause> {
        unimplemented!()
    }
    pub fn replace<T: Serialize, D: for<'a> Deserialize<'a>>(
        &mut self,
        key: BBKey,
        value: T,
    ) -> Result<D, RuntimeErrorCause> {
        unimplemented!()
    }
}
