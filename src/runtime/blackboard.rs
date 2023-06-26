use crate::runtime::args::RtValue;
use crate::runtime::RuntimeError;
use serde::{Deserialize, Serialize};

pub type BBKey = String;

#[derive(Default, Debug)]
pub struct BlackBoard {}

impl BlackBoard {
    pub fn lock(&self, key: BBKey) -> Result<(), RuntimeError> {
        unimplemented!()
    }
    pub fn unlock(&self, key: BBKey) -> Result<(), RuntimeError> {
        unimplemented!()
    }

    pub fn get(&self, key: BBKey) -> Result<&RtValue, RuntimeError> {
        unimplemented!()
    }
    pub fn take(&self, key: BBKey) -> Result<RtValue, RuntimeError> {
        unimplemented!()
    }
    pub fn check(&self, key: BBKey) -> Result<bool, RuntimeError> {
        unimplemented!()
    }

    pub fn put(&mut self, key: BBKey, value: RtValue) -> Result<(), RuntimeError> {
        unimplemented!()
    }
    pub fn put_if_absent(&mut self, key: BBKey, value: RtValue) -> Result<(), RuntimeError> {
        unimplemented!()
    }
}
