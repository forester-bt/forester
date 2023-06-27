use crate::runtime::args::RtValue;
use crate::runtime::blackboard::BBValue::{Locked, Taken, Unlocked};
use crate::runtime::{RtOk, RuntimeError};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::hash::Hash;

pub type BBKey = String;
#[derive(Debug)]
pub enum BBValue {
    Locked(RtValue),
    Unlocked(RtValue),
    Taken,
}

#[derive(Default, Debug)]
pub struct BlackBoard {
    storage: HashMap<BBKey, BBValue>,
}

impl BlackBoard {
    pub fn lock(&mut self, key: BBKey) -> RtOk {
        let v = self.storage.get(&key);
        match v {
            Some(Unlocked(v)) => {
                self.storage.insert(key, Locked(v.clone()));
                Ok(())
            }
            Some(Locked(_)) => Ok(()),
            None | Some(Taken) => Err(RuntimeError::bb(format!(
                "the key {key} is taken or absent"
            ))),
        }
    }
    pub fn unlock(&mut self, key: BBKey) -> RtOk {
        let v = self.storage.get(&key);
        match v {
            Some(Locked(v)) => {
                self.storage.insert(key, Unlocked(v.clone()));
                Ok(())
            }
            _ => Ok(()),
        }
    }

    pub fn get(&self, key: BBKey) -> Result<Option<&RtValue>, RuntimeError> {
        let v = self.storage.get(&key);
        match v {
            Some(Locked(_)) => Err(RuntimeError::bb(format!("the key {key} is locked"))),
            Some(Taken) | None => Ok(None),
            Some(Unlocked(v)) => Ok(Some(v)),
        }
    }
    pub fn take(&mut self, key: BBKey) -> Result<RtValue, RuntimeError> {
        let v = self.storage.get(&key);
        match v {
            None => Err(RuntimeError::bb(format!("the key {key} does not exist"))),
            Some(Locked(_)) => Err(RuntimeError::bb(format!("the key {key} is locked"))),
            Some(Taken) => Err(RuntimeError::bb(format!("the key {key} is already taken"))),
            Some(Unlocked(v)) => {
                let v = v.clone();
                self.storage.insert(key, Taken);
                Ok(v)
            }
        }
    }
    pub fn contains(&self, key: BBKey) -> Result<bool, RuntimeError> {
        Ok(self.storage.contains_key(&key))
    }

    pub fn put(&mut self, key: BBKey, value: RtValue) -> RtOk {
        let curr = self.storage.get(&key);
        match curr {
            Some(Locked(_)) => Err(RuntimeError::bb(format!("the key {key} is locked"))),
            _ => {
                self.storage.insert(key, Unlocked(value));
                Ok(())
            }
        }
    }
}
