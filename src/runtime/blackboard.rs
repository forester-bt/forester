use crate::read_file;
use crate::runtime::args::RtValue;
use crate::runtime::blackboard::BBValue::{Locked, Taken, Unlocked};
use crate::runtime::{RtOk, RtResult, RuntimeError};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

pub type BBKey = String;

/// Representation of the value in the cell.
/// It can be locked or unlocked.
///
/// It can be taken which means that the cell is empty but the value was there.
/// It can be useful in the operation for cas or in the communication between the threads.
///
/// If it is locked it can not be read or taken.
/// If it is unlocked it can be read or taken.
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub enum BBValue {
    Locked(RtValue),
    Unlocked(RtValue),
    Taken,
}

/// The representation of memory in the trees.
/// It represents a simple map in memory of in file.
///
/// **A pair of key and value is called cell.**
///
/// It provides a several basis features:
/// - put to store
/// - get to store
/// - lock/unlock the value in the cell.
/// - take the value in the cell
///
#[derive(Default, Debug, PartialEq, Serialize, Deserialize)]
pub struct BlackBoard {
    storage: HashMap<BBKey, BBValue>,
}

impl BlackBoard {
    /// Locks the value preventing from any actions that can modify the value including writing or taking.
    /// If the value is absent or taken will return an error
    ///
    /// #Notes:
    /// If it is already locked returns only ok.
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

    /// Checks if the value is locked.
    #[allow(clippy::wrong_self_convention)]
    pub fn is_locked(&mut self, key: BBKey) -> RtResult<bool> {
        Ok(match self.storage.get(&key) {
            Some(Locked(_)) => true,
            Some(Unlocked(_)) => false,
            None | Some(Taken) => false,
        })
    }

    /// Unlock the value enabling any actions including reading writing or taking.
    ///
    /// #Notes:
    /// If it absent or taken returns ok.
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

    /// Gets the element by key
    ///
    /// #Notes:
    /// - If taken returns none
    /// - If locked returns the value.
    pub fn get(&self, key: BBKey) -> Result<Option<&RtValue>, RuntimeError> {
        let v = self.storage.get(&key);
        match v {
            Some(Locked(v)) | Some(Unlocked(v)) => Ok(Some(v)),
            Some(Taken) | None => Ok(None),
        }
    }

    /// Trying to extract the element by key
    ///
    /// #Notes:
    /// - If the cell is absent returns error
    /// - If locked returns error
    /// - If taken returns error
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

    /// Check if the key is presented
    ///
    /// #Notes:
    /// Not considered whether it is taken or locked.
    pub fn contains(&self, key: BBKey) -> Result<bool, RuntimeError> {
        Ok(self.storage.contains_key(&key))
    }

    /// Puts an value to a cell
    ///
    /// #Notes:
    /// Error if it is locked
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
    pub fn new(elems: Vec<(BBKey, BBValue)>) -> Self {
        debug!(target:"bb", "create an empty bb");
        Self {
            storage: HashMap::from_iter(elems),
        }
    }
}
impl BlackBoard {
    /// Drops the snapshot to the file in json format.
    pub fn dump(&self, file: PathBuf) -> RtOk {
        let dump = self.text_dump()?;
        debug!(target:"bb", "dump snapshot to the file {:?}",file);
        fs::write(file, dump)?;

        Ok(())
    }

    /// Prints the snapshot in json format.
    pub fn print_dump(&self) -> RtOk {
        let dump = self.text_dump()?;
        info!("{dump}");
        Ok(())
    }
    pub fn text_dump(&self) -> RtResult<String> {
        let dump = serde_json::to_string_pretty(self)?;
        Ok(dump)
    }

    /// Loads the snapshot from the file.
    pub fn load(&self, file: &PathBuf) -> RtResult<BlackBoard> {
        debug!(target:"bb", "load snapshot from the file {:?}",file);
        let src = read_file(file)?;
        let bb: BlackBoard = serde_json::from_str(src.as_str())?;
        Ok(bb)
    }
}
