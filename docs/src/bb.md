# Blackboard

Blackboard represents a memory layer that enables to store and get the data, lock and take it. 
By default, it works in memory.

## Format

Blackboard preserves the pairs of `String` and `BBValue`.

```rust
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub enum BBValue {
    Locked(RtValue),
    Unlocked(RtValue),
    Taken,
}
```

where `Locked` means the value is locked for everything, 
`Unlocked` a normal value enables to read, write and other actions,
`Taken` means the key exists but the value is taken from the Blackboard.

### Load and dump

The Blackboard enables to `dump` the snapshot to the disk 
or print it and `load` the initial configuration from a file, see `ForesterBuilder` for details.

- dump: Drops the snapshot to the file in json format.
- print_dump: Prints the snapshot to the stdout in json format.
- text_dump: Returns the snapshot in json format.
- load: Loads the snapshot from the file in json format.


## Utils
A set of extra helper methods for the Blackboard is available in the `utils` 
module for instance ` blackboard::utils::push_to_arr` method.