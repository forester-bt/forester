# Built-in actions
By default, the framework provides a set of the actions 
and conditions that are already implemented.

To use them, the user should import the special file
```f-tree
import "std::actions"
```
or just a specific definition of the file
```f-tree
import "std::actions" {
    store => store_data,
    fail
}
```

## File
```f-tree
 //
// Built-in actions. 
// The actions are accessible using the import 'import "std::actions"' 
// Better off, the file be avoided modifying
//

// Fails execution, returning Result::Failure        
impl fail(reason:string);
impl fail_empty();

// Success execution, returning Result::Success  
impl success();

// Running execution, returning Result::Running  
impl running();

// Sleeps on duration(milliseconds) then returns Result::Success
// impl sleep(duration:num);

// Stores the string value in the given key. Returns Result::Success. 
// If the cell is locked, returns Result::Failure   
impl store_str(key:string, value:string);

// Compares given string value with what is in the cell:
// - Returns Result::Success if they are equal
// - Returns Fail(reason)if they are not equal
// - Returns Fail(reason) if there is no cell in bbe with the given key.
impl eq_str(key:string, expected:string);
impl eq_num(key:string, expected:num);

// Store the current tick
impl store_tick(name:string);

// Lock key in bb
impl lock(key:string);

// Unlock key in bb
impl unlock(key:string);

// Performs http get request
impl http_get(url:string, bb_key:string);


```



# Http server (sync | async)
# Curl
# lock | unlock key
 