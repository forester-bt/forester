# Invocations

The language provides the possibility to invoke the definitions 
or lambdas in the body of the other definitions

```f-tree

import "std::actions"

impl handle_distance(item:object);

sequence main_seq {
    inverter fail("for test")
    success()
}

sequence check_distance(item:object){
        store_str("log","start")
        handle_distance(item)
        store_str("log","end")
}

// definition
root main sequence {
    // invocation
    main_seq()
    
    // another invocation
    check_distance({"x":1,"y":2})
}
```

## Other types of invocation
The other types of invocation are described in the following sections but briefly are:
- higher order tree invocation: a possibility to pass a tree definition as parameter
- lambda invocation: an anonymous definition that creates and gets invoked at the same time.