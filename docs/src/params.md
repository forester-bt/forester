# Parameters

## Terminology

- Parameters are elements of tree definitions.
- Arguments are elements of tree invocations.

```f-tree
// parameters 'a' and 'b'
sequence tree(a:string,b:num){
    // arguments 'c', 'd'
    job(c = 1, d = "d")
}
```

## Arguments

Therefore, the arguments represent the attachments of the real value to the parameters.
The Argument can be one of two types:

- Named argument
- Unnamed argument

```f-tree
impl action(a:string, b:num)

root main sequnce {
    // Named Arguments
    action(a="a",b:1)
    
    // Unnamed Arguments
    action("a",1)
}
```

**There is impossible to mix named and unnamed arguments**
The following code will have an error during the compilation process.

```f-tree
impl action(a:string, b:num)
root main sequnce {
    action("a",b=1 )
}
```

## Types

### Number

The numbers are defined with a keyword `num`
There are 4 possible types of numbers presented:

- Integers(64)
- Floats(64)
- Hex
- Binary

**In case of exceeding the maximum value, the error will be raised on the compile time.**

```f-tree
impl action(param:num)

root main sequence {
    // Integers
    action(1)
    action(10e2)
    action(-1)
    action(0)
    
    // Floats
    action(0.0)
    action(100.0e1)
    action(-100.0)
    
    // Hex
    action(0x123)
    
    // Binary
    action(0b010101)
}
```

### String

The strings are defined with `string`

```f-tree
impl action(param:string)
root main action(param = "X")
```

### Boolean

The booleans are defined with a keyword `bool` and has the following parameters:

- `true` for the positive statement
- `false` for the negative statement

```f-tree
impl action(param:bool);
root main action(true)
```

### Arrays

The arrays are defined with keyword `array`
Arrays can have several aforementioned elements encompassed in one entity.

The arrays have the following syntax:

- `[` defines the start of array
- `]` defines the end of array
- `,` defines the separator between elements
- the rest is defined by the particular elements

**It is expected, the arrays are homogeneous and have all elements only one type**

**The arrays can have a trailing comma as well, `[1,]`**

```f-tree
impl action(elems:array);
root main sequence {
    action([1,2,3,4])
    action([1.1,0.1])
    action(["a","b"])
}
```

### Objects

The objects are defined with keyword `object`
Objects can have several aforementioned elements encompassed
in one entity with the unique key attached to the every entity

The objects have the following syntax:

- `{` defines the start of object
- `}` defines the end of object
- `,` defines the separator between elements
- "key" defines the name of the element key
- the rest is defined by the particular elements

**The objects can have a trailing comma as well, `{"a":1,}`**

```f-tree
impl action(elems:object);
root main sequence {
    action({"key":1, "key2":"key"})
    action(
        {
            "array": [1,2,3,4,],
            "string":"string",
            "num":1,
            "pointer": pointer
        }
    )
}
```

### Tree

The other tree definitions are defined with a keyword `tree`
**The parameters of this type can be added and defined only in the [flow](./flow.md) definitions.

```f-tree
impl log(id:string,info:string);
cond check();
cond task();

fallback checked_task(check:tree, task:tree){
    check(..)
    task(..)
}

sequence logged_task(id:string, task:tree){
    log(id,"start task")
    task(..)
    log(id,"end task")
}

root main sequence {
    // invoke the task parameter, passing the invokations with parameters 
    logged_task(
        "1",
        // invoke the task, passing the invokations with parameters
        checked_task(check = check(), task())
    )
}
```

### Pointers

Pointers are identifiers of the objects in the [BlackBoard](./bb.md)
Therefore, they can be used to obtain the value of the cell from bb, in argument invoking.

In the example below, the system expects to find a string value in the cell with a name `bb_key`.

```f-tree
impl action(value:string);

root main sequence {
    // this is a pointer to a cell in bb with an id 'bb_key'
    action(bb_key) 
}
```