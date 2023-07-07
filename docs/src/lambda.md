# Lambda

The anonymous definitions can be defined and instantly invoked at the same time.
The definitions are unique and every time the new definition is created.

- They don't have a name
- They are unique
- They don't have arguments

**Only the elements of [Flow](./flow.md)  can be used in lambdas**
**The actions always have to be defined explicitly.**

```f-tree
impl job();

root main {
    // lambda invocation
    sequence {
        job()
        job()
        job()
    }
    // another lambda invocation
    fallback {
        sequence {
            job()
            job()   
        }
        // the second level of lambda
        sequence {
            job()
            // also lambda, but the backets are omitted. 
            r_sequence job()
        }
    }

}
```

## Parameter

Lambda can be used as parameters as well.

```f-tree
impl savepoint();
impl job();

sequence wrapper(item:tree){
    savepoint()
    item(..)
    savepoint()
}

root main sequence {
    wrapper(
        sequence {
            job()
            job()
            job()
        }
    )
    wrapper(
        item = 
            fallback {
                job()
                job()
                job()
            }
    )
    
}
```