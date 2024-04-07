# Decorators

The decorators are the specific type of node,
that transforms the result of its child.
Every decorator has a specific keyword and set of parameters.

** Every decorator should have solely one child **

## Inverter

The keyword is `inverter`.
The decorator inverts the result of the child.\
*Only the final results are inverted, for `running` the result will be
`running` as well*

```f-tree
main root sequence {
    inverter check_condition() // inverts the result
}
```

## ForceSuccess

The keyword is `force_success`.
Always returns `success` regardless of the child response

## ForceFail

The keyword is `force_fail`.
Always returns `failure` regardless of the child response

## Repeat

The keyword is `repeat`
It repeats the child so the number of times according to the passing parameter

- count: the number of repetitions. 0 by default
- if the count is 0 the repeat becomes an infinite loop

```f-tree
// the job will be performed INF times
root main_idle repeat {
    job()
}

// the job will be performed 5 times
root main repeat(5) {
    job()    
}
```

## Retry

The keyword is `retry`
If the child returns `failure`, the decorator tries to run it again.
The number of attempts is restricted by the given argument

- attempt: the number of repetitions. 0 by default
- if the attempt is 0 the Retry becomes an infinite loop

```f-tree
// 0 by default. will be never failed.
root main retry {
    sequence { 
        job1() 
        job2() 
    }
}

// the decorator will try to repeat the sequence upt to 10 times if it returns failure
root main_with_retry retry(10) {
    sequence { 
        job1() 
        job2() 
    }
}
```

## Timeout

The keyword is `timeout`
The decorator tries to measure how long the child is running and shut id down if it exceeds the limit.
**For now, it works only for asynchronous actions since the decorator measures time when the child returns `running`**

- limit: the threshold in milliseconds. 1000 by default.

```f-tree
// if the squence works asynchonously (returns running)
// the timeout will count up the time of the first start 
// and then recheck it every time when the child returns running 
root main_with_retry timeout {
    sequence { 
        job1() 
        job2() 
    }
}
```

## Delay

The keyword is `delay`
The decorator delays the initial run of the child for the given as a parameter time.

- wait: the delay time in milliseconds. 0 by default.

```f-tree
// the delay is zero
root main delay job()

// the delay is 1 second
root main_d delay(1000) job()

```