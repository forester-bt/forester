
----

param    : name:type
params   : param (,param)* ,?

item :
   type name? (args)? {items}?
   name (args)? {items}?

args : arg (, args)* ,?
arg  : name (= value)?

value:
    "..."
    1..
    id              // in bb
    {f:.., f2:..}   // as json
    (..)            // tuple

def: type name? (params)? { items } | item

type : // only the second level counts if it exists otherwise the first.
- root
- fallback
  - fallback
  - r_fallback     // reactive_fallback
- sequence
  - sequence   
  - m_sequence  // memory_sequence
  - r_sequence  // reactive_sequence
- parallel
- decorator
  - inverter
  - f_success      // force
  - f_fail         // force
  - repeat
  - retry
  - timeout
- action
  - impl
  - wait
  - success
  - fail
- condition
  - impl
  - equal
  - greater
  - less
  - test
  - test_all
  - test_one

---

message types:
- tree
- string
- object
- num
- tuple
 

