# Antlr grammar

The grammar bears an introducing character (means it is not used straight in the code for now)

## Parser

```antlrv4
parser grammar TreeParser;

file
    : (definition | importSt)* EOF
    ;

import_name
    : id (EQ_A id)?
    ;

importCalls
    : LBC (import_name (COMMA import_name)* COMMA?)? RBC
    ;

importSt
    : IMPORT string importCalls?
    ;

definition
    : tree_type id params? (calls? | SEMI)
    ;

call
    : invocation
    | lambda
    ;

invocation
    : id (args | LPR DOT_DOT RPR)
    ;


lambda
    : tree_type args? calls
    ;

calls
    : LBC call* RBC
    | call
    ;


arg
    : id (EQ (message | id | call))?
    | message
    | call
    ;

args
    : LPR (arg (COMMA arg)* COMMA?)? RPR
    ;

params
    : LPR (param (COMMA param)*)? COMMA? RPR
    ;

param
    : id COLON mes_type
    ;

message
    : string
    | num
    | bool
    | array
    | object
    ;

mes_type
    : NUM_T
    | ARRAY_T
    | OBJECT_T
    | STRING_T
    | BOOL_T
    | TREE_T
    ;

tree_type
    : ROOT
    | PARALLEL
    | SEQUENCE
    | MSEQUENCE
    | RSEQUENCE
    | FALLBACK
    | RFALLBACK
    | id          // ambigulty
    ;


object
    : LBC (objectPair (COMMA objectPair)* COMMA? )? RBC
    ;

objectPair
    : string COLON message
    ;


array
    : LBR (message (COMMA message)* COMMA? )? RBR
    ;

bool
    : TRUE
    | FALSE
    ;

num
    : NUMBER
    ;

string
    : STRING
    ;
id
    : ID
    ;
```

## Lexer

```antlrv4
lexer grammar TreeLexer;

ROOT: 'ROOT';
PARALLEL : 'parallel';

SEQUENCE : 'sequence';
MSEQUENCE : 'm_sequence';
RSEQUENCE : 'r_sequence';

FALLBACK: 'fallback';
RFALLBACK : 'r_fallback';

ARRAY_T: 'array';
NUM_T: 'num';
OBJECT_T: 'object';
STRING_T: 'string';
BOOL_T: 'bool';
TREE_T: 'tree';
IMPORT: 'import';

ID : [-_a-zA-Z]+ (INT | [-_a-zA-Z]+)*  ;

COMMA : ',';
COLON : ':';
SEMI : ';';
DOT_DOT : '..';

EQ  : '=';
EQ_A  : '=>';

LPR  : '(';
RPR  : ')';

LBC  : '{';
RBC  : '}';

LBR  : '[';
RBR  : ']';

TRUE : 'TRUE';

FALSE : 'FALSE';

STRING  : '"' (ESC | SAFECODEPOINT)* '"' ;

NUMBER  : '-'? INT ('.' [0-9] +)? EXP? ;

Whitespace: [ \t]+ -> skip ;

Newline :   (   '\r' '\n'? | '\n') -> skip ;

BlockComment :   '/*' .*? '*/' -> skip ;

LineComment :   '//' ~[\r\n]* -> skip ;

fragment ESC : '\\' (["\\/bfnrt] | UNICODE) ;

fragment UNICODE : 'u' HEX HEX HEX HEX ;
fragment HEX : [0-9a-fA-F] ;

fragment SAFECODEPOINT : ~ ["\\\u0000-\u001F] ;
fragment INT : '0' | [1-9] [0-9]* ;
fragment EXP : [Ee] [+\-]? [0-9]+ ;
```