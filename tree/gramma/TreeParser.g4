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