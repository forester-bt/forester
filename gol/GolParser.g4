parser grammar GolParser;

definitions
    : definition* EOF
    ;

definition
    : tree_type id params? calls?
    ;

call
    : id args?
    | tree_type id? calls
    ;

calls
    : LBC call* RBC
    | call
    ;


arg
    : id (EQ message | EQ id)?
    | message
    ;

args
    : LPR (arg (COMMA arg)* COMMA?)? RPR
    ;

params
    : LPR param (COMMA param)* COMMA? RPR
    ;

param
    : id SEMI mes_type
    ;

message
    : string
    | num
    | bool
    | array
    | object
    | call
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
    | id
    ;


object
    : LBC (objectPair (COMMA objectPair)* COMMA? )? RBC
    ;

objectPair
    : string SEMI message
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