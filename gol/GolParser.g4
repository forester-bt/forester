parser grammar GolParser;

definitions
    : definition* EOF
    ;

definition
    : tree_type id? params? items
    | tree_type id params?
    ;

item
    : tree_call
    | tree_type id? items
    ;

items
    : LBC item* RBC
    | item
    ;

tree_call
    : id args?
    ;

arg
    : id (EQ message | EQ id)?
    | message
    ;

args
    : LPR arg (COMMA arg)* COMMA? RPR
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
    | tuple
    | object
    ;

mes_type
    : TUPLE_T
    | NUM_T
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

tuple
    : LPR (message (COMMA message)* COMMA? )? RPR
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