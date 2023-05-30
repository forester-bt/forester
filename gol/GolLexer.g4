lexer grammar GolLexer;

ROOT: 'ROOT';
PARALLEL : 'parallel';

SEQUENCE : 'sequence';
MSEQUENCE : 'm_sequence';
RSEQUENCE : 'r_sequence';

FALLBACK: 'fallback';
RFALLBACK : 'r_fallback';

TUPLE_T: 'tuple';
NUM_T: 'num';
OBJECT_T: 'object';
STRING_T: 'string';
BOOL_T: 'bool';
TREE_T: 'tree';

ID : [-_a-zA-Z]+ (INT | [-_a-zA-Z]+)*  ;

COMMA : ',';
SEMI : ':';

EQ  : '=';

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