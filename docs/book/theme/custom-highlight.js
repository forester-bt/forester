hljs.registerLanguage("tree", (hljs) => ({
    name: "Tree",
    keywords: {
        keyword: "import root parallel sequence m_sequence r_sequence fallback r_fallback inverter force_success force_fail repeat retry timeout delay impl cond",
        built_in: "array num object string bool tree",
        literal: "false true",
    },
    contains: [
        { // numbers
            className: 'number',
            relevance: 0,
            begin: '(-?)(\\b0[xXbBoOdD][a-fA-F0-9]+|(\\b\\d+(\\.\\d*)?f?|\\.\\d+f?)([eE][-+]?\\d+f?)?)'
        },
        {
            scope: 'string',
            relevance: 0,
            begin: '"', end: '"'
        },
        hljs.C_LINE_COMMENT_MODE, // single-line comments
        hljs.C_BLOCK_COMMENT_MODE, // comment blocks
    ],
}));

hljs.initHighlightingOnLoad();