# Contributing to Forester

Thank you for your interest in contributing to Forester!

## Feature Requests and Bug Reports

Feature requests should be reported in the
[issue page](https://github.com/besok/forester/issues). To reduce the number of
duplicates, please make sure to check the existing issues.

In general, the future request are labeled with 'enhancement' label and 'bug' label for bug reports.

If the story is not clear or big or supposed to be discussed and then broken down into the independent tasks, 
it should be labeled with 'design' label. 

## Patches / Pull Requests

All patches have to be sent on Github as [pull requests](https://github.com/besok/forester/pulls).

If you are looking for a place to start contributing to Alacritty, take a look at the
list of [issues](https://github.com/besok/forester/issues).

### Testing

To make sure no regressions were introduced, all tests should be run before sending a pull request.
The following command can be run to test Alacritty:

Additionally, if there's any functionality included which would lend itself to additional testing,
new tests should be added. These can either be in the form of Rust tests using the `#[test]`
annotation.

### Documentation

Code should be documented where appropriate. The existing code can be used as a guidance here and
the general `rustfmt` rules can be followed for formatting.

### Style

Unless otherwise specified, the Forester follows the Rust compiler's style guidelines:

https://rust-lang.github.io/api-guidelines

All comments should be fully punctuated with a trailing period. This applies both to regular and
documentation comments.

# Contact

If there are any outstanding questions about contributing to Forester, they can be asked on the issue page or
directly [me](https://github.com/besok).
