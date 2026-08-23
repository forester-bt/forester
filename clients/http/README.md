# Forester HTTP clients

Generated HTTP clients for the **Forester** runtime HTTP API. Each client talks
to the HTTP server embedded in a Forester instance and exposes the
**blackboard** and the **tracer** to remote actions.

| Language | Directory | Package                                   | Registry |
| -------- | --------- | ----------------------------------------- | -------- |
| Python   | `python/` | `forester-client-http`                    | [PyPI](https://pypi.org) |
| Rust     | `rust/`   | `forester-client-http`                    | [crates.io](https://crates.io) |

Both clients are generated from the same OpenAPI specification, `openapi.json`.

## Generating the OpenAPI spec

The spec is produced by the `gen-openapi` binary of the `forester-rs` crate:

```shell
cargo run --bin gen-openapi clients/http/openapi.json
```

## Publishing

### Python (PyPI)

```shell
cd clients/http/python
pip install build twine
python -m build
python -m twine upload dist/*
```

### Rust (crates.io)

```shell
cd clients/http/rust
cargo publish
```
