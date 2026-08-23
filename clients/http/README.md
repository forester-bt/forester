# Forester HTTP clients

Generated HTTP clients for the **Forester** runtime HTTP API. Each client talks
to the HTTP server embedded in a Forester instance and exposes the
**blackboard** and the **tracer** to remote actions.

| Language | Directory | Package                                   | Registry |
| -------- | --------- | ----------------------------------------- | -------- |
| Python   | `python/` | `forester-client-http`                    | [PyPI](https://pypi.org) |
| Rust     | `rust/`   | `forester-client-http`                    | [crates.io](https://crates.io) |

Both clients are generated from the same OpenAPI specification, `openapi.json`.

## Remote action (RA) wire types

Besides the blackboard/tracer API, each client also ships the wire types used
by the remote action protocol, so a remote action can deserialize the request
the runtime POSTs to it and serialize the `TickResult` it must answer with:

- `RemoteActionRequest` — what the runtime sends (`tick`, `args`, `serv_url`).
- `RtArgument` — a named argument inside the request.
- `TickResult` — the response (`Success`, `Running`, `Failure(reason)`).

| Language | Location                                       |
| -------- | ---------------------------------------------- |
| Python   | `forester_client_http.ra`                      |
| Rust     | `forester_client_http::ra`                     |

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
