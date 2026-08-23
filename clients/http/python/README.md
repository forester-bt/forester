# Forester HTTP client (Python)

Python client for the **Forester** HTTP API. Forester embeds a small HTTP
server that exposes the **blackboard** and the **tracer** to remote actions.
This package talks to that server.

The client is generated from the OpenAPI specification produced by the
Forester `gen-openapi` binary (`cargo run --bin gen-openapi`). The source
specification is kept in `clients/http/openapi.json`.

## Installation

```shell
pip install forester-client-http
```

## Usage

```python
from forester_client_http import ForesterClient

client = ForesterClient("http://127.0.0.1:46123")

# health
print(client.health())  # "OK"

# blackboard
client.put("count", 42)
assert client.get("count") == 42
assert client.contains("count") is True

value = client.take("count")  # 42, and the key is removed

client.put("config", {"timeout": 5, "tags": ["a", "b"]})
client.lock("config")
assert client.is_locked("config") is True
client.unlock("config")

# tracer
client.trace("robot armed", tick=3)
print(client.print_trace())
```

## API

| Method | Endpoint |
| ------ | -------- |
| `health()` | `GET /` |
| `openapi()` | `GET /openapi.json` |
| `get(key)` | `GET /bb/{key}` |
| `put(key, value)` | `POST /bb/{key}` |
| `take(key)` | `GET /bb/{key}/take` |
| `lock(key)` | `GET /bb/{key}/lock` |
| `unlock(key)` | `GET /bb/{key}/unlock` |
| `is_locked(key)` | `GET /bb/{key}/locked` |
| `contains(key)` | `GET /bb/{key}/contains` |
| `trace(text, tick)` | `POST /tracer/custom` |
| `print_trace()` | `GET /tracer/print` |

## Development

```shell
pip install -e .[test]
pytest
```

## License

Apache-2.0
