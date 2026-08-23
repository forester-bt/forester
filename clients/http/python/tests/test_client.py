"""Tests for the Forester Python HTTP client."""

from forester_client_http import ForesterClient, ForesterHttpError, CustomEvent


def test_health_and_openapi(server):
    client = ForesterClient(server)
    assert client.health() == "OK"
    spec = client.openapi()
    assert spec["info"]["title"] == "Forester HTTP API"


def test_blackboard_roundtrip(server):
    client = ForesterClient(server)

    assert client.contains("count") is False
    client.put("count", 42)
    assert client.contains("count") is True
    assert client.get("count") == 42

    assert client.take("count") == 42
    assert client.get("count") is None

    client.put("config", {"timeout": 5, "tags": ["a", "b"]})
    assert client.get("config") == {"timeout": 5, "tags": ["a", "b"]}


def test_lock_unlock(server):
    client = ForesterClient(server)
    client.put("k", "v")
    assert client.is_locked("k") is False
    client.lock("k")
    assert client.is_locked("k") is True
    client.unlock("k")
    assert client.is_locked("k") is False


def test_tracer(server):
    client = ForesterClient(server)
    client.trace("hello", 1)
    content = client.print_trace()
    assert "hello" in content


def test_error_on_bad_status(server):
    client = ForesterClient(server)
    try:
        client._request("GET", "/nope")
    except ForesterHttpError as e:
        assert e.status == 404
    else:  # pragma: no cover
        raise AssertionError("expected ForesterHttpError")
