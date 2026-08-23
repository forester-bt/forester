"""Tests for the remote-action (ra) wire models."""

from forester_client_http.ra import RemoteActionRequest, RtArgument, TickResult


def test_tick_result_wire_format():
    assert TickResult.success().to_dict() == "Success"
    assert TickResult.running().to_dict() == "Running"
    assert TickResult.failure("boom").to_dict() == {"Failure": "boom"}


def test_tick_result_roundtrip():
    for result in (
        TickResult.success(),
        TickResult.running(),
        TickResult.failure("boom"),
    ):
        assert TickResult.from_dict(result.to_dict()) == result


def test_request_roundtrip():
    request = RemoteActionRequest(
        tick=1,
        args=[
            RtArgument(name="a", value=1),
            RtArgument(name="b", value=[1, 2]),
            RtArgument(name="c", value={"x": True}),
        ],
        serv_url="http://127.0.0.1:46123",
    )

    assert RemoteActionRequest.from_dict(request.to_dict()) == request
