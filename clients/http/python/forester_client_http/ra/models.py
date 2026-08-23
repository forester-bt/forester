"""Wire models for the Forester remote action (RA) protocol.

These types are not part of the blackboard/tracer HTTP server; they describe
the request that the Forester runtime POSTs to a remote action and the
``TickResult`` that the remote action must respond with.

See ``openapi.json`` -> ``components.schemas``:
``RemoteActionRequest``, ``RtArgument`` and ``TickResult``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Union

from ..models import RtValue

__all__ = ["RtArgument", "RemoteActionRequest", "TickResult"]

# The wire format of TickResult mirrors the serde (externally tagged)
# representation of the Rust enum:
#   Success           -> "Success"
#   Running           -> "Running"
#   Failure(reason)   -> {"Failure": reason}


@dataclass
class RtArgument:
    """A single named argument sent to the remote action.

    Attributes:
        name: The argument name.
        value: The argument value (free-form JSON).
    """

    name: str
    value: RtValue

    def to_dict(self) -> Dict[str, Any]:
        return {"name": self.name, "value": self.value}

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> "RtArgument":
        return RtArgument(name=data["name"], value=data["value"])


@dataclass
class RemoteActionRequest:
    """The request that the Forester runtime POSTs to a remote action.

    Attributes:
        tick: The current tick of the tree.
        args: The arguments passed to the remote action.
        serv_url: The URL of the embedded Forester HTTP server exposing the
            blackboard and the tracer.
    """

    tick: int
    args: List[RtArgument]
    serv_url: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "tick": self.tick,
            "args": [a.to_dict() for a in self.args],
            "serv_url": self.serv_url,
        }

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> "RemoteActionRequest":
        return RemoteActionRequest(
            tick=data["tick"],
            args=[RtArgument.from_dict(a) for a in data["args"]],
            serv_url=data["serv_url"],
        )


@dataclass
class TickResult:
    """The result that a remote action must respond with.

    Use the :meth:`success`, :meth:`running` and :meth:`failure` helpers to
    build a result and :meth:`to_dict` to serialize it to the wire format.
    """

    status: str
    reason: Optional[str] = None

    @staticmethod
    def success() -> "TickResult":
        return TickResult("Success")

    @staticmethod
    def running() -> "TickResult":
        return TickResult("Running")

    @staticmethod
    def failure(reason: str) -> "TickResult":
        return TickResult("Failure", reason)

    def to_dict(self) -> Union[str, Dict[str, str]]:
        if self.status == "Success":
            return "Success"
        if self.status == "Running":
            return "Running"
        return {"Failure": self.reason or ""}

    @staticmethod
    def from_dict(data: Union[str, Dict[str, Any]]) -> "TickResult":
        if data == "Success":
            return TickResult.success()
        if data == "Running":
            return TickResult.running()
        if isinstance(data, dict) and "Failure" in data:
            return TickResult.failure(data["Failure"])
        raise ValueError(f"invalid TickResult: {data!r}")
