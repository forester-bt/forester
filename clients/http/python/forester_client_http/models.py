"""Data models for the Forester HTTP API.

The models mirror the OpenAPI specification served by the Forester HTTP
server (see ``openapi.json``). The blackboard exchanges a free-form JSON
value (:class:`RtValue`), while the tracer records :class:`CustomEvent`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Union

__all__ = ["RtValue", "CustomEvent"]

# A free-form value stored in the blackboard: string, number, boolean,
# array or object.
RtValue = Union[str, int, float, bool, None, List[Any], Dict[str, Any]]


@dataclass
class CustomEvent:
    """A custom event to record in the tracer.

    Attributes:
        text: The text of the event.
        tick: The tick the event belongs to.
    """

    text: str
    tick: int

    def to_dict(self) -> Dict[str, Any]:
        return {"text": self.text, "tick": self.tick}
