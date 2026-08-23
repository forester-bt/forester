"""Remote action (RA) wire types for the Forester HTTP client.

This package describes the request the Forester runtime sends to a remote
action and the ``TickResult`` it expects back.
"""

from .models import RemoteActionRequest, RtArgument, TickResult

__all__ = ["RemoteActionRequest", "RtArgument", "TickResult"]
