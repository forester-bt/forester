"""Exceptions raised by the Forester HTTP client."""

from __future__ import annotations

__all__ = ["ForesterClientError", "ForesterHttpError"]


class ForesterClientError(Exception):
    """Base class for all errors raised by the client."""


class ForesterHttpError(ForesterClientError):
    """Raised when the Forester server answers with an error status.

    Attributes:
        status: The HTTP status code returned by the server.
        message: The error message returned in the response body.
    """

    def __init__(self, status: int, message: str) -> None:
        self.status = status
        self.message = message
        super().__init__(f"Forester server returned {status}: {message}")
