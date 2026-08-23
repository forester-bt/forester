"""Synchronous HTTP client for the Forester runtime.

This client talks to the HTTP server embedded in a Forester instance and
exposes the blackboard and the tracer to remote actions. It is generated
from the OpenAPI specification produced by ``cargo run --bin gen-openapi``
(see ``openapi.json``).
"""

from __future__ import annotations

from typing import Any, Dict, Optional

import requests

from .exceptions import ForesterHttpError
from .models import CustomEvent, RtValue

__all__ = ["ForesterClient"]


class ForesterClient:
    """A client for the Forester HTTP API.

    Args:
        base_url: The URL of the Forester HTTP server, e.g.
            ``"http://127.0.0.1:46123"``.
        timeout: Request timeout in seconds.
        session: An optional :class:`requests.Session` to reuse. When not
            provided, a new session is created.
    """

    def __init__(
        self,
        base_url: str,
        timeout: float = 30.0,
        session: Optional[requests.Session] = None,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self._session = session or requests.Session()

    def close(self) -> None:
        """Close the underlying HTTP session."""
        self._session.close()

    def __enter__(self) -> "ForesterClient":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    # -- low-level helpers ---------------------------------------------------

    def _url(self, path: str) -> str:
        return f"{self.base_url}{path}"

    def _request(
        self,
        method: str,
        path: str,
        *,
        json: Any = None,
        raw: bool = False,
    ) -> Any:
        response = self._session.request(
            method,
            self._url(path),
            json=json,
            timeout=self.timeout,
        )
        if response.status_code >= 400:
            raise ForesterHttpError(response.status_code, response.text)
        if raw:
            return response.text
        if not response.content:
            return None
        return response.json()

    # -- health and metadata ------------------------------------------------

    def health(self) -> str:
        """Return the health status. Always ``"OK"``."""
        return self._request("GET", "/", raw=True)

    def openapi(self) -> Dict[str, Any]:
        """Return the OpenAPI specification of the server."""
        return self._request("GET", "/openapi.json")

    # -- blackboard ----------------------------------------------------------

    def get(self, key: str) -> RtValue:
        """Read the value stored under ``key`` (``None`` if absent)."""
        return self._request("GET", f"/bb/{key}")

    def put(self, key: str, value: RtValue) -> None:
        """Store ``value`` under ``key``."""
        self._request("POST", f"/bb/{key}", json=value)

    def take(self, key: str) -> RtValue:
        """Read and remove the value stored under ``key``."""
        return self._request("GET", f"/bb/{key}/take")

    def lock(self, key: str) -> None:
        """Lock ``key`` so it cannot be taken."""
        self._request("GET", f"/bb/{key}/lock")

    def unlock(self, key: str) -> None:
        """Unlock ``key``."""
        self._request("GET", f"/bb/{key}/unlock")

    def is_locked(self, key: str) -> bool:
        """Return whether ``key`` is locked."""
        return self._request("GET", f"/bb/{key}/locked")

    def contains(self, key: str) -> bool:
        """Return whether ``key`` exists."""
        return self._request("GET", f"/bb/{key}/contains")

    # -- tracer --------------------------------------------------------------

    def trace(self, text: str, tick: int) -> None:
        """Record a custom event in the tracer."""
        self._request("POST", "/tracer/custom", json=CustomEvent(text, tick).to_dict())

    def print_trace(self) -> str:
        """Return the tracer content as a string."""
        return self._request("GET", "/tracer/print", raw=True)
