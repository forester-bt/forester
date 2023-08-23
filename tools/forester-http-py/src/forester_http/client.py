# This file is part of Forester HTTP API.
import json
from enum import Enum
from typing import Optional, List

import requests


class TickResult(Enum):
    """
    The result that the node returns
    It can be Success, Failure or Running
    The Failure contains the error message
    The Running means that the node is still running
    and it will be executed on the next tick
    """

    Success = 1
    Failure = 2
    Running = 3

    # Failure constructor with a string parameter
    def __init__(self, value):
        self._value_ = value

    def __str__(self):
        return self.name


class RtArgument:
    """The argument that is sent from the Forester instance

    * The name of the argument
    * The value of the argument is a json
    """

    def __init__(self, name: str, value: str) -> None:
        self.name = name
        self.value = value


class RemoteActionRequest:
    """The request that is sent from the Forester instance

    * It has the current tick and the arguments in the action from tree
    """

    def __init__(self, tick: int, args: List[RtArgument], serv_url: str) -> None:
        self.tick = tick
        self.args = args
        self.serv_url = serv_url

    def from_bytes(self: bytes) -> "RemoteActionRequest":
        """
        Creates a RemoteActionRequest from a json string

        Example:
            >>> request = RemoteActionRequest.from_bytes(b'{"tick": 1, "args": [{"name": "arg1", "value": "value1"}]}')

        """

        body = json.loads(self.decode("utf-8"))

        return RemoteActionRequest(
            body["tick"],
            [RtArgument(arg["name"], arg["value"]) for arg in body["args"]],
            body["serv_url"],
        )


class ForesterHttpClient:
    """
    The ForesterHttpClient class provides a way to interact with the Forester HTTP API.
    """

    def __init__(self, base_url: str) -> None:
        self.api = ForesterHttpApi(base_url)
        self.timeout: Optional[int] = None

    def new_trace_event(self, tick: int, text: str) -> requests.Response:
        """
        Creates a new trace event.

        Args:
            tick: The tick of the trace event.
            text: The text of the trace event.

        Returns: The response from the server. It contains the status code only.
        """
        url = self.api.trace_event()
        data = {"text": text, "tick": tick}
        return requests.post(url, json=data, timeout=self.timeout)

    def print_trace(self) -> requests.Response:
        """
        Prints the trace or if the file is big the tail of the trace (last 100 lines).

        Returns: The response from the server. It contains the status code and text.
        """
        url = self.api.print_trace()
        return requests.get(url, timeout=self.timeout)

    def lock(self, key: str) -> requests.Response:
        """
        Locks the key in the blackboard.

        Args:
            key: The key to lock.

        Returns: The response from the server. It contains the status code only.
        """
        url = self.api.lock(key)
        return requests.get(url, timeout=self.timeout)

    def unlock(self, key: str) -> requests.Response:
        """
        Unlocks the key in the blackboard.

        Args:
            key: The key to unlock.

        Returns: The response from the server. It contains the status code only.
        """
        url = self.api.unlock(key)
        return requests.get(url, timeout=self.timeout)

    def locked(self, key: str) -> requests.Response:
        """
        Checks if the key is locked in the blackboard.

        Args:
            key: The key to check.

        Returns: The response from the server. It contains the status code and boolean.
        """
        url = self.api.locked(key)
        return requests.get(url, timeout=self.timeout)

    def contains(self, key: str) -> requests.Response:
        """
        Checks if the key is in the blackboard.

        Args:
            key: The key to check.

        Returns: The response from the server. It contains the status code and boolean.
        """
        url = self.api.contains(key)
        return requests.get(url, timeout=self.timeout)

    def take(self, key: str) -> requests.Response:
        """
        Takes the key from the blackboard.

        Args:
            key: The key to take.

        Returns: The response from the server. It contains the status code and json.
        """
        url = self.api.take(key)
        return requests.get(url, timeout=self.timeout)

    def get(self, key: str) -> requests.Response:
        """
        Gets the key from the blackboard.

        Args:
            key: The key to get.

        Returns: The response from the server. It contains the status code and json.
        """
        url = self.api.get(key)
        return requests.get(url, timeout=self.timeout)

    def put(self, key: str, value) -> requests.Response:
        """
        Puts the key to the blackboard.

        Args:
            key: The key to put.
            value: The value to put.

        Returns: The response from the server. It contains the status code only.
        """
        url = self.api.put(key)
        return requests.post(url, json=value, timeout=self.timeout)


class TickError(Exception):
    def __init__(self, message: str) -> None:
        super().__init__(message)


class CustomEvent:
    def __init__(self, text: str, tick: int) -> None:
        self.text = text
        self.tick = tick


class ForesterHttpApi:
    """
    The ForesterHttpApi class provides a way to interact with the Forester HTTP API.
    """

    def __init__(self, base: str) -> None:
        self.base = base

    def trace_event(self) -> str:
        """
        Creates a new trace event.

        Returns:
            The URL of the new trace event.
        """
        return f"{self.base}/tracer/custom"

    def print_trace(self) -> str:
        """
        Prints the trace or if the file is big the tail of the trace (last 100 lines).

        Returns:
            The URL of the trace print endpoint.
        """
        return f"{self.base}/tracer/print"

    def lock(self, key: str) -> str:
        """
        Locks the key in the blackboard.

        Args:
            key: The key to lock.

        Returns:
            The URL of the lock endpoint.
        """
        return f"{self.base}/bb/{key}/lock"

    def unlock(self, key: str) -> str:
        """
        Unlocks the key in the blackboard.

        Args:
            key: The key to unlock.

        Returns:
            The URL of the unlock endpoint.
        """
        return f"{self.base}/bb/{key}/unlock"

    def locked(self, key: str) -> str:
        """
        Checks if the key is locked in the blackboard.

        Args:
            key: The key to check.

        Returns:
            The URL of the locked endpoint.
        """
        return f"{self.base}/bb/{key}/locked"

    def contains(self, key: str) -> str:
        """
        Checks if the key is in the blackboard.

        Args:
            key: The key to check.

        Returns:
            The URL of the contains endpoint.
        """
        return f"{self.base}/bb/{key}/contains"

    def take(self, key: str) -> str:
        """
        Takes the key from the blackboard.

        Args:
            key: The key to take.

        Returns:
            The URL of the take endpoint.
        """
        return f"{self.base}/bb/{key}/take"

    def get(self, key: str) -> str:
        """
        Gets the key from the blackboard.

        Args:
            key: The key to get.

        Returns:
            The URL of the get endpoint.
        """
        return f"{self.base}/bb/{key}"

    def put(self, key: str) -> str:
        """
        Puts the key to the blackboard.

        Args:
            key: The key to put.

        Returns:
            The URL of the put endpoint.
        """
        return f"{self.base}/bb/{key}"
