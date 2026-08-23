"""Test fixtures: an in-memory fake of the Forester HTTP server."""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest


class FakeForesterHandler(BaseHTTPRequestHandler):
    blackboard = {}
    locks = set()
    tracer = []

    def log_message(self, *args):
        pass

    def _send(self, status, body=None, content_type="application/json"):
        if body is None:
            data = b""
        elif isinstance(body, str):
            data = body.encode("utf-8")
        else:
            data = json.dumps(body).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            return self._send(200, "OK", "text/plain")
        if path == "/openapi.json":
            return self._send(
                200, {"info": {"title": "Forester HTTP API", "version": "0.6.0"}}
            )
        if path == "/tracer/print":
            return self._send(200, "\n".join(self.tracer), "text/plain")
        if path.startswith("/bb/"):
            parts = path[len("/bb/"):].split("/")
            key = parts[0]
            action = parts[1] if len(parts) > 1 else None
            if action == "take":
                return self._send(200, self.blackboard.pop(key, None))
            if action == "lock":
                self.locks.add(key)
                return self._send(200)
            if action == "unlock":
                self.locks.discard(key)
                return self._send(200)
            if action == "locked":
                return self._send(200, key in self.locks)
            if action == "contains":
                return self._send(200, key in self.blackboard)
            return self._send(200, self.blackboard.get(key))
        return self._send(404, "not found")

    def do_POST(self):
        path = self.path.split("?")[0]
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length)) if length else None
        if path == "/tracer/custom":
            self.tracer.append(body.get("text", ""))
            return self._send(200)
        if path.startswith("/bb/"):
            key = path[len("/bb/"):]
            self.blackboard[key] = body
            return self._send(200)
        return self._send(404, "not found")


@pytest.fixture()
def server():
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), FakeForesterHandler)
    FakeForesterHandler.blackboard = {}
    FakeForesterHandler.locks = set()
    FakeForesterHandler.tracer = []
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{httpd.server_address[1]}"
    yield url
    httpd.shutdown()
    thread.join()
