# Library to create Remote Actions using Python

The Forester provides an http library that alleviates writing the remote http actions.

## Usage

The latest version can be obtained from the [test.pypi.org](https://test.pypi.org/project/forester-http/)

```shell
pip install -i https://test.pypi.org/simple/ forester-http==0.0.5
```

The contract is defined in the following way:

```python
from typing import List

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


```
 
On the other hand, the library provides a helper API `ForesterHttpApi` and Client `ForesterHttpClient`  to access the server.

## Example

The code is available in the [forester-examples](https://github.com/besok/forester-examples/tree/main/remote_action/simple_action_py) repository.

The gist is the following:

```python

import json
from http.server import BaseHTTPRequestHandler, HTTPServer

from forester_http.client import *

class MyServer(BaseHTTPRequestHandler):
    def do_POST(self):
    
        if self.path == "/action":
            content_length = int(self.headers["Content-Length"])
            # get body as json and deserialize it to RemoteActionRequest
            body = json.loads(self.rfile.read(content_length))
            req = RemoteActionRequest.from_bytes(body.encode("utf-8"))
    
            client = ForesterHttpClient(req.serv_url)
            client.put("test", "test")
    
            self.send_response(200)
            self.send_header("Content-Type", "application/json;charset=UTF-8")
            self.end_headers()
    
            self.wfile.write(json.dumps("Success").encode("utf-8"))
    
        else:
            self.send_error(404)


if __name__ == "__main__":
    webServer = HTTPServer((hostName, serverPort), MyServer)
    print("Server started http://%s:%s" % (hostName, serverPort))

    try:
        webServer.serve_forever()
    except KeyboardInterrupt:
        pass

    webServer.server_close()
    print("Server stopped.")

```

