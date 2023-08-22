import pytest

from src.forester_http.client import *


@pytest.mark.easy_operation
def test_smoke():
    client = ForesterHttpClient("http://localhost:9999")
    resp = client.put("k", 1)
    print(f"put k => {resp}")

    resp = client.get("k")
    print(f"get k => {resp.json()}")

    assert True
