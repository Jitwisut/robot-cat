"""Small local client for the Fusion MCP server on this Mac.

Usage:
  python3 fusion_scripts/mcp_local.py SESSION read PYTHON_FILE
  python3 fusion_scripts/mcp_local.py SESSION write PYTHON_FILE
  python3 fusion_scripts/mcp_local.py SESSION read-document

The Fusion MCP server must already be running in Fusion Preferences.
"""

import json
import pathlib
import sys
import urllib.request


def call(session, method, params):
    payload = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode()
    request = urllib.request.Request(
        "http://127.0.0.1:27182/mcp",
        data=payload,
        headers={
            "Content-Type": "application/json",
            "Accept": "application/json, text/event-stream",
            "MCP-Session-Id": session,
        },
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=180) as response:
        data = response.read().decode()
    return json.loads(data)


def main():
    session, mode, *rest = sys.argv[1:]
    if mode == "read-document":
        result = call(session, "tools/call", {
            "name": "fusion_mcp_read",
            "arguments": {"queryType": "document", "operation": "open"},
        })
    else:
        source = pathlib.Path(rest[0]).read_text()
        result = call(session, "tools/call", {
            "name": "fusion_mcp_execute",
            "arguments": {
                "featureType": "script",
                "object": {"script": source, "readOnly": mode == "read"},
            },
        })
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
