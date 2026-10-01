"""Capture a Fusion MCP screenshot into a local PNG without printing base64."""

import base64
import json
import pathlib
import sys

from mcp_local import call


def main():
    session, destination = sys.argv[1:3]
    direction = sys.argv[3] if len(sys.argv) > 3 else 'iso-top-right'
    result = call(session, "tools/call", {
        "name": "fusion_mcp_read",
        "arguments": {
            "queryType": "screenshot",
            "width": 1400,
            "height": 1000,
            "direction": direction,
            "transparentBackground": False,
        },
    })
    content = result["result"]["content"]
    for item in content:
        if item.get("type") == "image":
            data = item.get("data") or item.get("base64Data")
            pathlib.Path(destination).write_bytes(base64.b64decode(data))
            print(json.dumps({"path": destination, "bytes": pathlib.Path(destination).stat().st_size}))
            return
        if item.get("type") == "text":
            parsed = json.loads(item["text"])
            data = parsed.get("base64Data")
            if data:
                pathlib.Path(destination).write_bytes(base64.b64decode(data))
                print(json.dumps({"path": destination, "bytes": pathlib.Path(destination).stat().st_size}))
                return
    raise RuntimeError('Fusion did not return image data: %r' % [item.get('type') for item in content])


if __name__ == "__main__":
    main()
