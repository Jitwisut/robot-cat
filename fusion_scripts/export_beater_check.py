"""Save the live beater geometry/interference check returned by Fusion MCP."""

import json
import pathlib
import sys

from mcp_local import call


session, source, destination = sys.argv[1:4]
result = call(session, "tools/call", {
    "name": "fusion_mcp_execute",
    "arguments": {"featureType": "script", "object": {
        "script": pathlib.Path(source).read_text(), "readOnly": True}},
})
outer = json.loads(result["result"]["content"][0]["text"])
if not outer.get("success"):
    raise RuntimeError(outer.get("error", outer))
data = json.loads(outer["message"])
pathlib.Path(destination).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
print(json.dumps({"path": destination, "physical_cad_mass_g": data["physical_cad_mass_g"],
                  "body_count": data["body_count"], "interference_count": len(data["interference"])}))
