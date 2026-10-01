import adsk.core, adsk.fusion, json


def bb(b):
    q = b.boundingBox
    return [[round(q.minPoint.x*10, 2), round(q.minPoint.y*10, 2), round(q.minPoint.z*10, 2)],
            [round(q.maxPoint.x*10, 2), round(q.maxPoint.y*10, 2), round(q.maxPoint.z*10, 2)]]


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    out = {"doc": app.activeDocument.name, "modified": app.activeDocument.isModified,
           "groups": []}
    for group in root.occurrences:
        entry = {"group": group.component.name, "occurrences": []}
        if group.component.name in ("03_Front_Spinner", "01_Chassis"):
            for occ in group.childOccurrences:
                item = {"name": occ.component.name, "bodies": []}
                for body in occ.bRepBodies:
                    item["bodies"].append({"name": body.name,
                                           "world_bbox_mm": bb(body),
                                           "mass_g": round(body.physicalProperties.mass*1000, 3)})
                entry["occurrences"].append(item)
        out["groups"].append(entry)
    print(json.dumps(out))
