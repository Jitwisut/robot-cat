"""Hide construction sketches and retain the beater's saved XZ alignment."""

import adsk.core
import adsk.fusion
import json
import math


def box(body):
    b = body.boundingBox
    return [[round(b.minPoint.x*10, 2), round(b.minPoint.y*10, 2), round(b.minPoint.z*10, 2)],
            [round(b.maxPoint.x*10, 2), round(b.maxPoint.y*10, 2), round(b.maxPoint.z*10, 2)]]


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design or app.activeDocument.name != 'robot2':
        raise RuntimeError('Open robot2 first')
    root = design.rootComponent
    hidden = 0
    for comp in [root] + [occ.component for occ in root.allOccurrences]:
        for sketch in comp.sketches:
            if sketch.isLightBulbOn:
                sketch.isLightBulbOn = False
                hidden += 1
    base = next(b for o in root.allOccurrences for b in o.bRepBodies
                if b.name == 'Base_Plate_Al')
    aligned = [[-80.0, 3.0, -90.0], [80.0, 5.0, 90.0]]
    native = [[-80.0, -90.0, 3.0], [80.0, 90.0, 5.0]]
    if box(base) == native:
        mat = adsk.core.Matrix3D.create()
        mat.setToRotation(-math.pi/2, adsk.core.Vector3D.create(1, 0, 0),
                          adsk.core.Point3D.create(0, 0, 0))
        groups = list(root.occurrences)
        if not root.transformOccurrences(groups, [mat for _ in groups], True):
            raise RuntimeError('Alignment failed')
    elif box(base) != aligned:
        raise RuntimeError('Unexpected base orientation: %r' % box(base))
    app.activeDocument.save('Show beater concept clearly with construction sketches hidden')
    if box(base) != aligned:
        raise RuntimeError('Save changed base orientation')
    print(json.dumps({'hidden_sketches': hidden,
                      'base_bbox_world_mm': box(base),
                      'saved': not app.activeDocument.isModified}))
