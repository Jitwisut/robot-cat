"""Idempotently restore robot2 to its XZ floor orientation after visual edits."""

import adsk.core
import adsk.fusion
import json
import math


def bbox(body):
    box = body.boundingBox
    return [[round(box.minPoint.x*10, 2), round(box.minPoint.y*10, 2), round(box.minPoint.z*10, 2)],
            [round(box.maxPoint.x*10, 2), round(box.maxPoint.y*10, 2), round(box.maxPoint.z*10, 2)]]


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design or app.activeDocument.name != 'robot2':
        raise RuntimeError('Open robot2 first')
    root = design.rootComponent
    base = next(body for occ in root.allOccurrences for body in occ.bRepBodies
                if body.name == 'Base_Plate_Al')
    expected = [[-80.0, 3.0, -90.0], [80.0, 5.0, 90.0]]
    native = [[-80.0, -90.0, 3.0], [80.0, 90.0, 5.0]]
    before = bbox(base)
    if before == native:
        matrix = adsk.core.Matrix3D.create()
        matrix.setToRotation(-math.pi/2, adsk.core.Vector3D.create(1, 0, 0),
                             adsk.core.Point3D.create(0, 0, 0))
        occurrences = list(root.occurrences)
        if not root.transformOccurrences(occurrences,
                                         [matrix for _ in occurrences], True):
            raise RuntimeError('Root occurrence alignment failed')
    elif before != expected:
        raise RuntimeError('Unexpected base orientation: %r' % before)
    after = bbox(base)
    if after != expected:
        raise RuntimeError('Base still not aligned: %r' % after)
    app.activeDocument.save('Restore beater concept alignment after visual highlight')
    print(json.dumps({'before': before, 'after': bbox(base),
                      'saved': not app.activeDocument.isModified}))
