import adsk.core, adsk.fusion, json, math


def base_box(root):
    for occ in root.allOccurrences:
        for body in occ.bRepBodies:
            if body.name == 'Base_Plate_Al':
                bb = body.boundingBox
                return [[round(bb.minPoint.x*10, 2), round(bb.minPoint.y*10, 2), round(bb.minPoint.z*10, 2)],
                        [round(bb.maxPoint.x*10, 2), round(bb.maxPoint.y*10, 2), round(bb.maxPoint.z*10, 2)]]
    raise RuntimeError('Base plate not found')


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    before = base_box(root)
    mat = adsk.core.Matrix3D.create()
    mat.setToRotation(-math.pi/2, adsk.core.Vector3D.create(1, 0, 0),
                      adsk.core.Point3D.create(0, 0, 0))
    occs = list(root.occurrences)
    if not root.transformOccurrences(occs, [mat for _ in occs], True):
        raise RuntimeError('Root occurrence transform failed')
    after = base_box(root)
    if after != [[-80.0, 3.0, -90.0], [80.0, 5.0, 90.0]]:
        raise RuntimeError('Base plate alignment verification failed: %r' % after)
    app.activeDocument.save('Competition variant: FingerTech switch and front spinner only')
    post_save = base_box(root)
    if post_save != after:
        raise RuntimeError('Alignment changed after save: %r' % post_save)
    print(json.dumps({'before': before, 'after': after, 'post_save': post_save,
                      'saved': not app.activeDocument.isModified,
                      'root_occurrences': [occ.component.name for occ in occs]}))
