"""Place robot2 on the existing Y-up Fusion layout grid (XZ plane)."""
import adsk.core
import adsk.fusion
import json
import math
import os


PREVIEW = '/Users/jitwisutthobut/Desktop/robot/exports/views/robot2_grid_aligned_preview.png'


def run(_context):
    app = adsk.core.Application.get()
    doc = app.activeDocument
    if doc.name != 'robot2':
        raise RuntimeError('Activate robot2 before changing the assembly')
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    occurrences = list(root.occurrences)
    if len(occurrences) != 7:
        raise RuntimeError('Unexpected number of root groups: %s' % len(occurrences))
    for occurrence in occurrences:
        transform = occurrence.transform2
        for i in range(3):
            for j in range(4):
                expected = 1 if i == j else 0
                if abs(transform.getCell(i, j) - expected) > 0.001:
                    raise RuntimeError('Expected identity pose: ' + occurrence.component.name)

    pose = adsk.core.Matrix3D.create()
    pose.setToRotation(-math.pi / 2,
                       adsk.core.Vector3D.create(1, 0, 0),
                       adsk.core.Point3D.create(0, 0, 0))
    if not root.transformOccurrences(occurrences,
                                     [pose for _ in occurrences], True):
        raise RuntimeError('Could not align the robot with the XZ grid')
    for occurrence in root.occurrences:
        transform = occurrence.transform2
        for i in range(3):
            for j in range(4):
                if abs(transform.getCell(i, j) - pose.getCell(i, j)) > 0.001:
                    raise RuntimeError('Pose verification failed: ' + occurrence.component.name)

    viewport = app.activeViewport
    camera = viewport.camera
    camera.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    camera.eye = adsk.core.Point3D.create(28, 27, -32)
    camera.target = adsk.core.Point3D.create(0, 4, 0)
    camera.upVector = adsk.core.Vector3D.create(0, 1, 0)
    camera.isSmoothTransition = False
    viewport.camera = camera
    viewport.fit()
    viewport.refresh()
    adsk.doEvents()
    os.makedirs(os.path.dirname(PREVIEW), exist_ok=True)
    if not viewport.saveAsImageFile(PREVIEW, 1600, 1100):
        raise RuntimeError('Could not capture the aligned assembly')
    print(json.dumps({
        'pose': 'base on XZ grid, Y up',
        'groups': [o.component.name for o in root.occurrences],
        'preview': PREVIEW,
        'preview_bytes': os.path.getsize(PREVIEW),
        'document_modified': doc.isModified,
    }))
