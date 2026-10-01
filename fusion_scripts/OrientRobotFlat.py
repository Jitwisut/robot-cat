"""Restore robot2's native Z-up pose and inspect its front/top view.

This intentionally does not save the cloud document; inspect the captured
image before committing the changed occurrence transforms.
"""
import adsk.core
import adsk.fusion
import json
import os


PREVIEW = '/Users/jitwisutthobut/Desktop/robot/exports/views/robot2_flat_preview.png'


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
    expected = (
        (1, 0, 0, 0),
        (0, 0, 1, 0),
        (0, -1, 0, 0),
    )
    for occurrence in occurrences:
        transform = occurrence.transform2
        for row, cells in enumerate(expected):
            for col, value in enumerate(cells):
                if abs(transform.getCell(row, col) - value) > 0.001:
                    raise RuntimeError('Unexpected pose of ' + occurrence.component.name)

    identity = adsk.core.Matrix3D.create()
    if not root.transformOccurrences(
            occurrences, [identity for _ in occurrences], True):
        raise RuntimeError('Could not restore the flat assembly pose')
    for occurrence in root.occurrences:
        transform = occurrence.transform2
        for i in range(3):
            for j in range(4):
                expected_cell = 1 if i == j else 0
                if abs(transform.getCell(i, j) - expected_cell) > 0.001:
                    raise RuntimeError('Pose verification failed for ' + occurrence.component.name)

    viewport = app.activeViewport
    camera = viewport.camera
    camera.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    camera.eye = adsk.core.Point3D.create(0, 0, 35)
    camera.target = adsk.core.Point3D.create(0, 0, 0)
    camera.upVector = adsk.core.Vector3D.create(0, 1, 0)
    camera.isSmoothTransition = False
    viewport.camera = camera
    viewport.refresh()
    adsk.doEvents()
    if not viewport.setCurrentAsTop():
        raise RuntimeError('Could not make +Z the document Top direction')

    camera = viewport.camera
    camera.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    camera.eye = adsk.core.Point3D.create(28, 32, 27)
    camera.target = adsk.core.Point3D.create(0, 0, 4)
    camera.upVector = adsk.core.Vector3D.create(0, 0, 1)
    camera.isSmoothTransition = False
    viewport.camera = camera
    viewport.fit()
    viewport.refresh()
    adsk.doEvents()
    os.makedirs(os.path.dirname(PREVIEW), exist_ok=True)
    if not viewport.saveAsImageFile(PREVIEW, 1600, 1100):
        raise RuntimeError('Could not capture flat preview')
    print(json.dumps({
        'pose': 'root occurrences at identity; document Top set to +Z',
        'groups': [o.component.name for o in root.occurrences],
        'preview': PREVIEW,
        'preview_bytes': os.path.getsize(PREVIEW),
        'document_modified': doc.isModified,
    }))
