"""Capture robot2's saved orientation for final visual verification."""
import adsk.core
import adsk.fusion
import json
import os


PATH = '/Users/jitwisutthobut/Desktop/robot/exports/views/robot2_flat_saved.png'


def run(_context):
    app = adsk.core.Application.get()
    doc = app.activeDocument
    if doc.name != 'robot2':
        raise RuntimeError('robot2 must be active')
    root = adsk.fusion.Design.cast(app.activeProduct).rootComponent
    pose = []
    for occurrence in root.occurrences:
        transform = occurrence.transform2
        pose.append({
            'group': occurrence.component.name,
            'identity': all(
                abs(transform.getCell(i, j) - (1 if i == j else 0)) < 0.001
                for i in range(3) for j in range(4)),
        })
    viewport = app.activeViewport
    viewport.refresh()
    adsk.doEvents()
    if not viewport.saveAsImageFile(PATH, 1600, 1100):
        raise RuntimeError('Capture failed')
    print(json.dumps({
        'saved': not doc.isModified,
        'pose': pose,
        'image': PATH,
        'image_bytes': os.path.getsize(PATH),
    }))
