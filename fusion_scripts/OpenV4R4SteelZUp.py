"""Open r4_steel/output/ROBOT_V4_R4_STEEL.step in a new Fusion document with Top = +Z (AGENTS.md).

Imports the STEP as is (no component moved), sets the document Top to +Z and Home to the oblique view above the
robot, checks the bounds against build_report.json and saves a viewport image next to the STEP.
Does not save to the Fusion cloud.
"""
import adsk.core
import adsk.fusion
import json
from pathlib import Path

OUT = Path('/Users/jitwisutthobut/Desktop/robot/robot_v4/print_revision/r4_steel/output')


def run(_context: str):
    app = adsk.core.Application.get()
    im = app.importManager
    doc = im.importToNewDocument(im.createSTEPImportOptions(str(OUT / 'ROBOT_V4_R4_STEEL.step')))
    doc.activate()
    adsk.doEvents()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    bb = root.boundingBox
    bounds = [[round(v * 10, 2) for v in bb.minPoint.asArray()], [round(v * 10, 2) for v in bb.maxPoint.asArray()]]
    want = json.loads((OUT / 'build_report.json').read_text())['bounds_mm']
    expected = [[want['xmin'], want['ymin'], want['zmin']], [want['xmax'], want['ymax'], want['zmax']]]
    if any(abs(a - b) > 0.05 for p, q in zip(bounds, expected) for a, b in zip(p, q)):
        raise RuntimeError('imported bounds %s differ from build_report %s' % (bounds, expected))

    viewport = app.activeViewport
    camera = viewport.camera
    camera.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    camera.eye = adsk.core.Point3D.create(0, 0, 40)
    camera.target = adsk.core.Point3D.create(0, 0, 0)
    camera.upVector = adsk.core.Vector3D.create(0, 1, 0)
    camera.isSmoothTransition = False
    viewport.camera = camera
    viewport.refresh()
    adsk.doEvents()
    if not viewport.setCurrentAsTop():
        raise RuntimeError('Could not set document Top to +Z')
    camera = viewport.camera
    camera.eye = adsk.core.Point3D.create(28, 32, 27)
    camera.target = adsk.core.Point3D.create(0, 0, 3.2)
    camera.upVector = adsk.core.Vector3D.create(0, 0, 1)
    camera.isSmoothTransition = False
    viewport.camera = camera
    viewport.fit()
    viewport.refresh()
    adsk.doEvents()
    if not viewport.setCurrentAsHome(True):
        raise RuntimeError('Could not set Home view')
    if bounds != [[round(v * 10, 2) for v in root.boundingBox.minPoint.asArray()], [round(v * 10, 2) for v in root.boundingBox.maxPoint.asArray()]]:
        raise RuntimeError('Geometry moved while setting the view')
    image = OUT / 'ROBOT_V4_R4_STEEL_FUSION_Z_UP.png'
    viewport.saveAsImageFile(str(image), 1600, 1100)
    print(json.dumps({'document': doc.name, 'top': '+Z', 'bounds_mm': bounds, 'bodies': root.bRepBodies.count,
                      'occurrences': root.allOccurrences.count, 'image': str(image)}))
