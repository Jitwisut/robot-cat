"""Set R3's Fusion Top/Home to native +Z without moving any CAD component.

Run with ROBOT_V4_PRINT_R3 active. Keeps STEP/STL coordinates unchanged.
Exports a local F3D with the corrected view; does not save to Fusion cloud.
"""
import adsk.core
import adsk.fusion
import json
from pathlib import Path


DEST = Path('/Users/jitwisutthobut/Desktop/robot/robot_v4/print_revision/r3/output_R3/DRAFT')


def run(_context: str):
    app = adsk.core.Application.get()
    doc = app.activeDocument
    if not doc.name.startswith('ROBOT_V4_PRINT_R3'):
        raise RuntimeError('Activate ROBOT_V4_PRINT_R3 before correcting the view')
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    occurrences = list(root.occurrences)
    if len(occurrences) != 70:
        raise RuntimeError('Unexpected R3 component count')
    original = [o.transform2.asArray() for o in occurrences]
    bounds = [root.boundingBox.minPoint.asArray(), root.boundingBox.maxPoint.asArray()]

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
    if not viewport.fit():
        raise RuntimeError('Could not fit assembly in viewport')
    viewport.refresh()
    adsk.doEvents()
    if not viewport.setCurrentAsHome(True):
        raise RuntimeError('Could not set corrected Home view')

    if original != [o.transform2.asArray() for o in occurrences]:
        raise RuntimeError('Component transforms changed unexpectedly')
    if bounds != [root.boundingBox.minPoint.asArray(), root.boundingBox.maxPoint.asArray()]:
        raise RuntimeError('Geometry bounds changed unexpectedly')
    DEST.mkdir(parents=True, exist_ok=True)
    preview = DEST / 'ROBOT_V4_PRINT_R3_FUSION_Z_UP.png'
    if not viewport.saveAsImageFile(str(preview), 1600, 1100):
        raise RuntimeError('Could not capture corrected Fusion view')
    archive = DEST / 'ROBOT_V4_PRINT_R3_FUSION_Z_UP.f3d'
    options = design.exportManager.createFusionArchiveExportOptions(str(archive))
    if not design.exportManager.execute(options):
        raise RuntimeError('Could not export local Fusion archive')
    print(json.dumps({'document': doc.name, 'top': '+Z', 'ground': 'XY / Z=0',
                      'unchanged_component_transforms': len(occurrences),
                      'bounds_cm': bounds, 'preview': str(preview),
                      'archive': str(archive), 'archive_bytes': archive.stat().st_size}))
