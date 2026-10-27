"""Import the V5 concept assembly STEP into a new Fusion document and export a Z-up F3D.

Source: robot_v5/cad/output/ROBOT_V5_CONCEPT_ASSEMBLY.step (from robot_v5/cad/build_v5.py, park pose).
Sets the document Top to +Z and Home to an oblique view above the robot (AGENTS.md convention: Z up,
nose +Y, floor XY at Z=0) without moving any component, captures a preview and exports a local F3D.
Does not save to the Fusion cloud.
"""
import adsk.core
import adsk.fusion
import json
from pathlib import Path

OUT = Path('/Users/jitwisutthobut/Desktop/robot/robot_v5/cad/output')
STEP = OUT / 'ROBOT_V5_CONCEPT_ASSEMBLY.step'
EXPECT_CM = [-9.6, 9.6, -9.5, 12.31, 0.0, 22.39]          # build_report bounds_park_mm / 10


def bounds(root):
    b = root.boundingBox
    return [round(v, 2) for v in (b.minPoint.x, b.maxPoint.x, b.minPoint.y, b.maxPoint.y, b.minPoint.z, b.maxPoint.z)]


def run(_context: str):
    app = adsk.core.Application.get()
    im = app.importManager
    doc = im.importToNewDocument(im.createSTEPImportOptions(str(STEP)))
    adsk.doEvents()
    doc = app.activeDocument
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    occs = list(root.allOccurrences)
    n_bodies = sum(1 for _ in root.bRepBodies) + sum(o.component.bRepBodies.count for o in occs)
    original = [o.transform2.asArray() for o in occs]
    bb = bounds(root)
    if max(abs(a - e) for a, e in zip(bb, EXPECT_CM)) > 0.3:
        raise RuntimeError('Imported bounds %s differ from the CAD %s: axes or units changed' % (bb, EXPECT_CM))

    vp = app.activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    cam.target = adsk.core.Point3D.create(0, 1.4, 5)
    cam.eye = adsk.core.Point3D.create(0, 1.4, 60)
    cam.upVector = adsk.core.Vector3D.create(0, 1, 0)
    cam.isSmoothTransition = False
    vp.camera = cam
    vp.refresh(); adsk.doEvents()
    if not vp.setCurrentAsTop():
        raise RuntimeError('Could not set document Top to +Z')

    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
    cam.target = adsk.core.Point3D.create(0, 1.4, 6)
    cam.eye = adsk.core.Point3D.create(38, 46, 36)            # front-right, above: nose +Y toward the viewer's right
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isSmoothTransition = False
    vp.camera = cam
    vp.fit()
    vp.refresh(); adsk.doEvents()
    if not vp.setCurrentAsHome(True):
        raise RuntimeError('Could not set Home view')

    if original != [o.transform2.asArray() for o in occs] or bb != bounds(root):
        raise RuntimeError('Import view setup moved geometry')
    png = OUT / 'ROBOT_V5_CONCEPT_FUSION_Z_UP.png'
    vp.saveAsImageFile(str(png), 1600, 1100)
    f3d = OUT / 'ROBOT_V5_CONCEPT.f3d'
    opts = design.exportManager.createFusionArchiveExportOptions(str(f3d))
    if not design.exportManager.execute(opts):
        raise RuntimeError('F3D export failed')
    print(json.dumps({'document': doc.name, 'occurrences': len(occs), 'bodies': n_bodies, 'bounds_cm': bb,
                      'units': design.unitsManager.defaultLengthUnits, 'top': '+Z', 'png': str(png),
                      'f3d': str(f3d), 'f3d_bytes': f3d.stat().st_size}))
    doc.close(False)
