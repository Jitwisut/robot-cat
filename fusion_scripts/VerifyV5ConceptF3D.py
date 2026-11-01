"""Reopen robot_v5/cad/output/ROBOT_V5_CONCEPT.f3d and check the Z-up orientation persisted.

Opens the archive as an unsaved copy, goes to its Home view, checks bounds (floor Z=0, nose +Y),
captures ROBOT_V5_CONCEPT_FUSION_REOPENED.png and closes without saving.
"""
import adsk.core
import adsk.fusion
import json
from pathlib import Path

OUT = Path('/Users/jitwisutthobut/Desktop/robot/robot_v5/cad/output')
EXPECT_CM = [-9.6, 9.6, -9.5, 12.31, 0.0, 22.39]


def run(_context: str):
    app = adsk.core.Application.get()
    im = app.importManager
    im.importToNewDocument(im.createFusionArchiveImportOptions(str(OUT / 'ROBOT_V5_CONCEPT.f3d')))
    adsk.doEvents()
    doc = app.activeDocument
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    b = root.boundingBox
    bb = [round(v, 2) for v in (b.minPoint.x, b.maxPoint.x, b.minPoint.y, b.maxPoint.y, b.minPoint.z, b.maxPoint.z)]
    vp = app.activeViewport
    vp.goHome(False)
    vp.refresh(); adsk.doEvents()
    up = vp.camera.upVector
    png = OUT / 'ROBOT_V5_CONCEPT_FUSION_REOPENED.png'
    vp.saveAsImageFile(str(png), 1600, 1100)
    ok = max(abs(a - e) for a, e in zip(bb, EXPECT_CM)) <= 0.3
    print(json.dumps({'document': doc.name, 'bounds_cm': bb, 'bounds_match_cad': ok,
                      'home_up_vector': [round(up.x, 3), round(up.y, 3), round(up.z, 3)],
                      'occurrences': root.allOccurrences.count, 'png': str(png)}))
    doc.close(False)
