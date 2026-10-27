"""Open robot_v5/cad/output/ROBOT_V5_CONCEPT.f3d in Fusion and leave it open at its Z-up Home view."""
import adsk.core
import json

F3D = '/Users/jitwisutthobut/Desktop/robot/robot_v5/cad/output/ROBOT_V5_CONCEPT.f3d'


def run(_context: str):
    app = adsk.core.Application.get()
    im = app.importManager
    im.importToNewDocument(im.createFusionArchiveImportOptions(F3D))
    adsk.doEvents()
    vp = app.activeViewport
    vp.goHome(False)
    vp.refresh()
    print(json.dumps({'document': app.activeDocument.name, 'opened': F3D}))
