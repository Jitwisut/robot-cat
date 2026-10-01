import adsk.core, adsk.fusion, json, os


EXPORT_DIR = '/Users/jitwisutthobut/Desktop/robot/exports'


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design or app.activeDocument.name != 'robot2':
        raise RuntimeError('robot2 must be active')
    root = design.rootComponent
    names = [o.component.name for o in root.allOccurrences]
    if names.count('Beater_Rotor_Concept') != 1 or names.count('Spinner_Bar'):
        raise RuntimeError('Beater variant not in expected state')

    app.activeDocument.save('Beater rotor concept: 34 mm wide within existing spinner bay')
    manager = design.exportManager
    f3d = os.path.join(EXPORT_DIR, 'robot2_beater_concept_v1.f3d')
    f3d_ok = manager.execute(manager.createFusionArchiveExportOptions(f3d))
    if not f3d_ok:
        raise RuntimeError('F3D export failed')
    print(json.dumps({'saved': not app.activeDocument.isModified,
                      'f3d': f3d,
                      'f3d_bytes': os.path.getsize(f3d)}))
