import adsk.core, adsk.fusion, json, os


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design or app.activeDocument.name != 'robot2':
        raise RuntimeError('robot2 must be active')
    rotor = [o for o in design.rootComponent.allOccurrences
             if o.component.name == 'Beater_Rotor_Concept']
    if len(rotor) != 1:
        raise RuntimeError('Expected one beater rotor')
    path = '/Users/jitwisutthobut/Desktop/robot/exports/Beater_Rotor_Concept.step'
    manager = design.exportManager
    options = manager.createSTEPExportOptions(path, rotor[0].component)
    ok = manager.execute(options)
    if not ok:
        raise RuntimeError('STEP export returned false')
    print(json.dumps({'path': path, 'bytes': os.path.getsize(path),
                      'scope': 'beater rotor concept only'}))
