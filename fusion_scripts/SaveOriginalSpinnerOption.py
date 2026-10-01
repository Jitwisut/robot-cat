import adsk.core, adsk.fusion, json, os

PATH='/Users/jitwisutthobut/Desktop/robot/exports/robot2_vertical_spinner_option.f3d'

def run(_context):
    app=adsk.core.Application.get()
    d=adsk.fusion.Design.cast(app.activeProduct)
    r=d.rootComponent
    names=[o.component.name for o in r.allOccurrences]
    if names.count('Spinner_Bar')!=1 or names.count('Beater_Rotor_Concept'):
        raise RuntimeError('Original spinner not present')
    for o in r.occurrences:
        m=o.transform2
        if abs(m.getCell(1,2)-1)>0.001 or abs(m.getCell(2,1)+1)>0.001:
            raise RuntimeError('Root group not grid aligned: '+o.component.name)
    app.activeDocument.save('Restore original vertical spinner as modular weapon option')
    if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(PATH)):
        raise RuntimeError('F3D export failed')
    print(json.dumps({'saved':not app.activeDocument.isModified,'export':PATH,'bytes':os.path.getsize(PATH),
                      'spinner_count':names.count('Spinner_Bar')}))
