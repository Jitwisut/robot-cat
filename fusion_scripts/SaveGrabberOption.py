import adsk.core, adsk.fusion, json, os

PATH='/Users/jitwisutthobut/Desktop/robot/exports/robot2_wedge_lifter_grabber_option.f3d'

def run(_context):
    app=adsk.core.Application.get();d=adsk.fusion.Design.cast(app.activeProduct)
    r=d.rootComponent
    groups={o.component.name:o for o in r.occurrences}
    if groups['03_Front_Spinner'].isVisible or not groups['07_Wedge_Lifter_Option'].isVisible or not groups['08_Grabber_Addon_Option'].isVisible:
        raise RuntimeError('Grabber option not correctly shown')
    for o in r.occurrences:
        m=o.transform2
        if abs(m.getCell(1,2)-1)>0.001 or abs(m.getCell(2,1)+1)>0.001:
            raise RuntimeError('Not grid aligned: '+o.component.name)
    app.activeDocument.save('Optional grabber jaw over modular wedge lifter')
    if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(PATH)):
        raise RuntimeError('F3D export failed')
    print(json.dumps({'saved':not app.activeDocument.isModified,'path':PATH,'bytes':os.path.getsize(PATH)}))
