import adsk.core, adsk.fusion, json, os

PATH='/Users/jitwisutthobut/Desktop/robot/exports/robot2_wedge_lifter_option.f3d'

def run(_context):
    app=adsk.core.Application.get();d=adsk.fusion.Design.cast(app.activeProduct)
    if app.activeDocument.name!='robot2':raise RuntimeError('robot2 must be active')
    r=d.rootComponent
    spin=next(o for o in r.occurrences if o.component.name=='03_Front_Spinner')
    lift=next(o for o in r.occurrences if o.component.name=='07_Wedge_Lifter_Option')
    grab=next((o for o in r.occurrences if o.component.name=='08_Grabber_Addon_Option'),None)
    if spin.isVisible or not lift.isVisible:raise RuntimeError('lifter option visibility is wrong')
    if grab and grab.isVisible:raise RuntimeError('grabber should be hidden in basic lifter option')
    for o in r.occurrences:
        m=o.transform2
        if abs(m.getCell(1,2)-1)>0.001 or abs(m.getCell(2,1)+1)>0.001:
            raise RuntimeError('Root group not grid aligned: '+o.component.name)
    app.activeDocument.save('Add modular wedge lifter option; show lifter configuration')
    if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(PATH)):
        raise RuntimeError('F3D export failed')
    print(json.dumps({'saved':not app.activeDocument.isModified,'export':PATH,'bytes':os.path.getsize(PATH),
                      'spinner_visible':spin.isVisible,'lifter_visible':lift.isVisible,
                      'grabber_visible':grab.isVisible if grab else None}))
