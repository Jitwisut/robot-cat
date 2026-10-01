"""Set all root occurrences to identity to inspect Fusion's visible floor orientation."""
import adsk.core, adsk.fusion, json

def run(_context):
    app=adsk.core.Application.get()
    d=adsk.fusion.Design.cast(app.activeProduct)
    if app.activeDocument.name!='robot2':raise RuntimeError('robot2 must be active')
    r=d.rootComponent
    occs=list(r.occurrences)
    for o in occs:
        m=o.transform2
        if abs(m.getCell(1,2)-1)>0.001 or abs(m.getCell(2,1)+1)>0.001:
            raise RuntimeError('Unexpected orientation '+o.component.name)
    identity=adsk.core.Matrix3D.create()
    ok=r.transformOccurrences(occs,[identity for _ in occs],True)
    if not ok:raise RuntimeError('Could not reset root occurrences')
    out=[]
    for o in r.occurrences:
        m=o.transform2
        out.append({'group':o.component.name,'y_y':round(m.getCell(1,1),3),'z_z':round(m.getCell(2,2),3)})
    print(json.dumps({'changed_to_identity':ok,'groups':out}))
