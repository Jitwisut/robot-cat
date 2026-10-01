import adsk.core, adsk.fusion, json, math

def run(_context):
    d=adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    r=d.rootComponent
    o=list(r.occurrences)
    m=adsk.core.Matrix3D.create()
    m.setToRotation(-math.pi/2,adsk.core.Vector3D.create(1,0,0),adsk.core.Point3D.create(0,0,0))
    before=[[round(o[0].transform2.getCell(i,j),3) for j in range(4)] for i in range(3)]
    if abs(o[0].transform2.getCell(1,2)-1)<0.001 and abs(o[0].transform2.getCell(2,1)+1)<0.001:
        ok=True;status='already_aligned'
    else:
        if abs(o[0].transform2.getCell(1,1)-1)>0.001:
            raise RuntimeError('Unexpected root orientation')
        ok=r.transformOccurrences(o,[m for _ in o],True)
        status='aligned'
    after=[[round(o[0].transform2.getCell(i,j),3) for j in range(4)] for i in range(3)]
    print(json.dumps({'ok':ok,'status':status,'before':before,'after':after,'timeline':d.timeline.count}))
