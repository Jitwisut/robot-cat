import adsk.core, adsk.fusion, json

def box(b):
    q=b.boundingBox
    return [[round(getattr(q.minPoint,a)*10,1) for a in 'xyz'],
            [round(getattr(q.maxPoint,a)*10,1) for a in 'xyz']]

def run(_context):
    d=adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    r=d.rootComponent
    out=[]
    for o in r.occurrences:
        m=o.transform2
        out.append({'name':o.component.name,'matrix':[[round(m.getCell(i,j),3) for j in range(4)] for i in range(3)]})
    b=next(b for o in r.allOccurrences for b in o.bRepBodies if b.name=='Base_Plate_Al')
    print(json.dumps({'base':box(b),'groups':out,'timelineCount':d.timeline.count}))
