import adsk.core, adsk.fusion, json

def box(b):
    p=b.boundingBox
    return [[round(getattr(p.minPoint,a)*10,1) for a in 'xyz'],
            [round(getattr(p.maxPoint,a)*10,1) for a in 'xyz']]

def run(_context):
    app=adsk.core.Application.get()
    design=adsk.fusion.Design.cast(app.activeProduct)
    root=design.rootComponent
    rows=[]
    for g in root.occurrences:
        group=g.component.name
        if group in ('01_Chassis','03_Front_Spinner','05_Electronics'):
            for occ in g.childOccurrences:
                for body in occ.bRepBodies:
                    rows.append({'group':group,'part':occ.component.name,'body':body.name,
                                 'box':box(body),'visible':occ.isVisible})
    print(json.dumps({'document':app.activeDocument.name,'parts':rows}))
