import adsk.core, adsk.fusion, json

def box(b):
    q=b.boundingBox
    return [[round(getattr(q.minPoint,a)*10,2) for a in 'xyz'],[round(getattr(q.maxPoint,a)*10,2) for a in 'xyz']]

def run(_):
    app=adsk.core.Application.get(); d=adsk.fusion.Design.cast(app.activeProduct)
    if not d or app.activeDocument.name!='robot2': raise RuntimeError('robot2 must be active')
    root=d.rootComponent
    groups=[]
    for o in root.occurrences:
        children=[]
        for c in o.component.occurrences:
            bodies=[]
            for b in c.component.bRepBodies:
                bodies.append({'name':b.name,'box':box(b),'mass_g':round(b.physicalProperties.mass*1000,2),'sketches':[c.component.sketches.item(i).name for i in range(c.component.sketches.count)],'features':[c.component.features.item(i).name for i in range(c.component.features.count)]})
            children.append({'name':c.component.name,'bodies':bodies})
        groups.append({'name':o.component.name,'children':children})
    print(json.dumps({'document':app.activeDocument.name,'groups':groups},separators=(',',':')))
