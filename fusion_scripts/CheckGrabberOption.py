import adsk.core, adsk.fusion, json

def run(_context):
    d=adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    r=d.rootComponent
    grab=next(o for o in r.occurrences if o.component.name=='08_Grabber_Addon_Option')
    names={b.name for o in grab.childOccurrences for b in o.bRepBodies}
    oc=adsk.core.ObjectCollection.create()
    for top in r.occurrences:
        if top.component.name=='03_Front_Spinner':continue
        for o in top.childOccurrences:
            if o.component.name=='Holder_ESC_Tekko32':continue
            for b in o.bRepBodies:
                if not b.name.endswith(('Envelope','Envelope_Only')):oc.add(b)
    inp=d.createInterferenceInput(oc);inp.areCoincidentFacesIncluded=False
    res=d.analyzeInterference(inp)
    pairs=[]
    tbm=adsk.fusion.TemporaryBRepManager.get()
    for i in range(res.count):
        a=res.item(i).entityOne;b=res.item(i).entityTwo
        if a.name not in names and b.name not in names:continue
        tmp=tbm.copy(a);tbm.booleanOperation(tmp,tbm.copy(b),adsk.fusion.BooleanTypes.IntersectionBooleanType)
        pairs.append({'a':a.name,'b':b.name,'volume_mm3':round(tmp.volume*1000,2),
                      'external':(a.name in names)!=(b.name in names)})
    print(json.dumps({'grabber_visible':grab.isVisible,'grabber_interference':pairs}))
