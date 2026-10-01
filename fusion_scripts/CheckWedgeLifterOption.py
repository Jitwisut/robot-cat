import adsk.core, adsk.fusion, json

EXCLUDE={'Spinner_Sweep_Envelope','Beater_Sweep_Envelope','PCB_Keepout_Envelope',
         'ESP32_Dupont_Keepout_A','ESP32_Dupont_Keepout_B'}

def run(_context):
    app=adsk.core.Application.get();d=adsk.fusion.Design.cast(app.activeProduct)
    r=d.rootComponent
    lift=next(o for o in r.occurrences if o.component.name=='07_Wedge_Lifter_Option')
    shapes=[]
    for top in r.occurrences:
        if top.component.name=='03_Front_Spinner':continue
        for o in top.childOccurrences:
            if not o.isVisible:continue
            for b in o.bRepBodies:
                if b.name not in EXCLUDE:shapes.append(b)
    oc=adsk.core.ObjectCollection.create()
    for b in shapes:oc.add(b)
    inp=d.createInterferenceInput(oc);inp.areCoincidentFacesIncluded=False
    results=d.analyzeInterference(inp)
    pairs=[]
    tbm=adsk.fusion.TemporaryBRepManager.get()
    for i in range(results.count):
        v=results.item(i)
        a=v.entityOne;b=v.entityTwo
        if not (a.name.startswith(('Lifter_','Servo_')) or b.name.startswith(('Lifter_','Servo_'))):
            continue
        copy=tbm.copy(a)
        tbm.booleanOperation(copy,tbm.copy(b),adsk.fusion.BooleanTypes.IntersectionBooleanType)
        pairs.append({'a':a.name,'b':b.name,'volume_mm3':round(copy.volume*1000,2)})
    parts=[]
    for o in lift.childOccurrences:
        b=o.bRepBodies.item(0)
        parts.append({'name':b.name,'cad_mass_g':round(b.physicalProperties.mass*1000,2)})
    print(json.dumps({'document':app.activeDocument.name,'lifter_visible':lift.isVisible,
                      'lifter_parts':parts,'lifter_interference':pairs}))
