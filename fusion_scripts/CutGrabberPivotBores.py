import adsk.core, adsk.fusion, json

def run(_context):
    app=adsk.core.Application.get();d=adsk.fusion.Design.cast(app.activeProduct)
    r=d.rootComponent
    g=next(o for o in r.occurrences if o.component.name=='08_Grabber_Addon_Option')
    names={'Grabber_Arm_L','Grabber_Arm_R','Grabber_Pivot_Bracket_L','Grabber_Pivot_Bracket_R'}
    done=[]
    for o in g.childOccurrences:
        if o.component.name not in names:continue
        c=o.component
        sk=c.sketches.add(c.yZConstructionPlane);sk.name=c.name+'_6p4mm_Pivot_Clearance'
        p=sk.modelToSketchSpace(adsk.core.Point3D.create(0,6.8,4.9))
        sk.sketchCurves.sketchCircles.addByCenterRadius(p,0.32)
        e=c.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
        e.setSymmetricExtent(adsk.core.ValueInput.createByReal(10.0),True)
        e.participantBodies=[c.bRepBodies.item(0)]
        c.features.extrudeFeatures.add(e);sk.isVisible=False
        done.append(c.name)
    if len(done)!=4:raise RuntimeError('Expected four pivot bores, got '+str(done))
    print(json.dumps({'pivot_bores':done,'diameter_mm':6.4}))
