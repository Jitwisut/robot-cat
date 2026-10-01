"""Add a hidden grabber jaw packaging concept over the wedge lifter."""
import adsk.core, adsk.fusion, json

def cm(v):return v/10.0
def pt(sk,x,y,z):return sk.modelToSketchSpace(adsk.core.Point3D.create(cm(x),cm(y),cm(z)))
def part(parent,name):
    o=parent.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=name
    return o
def box(parent,name,x0,x1,y0,y1,z0,z1,material=None):
    o=part(parent,name);c=o.component
    sk=c.sketches.add(c.xYConstructionPlane);sk.name=name+'_Outline'
    sk.sketchCurves.sketchLines.addTwoPointRectangle(pt(sk,x0,y0,0),pt(sk,x1,y1,0))
    e=c.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(z0)))
    e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(cm(z1-z0)))
    b=c.features.extrudeFeatures.add(e).bodies.item(0);b.name=name
    if material:b.material=material
    sk.isVisible=False
    return o,b
def pivot(parent,name,y,z,r,x0,x1,material):
    o=part(parent,name);c=o.component
    sk=c.sketches.add(c.yZConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(sk,0,y,z),cm(r))
    e=c.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(x0)))
    e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(cm(x1-x0)))
    b=c.features.extrudeFeatures.add(e).bodies.item(0);b.name=name;b.material=material
    sk.isVisible=False
    return o,b
def bore(o,name):
    c=o.component;sk=c.sketches.add(c.yZConstructionPlane);sk.name=name
    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(sk,0,68,49),cm(3.2))
    e=c.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
    e.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(100)),True)
    e.participantBodies=[c.bRepBodies.item(0)]
    c.features.extrudeFeatures.add(e);sk.isVisible=False

def run(_context):
    app=adsk.core.Application.get();d=adsk.fusion.Design.cast(app.activeProduct)
    if app.activeDocument.name!='robot2':raise RuntimeError('robot2 must be active')
    r=d.rootComponent
    if any(o.component.name=='08_Grabber_Addon_Option' for o in r.occurrences):
        raise RuntimeError('grabber already exists')
    if not any(o.component.name=='07_Wedge_Lifter_Option' for o in r.occurrences):
        raise RuntimeError('wedge lifter missing')
    material=next(b.material for o in r.allOccurrences for b in o.bRepBodies if b.name=='Base_Plate_Al')
    grp=part(r,'08_Grabber_Addon_Option');g=grp.component
    # Jaw rests above the spatula. It closes about an X-axis pivot at Y=68,Z=49.
    cross,cb=box(g,'Grabber_Upper_Jaw',-30,30,101,109,44,49,material)
    left,lb=box(g,'Grabber_Arm_L',-30,-26,68,105,45,51,material)
    right,rb=box(g,'Grabber_Arm_R',26,30,68,105,45,51,material)
    pin,pb=pivot(g,'Grabber_Pivot_Pin_6mm',68,49,3,-35,35,material)
    sl,_=box(g,'Grabber_Pivot_Bracket_L',-35,-32,61,75,43,57,material)
    sr,_=box(g,'Grabber_Pivot_Bracket_R',32,35,61,75,43,57,material)
    for o in (left,right,sl,sr):bore(o,o.component.name+'_6p4mm_Pivot_Clearance')
    servo,sb=box(g,'Grabber_Second_Servo_Envelope_Only',-15,15,23,56,44,70)
    link,_=box(g,'Grabber_Linkage_Envelope_Only',-3,3,56,69,53,59)
    grp.isLightBulbOn=False
    print(json.dumps({'group':grp.component.name,'hidden':not grp.isVisible,
                      'jaw_cad_mass_g':round(cb.physicalProperties.mass*1000,2),
                      'arms_cad_mass_g':round((lb.physicalProperties.mass+rb.physicalProperties.mass)*1000,2),
                      'note':'optional second servo, linkage, pivots are only packaging concept; no moving joint'}))
