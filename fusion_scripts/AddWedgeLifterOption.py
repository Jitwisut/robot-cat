"""Add a swappable front wedge/lifter layout to robot2. Dimensions are mm."""
import adsk.core, adsk.fusion, json

def cm(x): return x/10.0
def pt(sk,x,y,z):
    return sk.modelToSketchSpace(adsk.core.Point3D.create(cm(x),cm(y),cm(z)))

def new_part(parent,name):
    occ=parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name=name
    return occ

def box(parent,name,x0,x1,y0,y1,z0,z1,material=None):
    occ=new_part(parent,name); c=occ.component
    sk=c.sketches.add(c.xYConstructionPlane); sk.name=name+'_Outline'
    sk.sketchCurves.sketchLines.addTwoPointRectangle(pt(sk,x0,y0,0),pt(sk,x1,y1,0))
    e=c.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(z0)))
    e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(cm(z1-z0)))
    body=c.features.extrudeFeatures.add(e).bodies.item(0);body.name=name
    if material: body.material=material
    sk.isVisible=False
    return occ,body

def x_cylinder(parent,name,cy,cz,r,x0,x1,material=None):
    occ=new_part(parent,name); c=occ.component
    sk=c.sketches.add(c.yZConstructionPlane);sk.name=name+'_Section'
    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(sk,0,cy,cz),cm(r))
    e=c.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(x0)))
    e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(cm(x1-x0)))
    body=c.features.extrudeFeatures.add(e).bodies.item(0);body.name=name
    if material:body.material=material
    sk.isVisible=False
    return occ,body

def x_bore(occ,name,cy,cz,r):
    c=occ.component
    sk=c.sketches.add(c.yZConstructionPlane);sk.name=name
    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(sk,0,cy,cz),cm(r))
    e=c.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
    e.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(100)),True)
    e.participantBodies=[c.bRepBodies.item(0)]
    c.features.extrudeFeatures.add(e)
    sk.isVisible=False

def poly_plate(parent,name,profile,x0,x1,material):
    occ=new_part(parent,name);c=occ.component
    sk=c.sketches.add(c.yZConstructionPlane);sk.name=name+'_Side_Profile'
    lines=sk.sketchCurves.sketchLines
    for i in range(len(profile)):
        a=profile[i];b=profile[(i+1)%len(profile)]
        lines.addByTwoPoints(pt(sk,0,*a),pt(sk,0,*b))
    if sk.profiles.count!=1:raise RuntimeError('spatula profile invalid')
    e=c.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(x0)))
    e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(cm(x1-x0)))
    body=c.features.extrudeFeatures.add(e).bodies.item(0);body.name=name;body.material=material
    sk.isVisible=False
    return occ,body

def bbox(b):
    q=b.boundingBox
    return [[round(getattr(q.minPoint,a)*10,1) for a in 'xyz'],
            [round(getattr(q.maxPoint,a)*10,1) for a in 'xyz']]

def run(_context):
    app=adsk.core.Application.get()
    d=adsk.fusion.Design.cast(app.activeProduct)
    if app.activeDocument.name!='robot2':raise RuntimeError('robot2 must be active')
    root=d.rootComponent
    if any(o.component.name=='07_Wedge_Lifter_Option' for o in root.occurrences):
        raise RuntimeError('lifter option already exists')
    base=next(b for o in root.allOccurrences for b in o.bRepBodies if b.name=='Base_Plate_Al')
    metal=base.material
    group=new_part(root,'07_Wedge_Lifter_Option');g=group.component

    # Native axes: X across robot, Y toward the nose, Z above the floor.
    # Central 62 mm spatula sits between the two existing front wedges (X=+-35).
    blade_o,blade=poly_plate(g,'Lifter_Spatula_62mm',
        [(70,27),(112,5.8),(112,8.5),(72,30)],-31,31,metal)
    hinge_o,hinge=box(g,'Lifter_Hinge_Boss',-30,30,64,74,27,36,metal)
    x_bore(hinge_o,'Hinge_Clearance_Bore_6p4mm',68,31.5,3.2)
    left_o,left=box(g,'Lifter_Pivot_Support_L',-35,-32,61,76,24,41,metal)
    right_o,right=box(g,'Lifter_Pivot_Support_R',32,35,61,76,24,41,metal)
    x_bore(left_o,'Left_Pivot_Bore_6p4mm',68,31.5,3.2)
    x_bore(right_o,'Right_Pivot_Bore_6p4mm',68,31.5,3.2)
    pin_o,pin=x_cylinder(g,'Lifter_Pivot_Pin_6mm',68,31.5,3,-36,36,metal)

    # Actuator blocks are packaging proxies. Exact purchased servo and linkage
    # must be selected before drilling the chassis and calculating true mass.
    servo_o,servo=box(g,'Servo_30kgcm_Envelope_Only',-20,20,17,57,9,39)
    horn_o,horn=box(g,'Lifter_Servo_Horn_Envelope_Only',-4,4,48,63,37,40)
    link_o,link=box(g,'Lifter_Linkage_Envelope_Only',-3,3,61,72,34,38)
    group.isLightBulbOn=False
    for o in root.allOccurrences:
        if o.component.name.endswith('Sweep_Envelope'):o.isLightBulbOn=False
    print(json.dumps({'group':group.component.name,'visible':group.isVisible,
                      'spatula_native_mm':bbox(blade),'pivot_native_mm':bbox(pin),
                      'servo_native_mm':bbox(servo),'blade_mass_cad_g':round(blade.physicalProperties.mass*1000,2),
                      'note':'servo, horn and linkage are provisional envelopes; no motion joint yet'}))
