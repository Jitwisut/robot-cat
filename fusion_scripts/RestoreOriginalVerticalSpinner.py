"""Restore the 6 mm axial vertical bar option in robot2's existing spinner bay."""
import adsk.core, adsk.fusion, json, os
from math import pi

EXPORT='/Users/jitwisutthobut/Desktop/robot/exports/robot2_vertical_spinner_option.f3d'

def mm(v): return v/10.0
def pt(sk,x,y,z): return sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x),mm(y),mm(z)))

def bbox(body):
    b=body.boundingBox
    return [[round(getattr(b.minPoint,a)*10,2) for a in 'xyz'],
            [round(getattr(b.maxPoint,a)*10,2) for a in 'xyz']]

def run(_context):
    app=adsk.core.Application.get()
    design=adsk.fusion.Design.cast(app.activeProduct)
    if app.activeDocument.name!='robot2': raise RuntimeError('robot2 must be active')
    root=design.rootComponent
    spin=next(o for o in root.occurrences if o.component.name=='03_Front_Spinner')
    sc=spin.component
    beater=next(o for o in sc.occurrences if o.component.name=='Beater_Rotor_Concept')
    material=beater.component.bRepBodies.item(0).material
    if any(o.component.name=='Spinner_Bar' for o in sc.occurrences):
        raise RuntimeError('Spinner_Bar already exists')
    for o in list(sc.occurrences):
        if o.component.name in ('Beater_Rotor_Concept','Beater_Sweep_Envelope'):
            o.deleteMe()

    occ=sc.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp=occ.component; comp.name='Spinner_Bar'
    sk=comp.sketches.add(comp.xYConstructionPlane)
    sk.name='Original_Vertical_Bar_6x65'
    sk.sketchCurves.sketchLines.addTwoPointRectangle(pt(sk,-3,42.5,0),pt(sk,3,107.5,0))
    e=comp.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(28.5)))
    e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(20)))
    body=comp.features.extrudeFeatures.add(e).bodies.item(0)
    body.name='Spinner_Bar'
    bore=comp.sketches.add(comp.yZConstructionPlane)
    bore.name='Existing_Shaft_4mm_Bore_Provisional'
    bore.sketchCurves.sketchCircles.addByCenterRadius(pt(bore,0,75,38.5),mm(2))
    cut=comp.features.extrudeFeatures.createInput(bore.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.setSymmetricExtent(adsk.core.ValueInput.createByReal(mm(10)),True)
    cut.participantBodies=[body]
    comp.features.extrudeFeatures.add(cut)
    body.material=material

    sweep=sc.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    sweep.component.name='Spinner_Sweep_Envelope'
    ssk=sweep.component.sketches.add(sweep.component.yZConstructionPlane)
    ssk.name='Original_Bar_Rotation_Radius_32p5'
    ssk.sketchCurves.sketchCircles.addByCenterRadius(pt(ssk,0,75,38.5),mm(32.5))
    se=sweep.component.features.extrudeFeatures.createInput(ssk.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    se.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(-3)))
    se.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(6)))
    sb=sweep.component.features.extrudeFeatures.add(se).bodies.item(0)
    sb.name='Spinner_Sweep_Envelope'
    sweep.isLightBulbOn=False

    for o in root.allOccurrences:
        if o.component.name.endswith('Sweep_Envelope'): o.isLightBulbOn=False
    print(json.dumps({'spinner_body_mm':bbox(body),'mass_cad_g':round(body.physicalProperties.mass*1000,2),
                      'stage':'geometry created; align and save in next command'}))
