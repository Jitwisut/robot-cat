"""Positive two-bar servo restraint bolted to base posts."""
import adsk.core,adsk.fusion,json
def mm(v):return v/10
def pt(s,x,y,z):return s.modelToSketchSpace(adsk.core.Point3D.create(mm(x),mm(y),mm(z)))
def get(r,n):return next(o for o in r.allOccurrences if o.component.name==n)
def box(p,n,x0,x1,y0,y1,z0,z1,mat):
 o=p.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=n;c=o.component
 s=c.sketches.add(c.xYConstructionPlane);s.name=n+'_Outline'
 s.sketchCurves.sketchLines.addTwoPointRectangle(pt(s,x0,y0,0),pt(s,x1,y1,0))
 e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(z0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(z1-z0)))
 b=c.features.extrudeFeatures.add(e).bodies.item(0);b.name=n;b.material=mat;s.isVisible=False;return o
def cut(o,n,centers,r,z0,z1):
 c=o.component;s=c.sketches.add(c.xYConstructionPlane);s.name=n
 for x,y in centers:s.sketchCurves.sketchCircles.addByCenterRadius(pt(s,x,y,0),mm(r))
 ps=adsk.core.ObjectCollection.create()
 for p in s.profiles:ps.add(p)
 e=c.features.extrudeFeatures.createInput(ps,adsk.fusion.FeatureOperations.CutFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(z0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(z1-z0)))
 e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);s.isVisible=False
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent;g=get(r,'07_Wedge_Lifter_Option').component
 if any(o.component.name=='Lifter_Servo_Clamp_Bar_L' for o in g.occurrences):raise RuntimeError('already applied')
 mat=get(r,'Base_Plate_Al').component.bRepBodies.item(0).material
 all_holes=[]
 for side,x in [('L',-12),('R',12)]:
  bar=box(g,'Lifter_Servo_Clamp_Bar_'+side,x-3,x+3,-15,60,32,34,mat)
  cut(bar,'Clamp_Bar_M3_Clear_'+side,[(x,-15),(x,60)],1.7,31.9,34.1)
  for pos,y in [('Rear',-15),('Front',60)]:
   post=box(g,'Lifter_Servo_Clamp_Post_'+side+'_'+pos,x-3,x+3,y-3,y+3,5,32,mat)
   cut(post,'Clamp_Post_M3_Tap_'+side+'_'+pos,[(x,y)],1.25,4.9,32.1)
   all_holes.append((x,y))
 cut(get(r,'Base_Plate_Al'),'Lifter_Servo_Clamp_Base_M3_Clear',all_holes,1.7,2.9,5.1)
 print(json.dumps({'clamp_bars':2,'posts':4,'M3_base_holes':all_holes}))
