import adsk.core,adsk.fusion,json
def mm(x):return x/10
def pt(s,x,y,z):return s.modelToSketchSpace(adsk.core.Point3D.create(mm(x),mm(y),mm(z)))
def get(r,n):return next(o for o in r.allOccurrences if o.component.name==n)
def cut_box(o,n,x0,x1,y0,y1,z0,z1):
 c=o.component;s=c.sketches.add(c.xYConstructionPlane);s.name=n
 s.sketchCurves.sketchLines.addTwoPointRectangle(pt(s,x0,y0,0),pt(s,x1,y1,0))
 e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(z0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(z1-z0)))
 e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);s.isVisible=False
def xhole(o,n,y,z,r):
 c=o.component;s=c.sketches.add(c.yZConstructionPlane);s.name=n
 s.sketchCurves.sketchCircles.addByCenterRadius(pt(s,0,y,z),mm(r))
 e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
 e.setSymmetricExtent(adsk.core.ValueInput.createByReal(mm(100)),True)
 e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);s.isVisible=False
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent
 for side in ['L','R']:
  o=get(r,'Lifter_Pivot_Support_'+side);n='Flange_Clearance_'+side
  if any(s.name==n for s in o.component.sketches):continue
  if side=='L':x0,x1=-32,-24.9
  else:x0,x1=24.9,32
  cut_box(o,n,x0,x1,60.9,76.1,14,25.1)
 b=get(r,'Lifter_Hinge_Boss')
 if not any(s.name=='Lever_Repair_Pivot_Bore' for s in b.component.sketches):
  xhole(b,'Lever_Repair_Pivot_Bore',68,31.5,3.2)
 print(json.dumps({'support_flange_top_mm':14,'hinge_bore_mm':6.4}))
