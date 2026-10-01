import adsk.core,adsk.fusion,json
def mm(x):return x/10
def p(s,x,y,z):return s.modelToSketchSpace(adsk.core.Point3D.create(mm(x),mm(y),mm(z)))
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent
 for side in ['L','R']:
  o=next(q for q in r.allOccurrences if q.component.name=='BBB_Motor_Cradle_'+side);c=o.component
  n='Battery_Tray_Rear_Clearance_'+side
  if any(s.name==n for s in c.sketches):continue
  x0,x1=(-58.1,-47.9) if side=='L' else (47.9,58.1)
  s=c.sketches.add(c.xYConstructionPlane);s.name=n
  s.sketchCurves.sketchLines.addTwoPointRectangle(p(s,x0,-20.1,0),p(s,x1,-18.0,0))
  e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
  e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(30)))
  e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(7.1)))
  e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);s.isVisible=False
 print(json.dumps({'trim':'rear upper corner of both new motor cradles','battery_holder_gap_mm':0.5}))
