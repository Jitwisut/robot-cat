import adsk.core,adsk.fusion,json
def mm(x):return x/10
def pt(s,x,y,z):return s.modelToSketchSpace(adsk.core.Point3D.create(mm(x),mm(y),mm(z)))
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent
 for side,x0,x1 in [('L',-31.5,-30.5),('R',30.5,31.5)]:
  o=next(q for q in r.allOccurrences if q.component.name=='Lifter_Pin_C_Clip_'+side);c=o.component
  n='C_Clip_OD_8p0_'+side
  if any(s.name==n for s in c.sketches):continue
  s=c.sketches.add(c.yZConstructionPlane);s.name=n
  for rad in [5.1,4.0]:s.sketchCurves.sketchCircles.addByCenterRadius(pt(s,0,68,31.5),mm(rad))
  prof=max((s.profiles.item(i) for i in range(s.profiles.count)),key=lambda p:p.profileLoops.count)
  e=c.features.extrudeFeatures.createInput(prof,adsk.fusion.FeatureOperations.CutFeatureOperation)
  e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(x0)))
  e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(x1-x0)))
  e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);s.isVisible=False
 print(json.dumps({'clip_outer_diameter_mm':8.0,'purpose':'clear spatula'}))
