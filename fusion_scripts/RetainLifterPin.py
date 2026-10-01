"""6 mm steel hinge pin with two shaft grooves and C-clip retainers."""
import adsk.core,adsk.fusion,json
def mm(x):return x/10
def p(s,x,y,z):return s.modelToSketchSpace(adsk.core.Point3D.create(mm(x),mm(y),mm(z)))
def get(r,n):return next(o for o in r.allOccurrences if o.component.name==n)
def ring_profile(s):return max((s.profiles.item(i) for i in range(s.profiles.count)),key=lambda q:(q.profileLoops.count,q.areaProperties().area))
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent;pin=get(r,'Lifter_Pivot_Pin_6mm');c=pin.component
 if any(s.name=='Pin_Clip_Groove_L' for s in c.sketches):raise RuntimeError('already applied')
 steel=get(r,'Spinner_Shaft').component.bRepBodies.item(0).material
 for side,x0,x1 in [('L',-31.4,-30.6),('R',30.6,31.4)]:
  s=c.sketches.add(c.yZConstructionPlane);s.name='Pin_Clip_Groove_'+side
  for rad in [3.05,2.5]:s.sketchCurves.sketchCircles.addByCenterRadius(p(s,0,68,31.5),mm(rad))
  e=c.features.extrudeFeatures.createInput(ring_profile(s),adsk.fusion.FeatureOperations.CutFeatureOperation)
  e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(x0)))
  e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(x1-x0)))
  e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);s.isVisible=False
 c.bRepBodies.item(0).material=steel
 g=get(r,'07_Wedge_Lifter_Option').component
 for side,x0,x1 in [('L',-31.4,-30.6),('R',30.6,31.4)]:
  o=g.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name='Lifter_Pin_C_Clip_'+side;cc=o.component
  s=cc.sketches.add(cc.yZConstructionPlane);s.name='C_Clip_Section_'+side
  for rad in [5,2.5]:s.sketchCurves.sketchCircles.addByCenterRadius(p(s,0,68,31.5),mm(rad))
  e=cc.features.extrudeFeatures.createInput(ring_profile(s),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
  e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(x0)))
  e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(x1-x0)))
  b=cc.features.extrudeFeatures.add(e).bodies.item(0);b.name=o.component.name;b.material=steel;s.isVisible=False
  # A radial opening makes it a removable clip rather than a closed washer.
  cut=cc.sketches.add(cc.yZConstructionPlane);cut.name='C_Clip_Opening_'+side
  cut.sketchCurves.sketchLines.addTwoPointRectangle(p(cut,0,70.2,30.4),p(cut,0,73.2,32.6))
  ci=cc.features.extrudeFeatures.createInput(cut.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
  ci.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(x0-.1)))
  ci.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(x1-x0+.2)))
  ci.participantBodies=[b];cc.features.extrudeFeatures.add(ci);cut.isVisible=False
 print(json.dumps({'pin_material':c.bRepBodies.item(0).material.name,'pin_mass_g':round(c.bRepBodies.item(0).physicalProperties.mass*1000,2),
                   'grooves_x_mm':[[-31.4,-30.6],[30.6,31.4]],'clips':2}))
