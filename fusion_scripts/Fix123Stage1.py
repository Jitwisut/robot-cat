"""Repair wedge/lifter load path in the live Z-up robot2, millimetre dimensions."""
import adsk.core, adsk.fusion, json

def mm(v): return v/10.0
def sp(sk,x,y,z): return sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x),mm(y),mm(z)))
def child(root,name): return next(o for o in root.allOccurrences if o.component.name==name)
def rect_join(occ,name,x0,x1,y0,y1,z0,z1):
 c=occ.component;sk=c.sketches.add(c.xYConstructionPlane);sk.name=name
 sk.sketchCurves.sketchLines.addTwoPointRectangle(sp(sk,x0,y0,0),sp(sk,x1,y1,0))
 e=c.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.JoinFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(z0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(z1-z0)))
 e.participantBodies=[c.bRepBodies.item(0)]
 c.features.extrudeFeatures.add(e);sk.isVisible=False
def zholes(occ,name,centers,r,z0,z1):
 c=occ.component
 sk=next((s for s in c.sketches if s.name==name),None)
 if sk is None:
  sk=c.sketches.add(c.xYConstructionPlane);sk.name=name
  for x,y in centers:sk.sketchCurves.sketchCircles.addByCenterRadius(sp(sk,x,y,0),mm(r))
 profiles=adsk.core.ObjectCollection.create()
 for p in sk.profiles:profiles.add(p)
 e=c.features.extrudeFeatures.createInput(profiles,adsk.fusion.FeatureOperations.CutFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(z0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(z1-z0)))
 e.participantBodies=[c.bRepBodies.item(0)];f=c.features.extrudeFeatures.add(e);f.name=name+'_Cut';sk.isVisible=False
def trim_pin(occ):
 c=occ.component
 for n,x0,x1 in [('L',-36.1,-34.8),('R',34.8,36.1)]:
  sk=c.sketches.add(c.yZConstructionPlane);sk.name='Pin_End_Trim_'+n
  sk.sketchCurves.sketchCircles.addByCenterRadius(sp(sk,0,68,31.5),mm(3.5))
  e=c.features.extrudeFeatures.createInput(sk.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
  e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(x0)))
  e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(x1-x0)))
  e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);sk.isVisible=False
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent
 if any(abs(o.transform2.getCell(i,j)-(1 if i==j else 0))>1e-3 for o in r.occurrences for i in range(3) for j in range(4)):
  raise RuntimeError('root groups not grid aligned')
 g=child(r,'07_Wedge_Lifter_Option');pin=child(r,'Lifter_Pivot_Pin_6mm')
 if not any(s.name=='Pin_End_Trim_L' for s in pin.component.sketches):trim_pin(pin)
 base=child(r,'Base_Plate_Al')
 result={'pin_x_mm':[-34.8,34.8],'support_fasteners':{}}
 for side in ['L','R']:
  support=child(r,'Lifter_Pivot_Support_'+side)
  if side=='L': x0,x1,xc=-35,-25,-29.5
  else:x0,x1,xc=25,35,29.5
  if not any(s.name=='Load_Path_To_Base_'+side for s in support.component.sketches):
   rect_join(support,'Load_Path_To_Base_'+side,x0,x1,61,76,5,25)
  holes=[(xc,64.5),(xc,72.5)]
  if not any(f.name=='Lifter_Support_Base_M3_Clear_'+side+'_Cut' for f in base.component.features):
   zholes(base,'Lifter_Support_Base_M3_Clear_'+side,holes,1.7,2.9,5.1)
  if not any(f.name=='Lifter_Support_M3_Tap_'+side+'_Cut' for f in support.component.features):
   zholes(support,'Lifter_Support_M3_Tap_'+side,holes,1.25,4.9,13.0)
  result['support_fasteners'][side]=holes
 print(json.dumps(result))
