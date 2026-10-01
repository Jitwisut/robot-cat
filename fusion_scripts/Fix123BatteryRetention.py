"""Two positive battery clamp bars fastened to aluminium side plates."""
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
def xcut(o,n,y,z,r,x0,x1):
 c=o.component;s=c.sketches.add(c.yZConstructionPlane);s.name=n
 s.sketchCurves.sketchCircles.addByCenterRadius(pt(s,0,y,z),mm(r))
 e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(x0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(x1-x0)))
 e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);s.isVisible=False
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent
 if any(o.component.name=='10_Battery_Restraints' for o in r.occurrences):raise RuntimeError('already applied')
 metal=get(r,'Base_Plate_Al').component.bRepBodies.item(0).material
 g=r.occurrences.addNewComponent(adsk.core.Matrix3D.create());g.component.name='10_Battery_Restraints'
 masses=[]
 for idx,y in enumerate([-48,-28],1):
  bar=box(g.component,'Battery_Clamp_Bar_'+str(idx),-78,78,y-3,y+3,61.6,67.6,metal)
  xcut(bar,'Clamp_M3_Tap_L_'+str(idx),y,64.6,1.25,-78.1,-67.9)
  xcut(bar,'Clamp_M3_Tap_R_'+str(idx),y,64.6,1.25,67.9,78.1)
  xcut(get(r,'Left_Side_Plate_Al'),'Clamp_M3_Clear_L_'+str(idx),y,64.6,1.7,-80.1,-77.9)
  xcut(get(r,'Right_Side_Plate_Al'),'Clamp_M3_Clear_R_'+str(idx),y,64.6,1.7,77.9,80.1)
  masses.append(round(bar.bRepBodies.item(0).physicalProperties.mass*1000,2))
 print(json.dumps({'bars_y_mm':[-48,-28],'bar_mass_cad_g':masses,'fasteners':'four M3 from side plate into clamp bars'}))
