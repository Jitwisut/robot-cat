"""Specified 7.4V lifter actuator and linkage in the Z-up robot2 option."""
import adsk.core, adsk.fusion, json, math
def mm(v):return v/10
def pt(sk,x,y,z):return sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x),mm(y),mm(z)))
def get(r,n):return next(o for o in r.allOccurrences if o.component.name==n)
def part(p,n):
 o=p.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=n;return o
def box(p,n,x0,x1,y0,y1,z0,z1,mat=None):
 o=part(p,n);c=o.component;s=c.sketches.add(c.xYConstructionPlane);s.name=n+'_Outline'
 s.sketchCurves.sketchLines.addTwoPointRectangle(pt(s,x0,y0,0),pt(s,x1,y1,0))
 e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(z0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(z1-z0)))
 b=c.features.extrudeFeatures.add(e).bodies.item(0);b.name=n
 if mat:b.material=mat
 s.isVisible=False;return o
def yzbar(p,n,x0,x1,a,b,width,mat):
 ay,az=a;by,bz=b;dy=by-ay;dz=bz-az;h=math.hypot(dy,dz)
 ny=-dz*width/(2*h);nz=dy*width/(2*h)
 ey=4*dy/h;ez=4*dz/h
 coords=[(ay-ey+ny,az-ez+nz),(by+ey+ny,bz+ez+nz),
         (by+ey-ny,bz+ez-nz),(ay-ey-ny,az-ez-nz)]
 o=part(p,n);c=o.component;s=c.sketches.add(c.yZConstructionPlane);s.name=n+'_Profile'
 for i in range(4):
  q=coords[i];v=coords[(i+1)%4];s.sketchCurves.sketchLines.addByTwoPoints(pt(s,0,*q),pt(s,0,*v))
 e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(x0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(x1-x0)))
 bb=c.features.extrudeFeatures.add(e).bodies.item(0);bb.name=n;bb.material=mat;s.isVisible=False
 bore_x(o,n,[a,b],1.6)
 return o
def bore_x(o,n,centers,r):
 c=o.component;s=c.sketches.add(c.yZConstructionPlane);s.name=n
 for y,z in centers:s.sketchCurves.sketchCircles.addByCenterRadius(pt(s,0,y,z),mm(r))
 ps=adsk.core.ObjectCollection.create()
 for p in s.profiles:ps.add(p)
 e=c.features.extrudeFeatures.createInput(ps,adsk.fusion.FeatureOperations.CutFeatureOperation)
 e.setSymmetricExtent(adsk.core.ValueInput.createByReal(mm(100)),True)
 e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);s.isVisible=False
def join_box(o,n,x0,x1,y0,y1,z0,z1):
 c=o.component;s=c.sketches.add(c.xYConstructionPlane);s.name=n
 s.sketchCurves.sketchLines.addTwoPointRectangle(pt(s,x0,y0,0),pt(s,x1,y1,0))
 e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.JoinFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(z0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(z1-z0)))
 e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);s.isVisible=False
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent;g=get(r,'07_Wedge_Lifter_Option').component
 if any(o.component.name=='Repeat_40kg_Servo_MaxEnvelope' for o in g.occurrences):raise RuntimeError('stage 2 already applied')
 metal=get(r,'Base_Plate_Al').component.bRepBodies.item(0).material
 # Vendor maximum envelope 56.2 x 44.5 x 20 mm, shaft orientation along X.
 servo=box(g,'Repeat_40kg_Servo_MaxEnvelope',-25.1,31.1,12,56.5,12,32)
 # Two foundation rails: final lug drilling is to be located from supplier STEP.
 for n,x0,x1 in [('L',-25.1,-21.1),('R',27.1,31.1)]:
  box(g,'Lifter_Servo_Base_Rail_'+n,x0,x1,14,54,5,12,metal)
 horn=yzbar(g,'Lifter_Aluminium_Horn',-28.1,-25.1,(42,22),(57,22),6,metal)
 link=yzbar(g,'Lifter_Connecting_Rod',-31.1,-28.1,(57,22),(63,45),6,metal)
 boss=get(r,'Lifter_Hinge_Boss')
 join_box(boss,'Welded_Lifter_Lever',-28.1,-25.1,59,69,31,49)
 bore_x(boss,'Lever_M3_Clevis_Hole',[(63,45)],1.6)
 # Catch the blade near its lowered position if linkage or servo output fails.
 stop=box(g,'Lifter_Lower_Hard_Stop',-10,10,82,89,5,13,metal)
 for n in ['Servo_30kgcm_Envelope_Only','Lifter_Servo_Horn_Envelope_Only','Lifter_Linkage_Envelope_Only']:
  get(r,n).isLightBulbOn=False
 print(json.dumps({'servo':'Repeat 40kg 7.4V','max_envelope_mm':[56.2,44.5,20],
                   'servo_box_mm':[[-25.1,12,12],[31.1,56.5,32]],
                   'horn_pivots_yz_mm':[[42,22],[57,22]],
                   'link_pivots_yz_mm':[[57,22],[63,45]],'hard_stop':'added'}))
