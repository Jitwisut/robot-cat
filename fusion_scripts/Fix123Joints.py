"""As-built lifter joints at actual bore centres, with a limited lift angle."""
import adsk.core, adsk.fusion, json, traceback, math
def mm(v):return v/10
def p(sk,x,y,z):return sk.modelToSketchSpace(adsk.core.Point3D.create(mm(x),mm(y),mm(z)))
def occ(r,n):return next(o for o in r.allOccurrences if o.component.name==n)
def cyl_face(o,rad,y,z):
 for f in o.bRepBodies.item(0).faces:
  g=f.geometry
  if g.objectType=='adsk::core::Cylinder' and abs(g.radius*10-rad)<0.05 and abs(g.origin.y*10-y)<0.05 and abs(g.origin.z*10-z)<0.05:
   return f
 raise RuntimeError('face missing '+o.component.name+' '+str((rad,y,z)))
def shaft_on_servo(o):
 c=o.component
 if any(s.name=='Servo_Output_Shaft_D3' for s in c.sketches):return
 s=c.sketches.add(c.yZConstructionPlane);s.name='Servo_Output_Shaft_D3'
 s.sketchCurves.sketchCircles.addByCenterRadius(p(s,0,42,22),mm(1.5))
 e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.JoinFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(-28.1)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(4.0)))
 e.participantBodies=[c.bRepBodies.item(0)];c.features.extrudeFeatures.add(e);s.isVisible=False
def add(r,n,a,b,typ,face=None,limits=None):
 if any(j.name==n for j in r.asBuiltJoints):return 'existing'
 x=occ(r,a);y=occ(r,b)
 geom=None
 if face is not None:
  f=cyl_face(occ(r,face[0]),face[1],face[2],face[3])
  geom=adsk.fusion.JointGeometry.createByNonPlanarFace(f,adsk.fusion.JointKeyPointTypes.MiddleKeyPoint)
 j=r.asBuiltJoints.createInput(x,y,geom)
 if typ=='rigid':j.setAsRigidJointMotion()
 else:j.setAsRevoluteJointMotion(adsk.fusion.JointDirections.XAxisJointDirection)
 q=r.asBuiltJoints.add(j);q.name=n
 if limits:
  lim=q.jointMotion.rotationLimits
  lim.isMinimumValueEnabled=True;lim.minimumValue=math.radians(limits[0])
  lim.isMaximumValueEnabled=True;lim.maximumValue=math.radians(limits[1])
 return 'added'
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent;shaft_on_servo(occ(r,'Repeat_40kg_Servo_MaxEnvelope'))
 rows=[
  ('Lifter_Left_Support_to_Base','Base_Plate_Al','Lifter_Pivot_Support_L','rigid',None,None),
  ('Lifter_Right_Support_to_Base','Base_Plate_Al','Lifter_Pivot_Support_R','rigid',None,None),
  ('Lifter_Servo_to_Base','Base_Plate_Al','Repeat_40kg_Servo_MaxEnvelope','rigid',None,None),
  ('Lifter_Blade_to_Boss','Lifter_Hinge_Boss','Lifter_Spatula_62mm','rigid',None,None),
  ('Lifter_Pin_to_Support','Lifter_Pivot_Support_L','Lifter_Pivot_Pin_6mm','rigid',None,None),
  ('Lifter_Main_Revolute','Lifter_Pivot_Support_L','Lifter_Hinge_Boss','revolute',('Lifter_Pivot_Support_L',3.2,68,31.5),(0,45)),
  ('Lifter_Servo_Revolute','Repeat_40kg_Servo_MaxEnvelope','Lifter_Aluminium_Horn','revolute',('Repeat_40kg_Servo_MaxEnvelope',1.5,42,22),None),
  ('Lifter_Horn_Link_Revolute','Lifter_Aluminium_Horn','Lifter_Connecting_Rod','revolute',('Lifter_Aluminium_Horn',1.6,57,22),None),
  ('Lifter_Link_Lever_Revolute','Lifter_Connecting_Rod','Lifter_Hinge_Boss','revolute',('Lifter_Connecting_Rod',1.6,63,45),None)]
 result={}
 for row in rows:
  try:result[row[0]]=add(r,*row)
  except Exception as ex:result[row[0]]='ERROR: '+str(ex)
 print(json.dumps({'root_as_built_joints':r.asBuiltJoints.count,'results':result}))
