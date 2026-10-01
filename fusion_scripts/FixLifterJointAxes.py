import adsk.core,adsk.fusion,json
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent;g=next(o for o in r.occurrences if o.component.name=='07_Wedge_Lifter_Option').component
 result={}
 for j in g.asBuiltJoints:
  m=j.jointMotion
  if m.objectType!='adsk::fusion::RevoluteJointMotion':continue
  n=j.name
  try:
   j.timelineObject.rollTo(True)
   if not j.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection):
    raise RuntimeError('axis edit failed')
  finally:d.timeline.moveToEnd()
  jj=next(q for q in g.asBuiltJoints if q.name==n)
  v=jj.jointMotion.rotationAxisVector
  result[n]=[round(v.x,3),round(v.y,3),round(v.z,3)]
 print(json.dumps(result))
