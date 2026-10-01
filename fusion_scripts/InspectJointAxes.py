import adsk.core,adsk.fusion,json
def run(_):
 r=adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
 g=next(o for o in r.occurrences if o.component.name=='07_Wedge_Lifter_Option').component
 out=[]
 for j in g.asBuiltJoints:
  m=j.jointMotion
  if m.objectType=='adsk::fusion::RevoluteJointMotion':
   v=m.rotationAxisVector
   out.append({'name':j.name,'dir':[round(v.x,3),round(v.y,3),round(v.z,3)],'value':m.rotationValue})
 print(json.dumps(out))
