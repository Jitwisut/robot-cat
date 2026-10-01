import adsk.core, adsk.fusion, json
def run(_):
 a=adsk.core.Application.get();r=adsk.fusion.Design.cast(a.activeProduct).rootComponent;out={}
 for n in ['Lifter_Pivot_Support_L','Lifter_Hinge_Boss','Lifter_Aluminium_Horn','Lifter_Connecting_Rod','Repeat_40kg_Servo_MaxEnvelope']:
  o=next(q for q in r.allOccurrences if q.component.name==n);faces=[]
  for f in o.component.bRepBodies.item(0).faces:
   g=f.geometry
   if g.objectType=='adsk::core::Cylinder':
    faces.append({'radius_mm':round(g.radius*10,2),'origin_mm':[round(g.origin.x*10,2),round(g.origin.y*10,2),round(g.origin.z*10,2)],'axis':[round(g.axis.x,1),round(g.axis.y,1),round(g.axis.z,1)]})
  out[n]=faces
 print(json.dumps(out))
