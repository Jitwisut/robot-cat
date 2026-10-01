"""Drive the lifter joint through its range and restore the original pose."""
import adsk.core,adsk.fusion,json,math
def box(o):
 b=o.bRepBodies.item(0).boundingBox
 return [[round(getattr(b.minPoint,a)*10,2) for a in 'xyz'],[round(getattr(b.maxPoint,a)*10,2) for a in 'xyz']]
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent;g=next(o for o in r.occurrences if o.component.name=='07_Wedge_Lifter_Option').component
 j=next(q for q in g.asBuiltJoints if q.name=='Lifter_Main_Revolute')
 m=j.jointMotion;original=m.rotationValue;out=[]
 try:
  for deg in [0,-15,-30,-45]:
   m.rotationValue=math.radians(deg);adsk.doEvents()
   vals={}
   for n in ['Lifter_Spatula_62mm','Lifter_Aluminium_Horn','Lifter_Connecting_Rod']:
    o=next(q for q in r.allOccurrences if q.component.name==n);vals[n]=box(o)
   out.append({'requested_deg':deg,'reported_deg':round(math.degrees(m.rotationValue),2),'boxes':vals})
 finally:
  m.rotationValue=original;adsk.doEvents()
 print(json.dumps({'samples':out,'restored_deg':round(math.degrees(m.rotationValue),2)}))
