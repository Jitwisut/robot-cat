import adsk.core,adsk.fusion,json
def run(_):
 r=adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
 out={}
 for n in ['Top_Cover','FingerTech_Mini_Switch_Envelope']:
  o=next(q for q in r.allOccurrences if q.component.name==n);cs=[]
  for s in o.component.sketches:
   if 'Access' in s.name or 'Mount' in s.name:
    for c in s.sketchCurves.sketchCircles:
     p=s.sketchToModelSpace(c.centerSketchPoint.geometry)
     cs.append({'sketch':s.name,'xyz_mm':[round(getattr(p,a)*10,2) for a in 'xyz'],'radius_mm':round(c.radius*10,2)})
  b=o.bRepBodies.item(0).boundingBox
  out[n]={'box_mm':[[round(getattr(b.minPoint,a)*10,2) for a in 'xyz'],[round(getattr(b.maxPoint,a)*10,2) for a in 'xyz']],'circles':cs}
 print(json.dumps(out))
