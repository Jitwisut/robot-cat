import adsk.core, adsk.fusion, json
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct);r=d.rootComponent;out={}
 for n in ['Base_Plate_Al','Front_Wedge_L','Front_Wedge_R']:
  o=next(x for x in r.allOccurrences if x.component.name==n);ss=[]
  for s in o.component.sketches:
   if 'Wedge' in s.name:
    cs=[]
    for c in s.sketchCurves.sketchCircles:
     p=s.sketchToModelSpace(c.centerSketchPoint.geometry)
     cs.append([round(p.x*10,2),round(p.y*10,2),round(p.z*10,2),round(c.radius*10,2)])
    ss.append({'name':s.name,'circles':cs})
  out[n]=ss
 print(json.dumps(out))
