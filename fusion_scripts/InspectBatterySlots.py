import adsk.core,adsk.fusion,json
def run(_):
 r=adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
 o=next(x for x in r.allOccurrences if x.component.name=='Battery_Holder')
 out=[]
 for s in o.component.sketches:
  if 'Strap' in s.name:
   lines=[]
   for l in s.sketchCurves.sketchLines:
    p=s.sketchToModelSpace(l.startSketchPoint.geometry);q=s.sketchToModelSpace(l.endSketchPoint.geometry)
    lines.append([[round(getattr(p,a)*10,2) for a in 'xyz'],[round(getattr(q,a)*10,2) for a in 'xyz']])
   out.append({'name':s.name,'lines':lines})
 print(json.dumps(out))
