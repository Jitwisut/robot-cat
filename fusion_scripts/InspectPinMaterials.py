import adsk.core,adsk.fusion,json
def run(_):
 r=adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
 out={}
 for n in ['Lifter_Pivot_Pin_6mm','Spinner_Shaft','Spinner_Bar','Base_Plate_Al','Lifter_Connecting_Rod']:
  o=next(q for q in r.allOccurrences if q.component.name==n);b=o.bRepBodies.item(0)
  out[n]={'material':b.material.name,'mass_g':round(b.physicalProperties.mass*1000,2)}
 print(json.dumps(out))
