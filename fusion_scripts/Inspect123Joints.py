import adsk.core,adsk.fusion,json
def run(_):
 r=adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
 out={}
 for c in [r]+[o.component for o in r.occurrences]:
  out[c.name]={'as_built':[c.asBuiltJoints.item(i).name for i in range(c.asBuiltJoints.count)],'joints':[c.joints.item(i).name for i in range(c.joints.count)]}
 print(json.dumps(out))
