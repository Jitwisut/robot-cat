import adsk.core,adsk.fusion,json,math
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent;g=next(o for o in r.occurrences if o.component.name=='07_Wedge_Lifter_Option').component
 j=next(q for q in g.asBuiltJoints if q.name=='Lifter_Main_Revolute')
 lim=j.jointMotion.rotationLimits
 lim.isMinimumValueEnabled=True;lim.minimumValue=math.radians(-45)
 lim.isMaximumValueEnabled=True;lim.maximumValue=0
 print(json.dumps({'lift_range_deg':[-45,0],'rest_deg':round(math.degrees(j.jointMotion.rotationValue),2)}))
