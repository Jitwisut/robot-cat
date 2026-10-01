import adsk.core,adsk.fusion,json,math
EXCLUDE={'Spinner_Sweep_Envelope','Beater_Sweep_Envelope','PCB_Keepout_Envelope','ESP32_Dupont_Keepout_A','ESP32_Dupont_Keepout_B'}
MOVING={'Lifter_Spatula_62mm','Lifter_Hinge_Boss','Lifter_Aluminium_Horn','Lifter_Connecting_Rod'}
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent;g=next(o for o in r.occurrences if o.component.name=='07_Wedge_Lifter_Option').component
 j=next(q for q in g.asBuiltJoints if q.name=='Lifter_Main_Revolute');m=j.jointMotion;old=m.rotationValue
 out=[];tbm=adsk.fusion.TemporaryBRepManager.get()
 try:
  for deg in [0,-15,-30,-45]:
   m.rotationValue=math.radians(deg);adsk.doEvents()
   bodies=[]
   for top in r.occurrences:
    if not top.isVisible:continue
    for o in top.childOccurrences:
     if o.isVisible:
      for b in o.bRepBodies:
       if b.name not in EXCLUDE:bodies.append(b)
   col=adsk.core.ObjectCollection.create()
   for b in bodies:col.add(b)
   inp=d.createInterferenceInput(col);inp.areCoincidentFacesIncluded=False
   rr=d.analyzeInterference(inp);pairs=[]
   for i in range(rr.count):
    x=rr.item(i).entityOne;y=rr.item(i).entityTwo
    if x.name not in MOVING and y.name not in MOVING:continue
    if {x.name,y.name}=={'Lifter_Spatula_62mm','Lifter_Hinge_Boss'}:continue
    c=tbm.copy(x);tbm.booleanOperation(c,tbm.copy(y),adsk.fusion.BooleanTypes.IntersectionBooleanType)
    pairs.append([x.name,y.name,round(c.volume*1000,2)])
   out.append({'deg':deg,'reported_deg':round(math.degrees(m.rotationValue),1),'unintended_interferences':pairs})
 finally:
  m.rotationValue=old;adsk.doEvents()
 print(json.dumps({'motion_checks':out,'restored_deg':round(math.degrees(m.rotationValue),1)}))
