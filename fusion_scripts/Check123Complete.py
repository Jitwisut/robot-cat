import adsk.core,adsk.fusion,json
EXCLUDE={'Spinner_Sweep_Envelope','Beater_Sweep_Envelope','PCB_Keepout_Envelope','ESP32_Dupont_Keepout_A','ESP32_Dupont_Keepout_B'}
def bb(b):
 q=b.boundingBox;return [[round(getattr(q.minPoint,a)*10,2) for a in 'xyz'],[round(getattr(q.maxPoint,a)*10,2) for a in 'xyz']]
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct);r=d.rootComponent
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 bodies=[];groups={}
 for top in r.occurrences:
  groups[top.component.name]={'visible':top.isVisible,'identity':all(abs(top.transform2.getCell(i,j)-(1 if i==j else 0))<.001 for i in range(3) for j in range(4))}
  if not top.isVisible:continue
  for o in top.childOccurrences:
   if o.isVisible:
    for b in o.bRepBodies:
     if b.name not in EXCLUDE:bodies.append(b)
 oc=adsk.core.ObjectCollection.create()
 for b in bodies:oc.add(b)
 inp=d.createInterferenceInput(oc);inp.areCoincidentFacesIncluded=False
 ints=d.analyzeInterference(inp);pairs=[];tbm=adsk.fusion.TemporaryBRepManager.get()
 for i in range(ints.count):
  v=ints.item(i);x=v.entityOne;y=v.entityTwo
  if not (x.name.startswith(('BBB_','Repeat_4mm','Lifter_','Battery_Clamp')) or y.name.startswith(('BBB_','Repeat_4mm','Lifter_','Battery_Clamp'))):continue
  t=tbm.copy(x);tbm.booleanOperation(t,tbm.copy(y),adsk.fusion.BooleanTypes.IntersectionBooleanType)
  pairs.append({'a':x.name,'b':y.name,'volume_mm3':round(t.volume*1000,3)})
 keys=['BBB_22mm_Gearmotor_L','BBB_22mm_Gearmotor_R','BBB_4mm_Output_Shaft_L','BBB_4mm_Output_Shaft_R',
       'BBB_Motor_Cradle_L','BBB_Motor_Cradle_R','Repeat_4mm_Clamp_Hub_L','Repeat_4mm_Clamp_Hub_R',
       'BBB_Dual_Brushed_ESC_v2_Envelope','Repeat_40kg_Servo_MaxEnvelope','Left_Wheel','Right_Wheel']
 parts={}
 for n in keys:
  o=next((q for q in r.allOccurrences if q.component.name==n),None)
  if o:parts[n]={'visible':o.isVisible,'box_mm':bb(o.bRepBodies.item(0))}
 print(json.dumps({'groups':groups,'joints':r.asBuiltJoints.count+sum(t.component.asBuiltJoints.count for t in r.occurrences),
                   'parts':parts,'interferences':pairs},separators=(',',':')))
