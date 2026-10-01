import adsk.core,adsk.fusion,json
OVERRIDE={'Repeat_40kg_Servo_MaxEnvelope':62,'BBB_22mm_Gearmotor_L':61,'BBB_22mm_Gearmotor_R':61,
 'BBB_4mm_Output_Shaft_L':0,'BBB_4mm_Output_Shaft_R':0,
 'BBB_Dual_Brushed_ESC_v2_Envelope':9,'FingerTech_Mini_Switch_Envelope':2.15,
 'UBEC_5A':21,'Repeat_4mm_Clamp_Hub_L':3.2,'Repeat_4mm_Clamp_Hub_R':3.2}
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct);r=d.rootComponent
 result={};parts=[];tot=0
 for top in r.occurrences:
  if not top.isVisible:continue
  s=0
  for o in top.childOccurrences:
   if not o.isVisible:continue
   for b in o.bRepBodies:
    if b.name.endswith(('Sweep_Envelope','Keepout_A','Keepout_B')):continue
    mass=OVERRIDE.get(b.name,b.physicalProperties.mass*1000);s+=mass
    if b.name in OVERRIDE:parts.append({'name':b.name,'cad_g':round(b.physicalProperties.mass*1000,2),'bill_of_materials_g':mass})
  result[top.component.name]=round(s,2);tot+=s
 print(json.dumps({'group_mass_g':result,'estimate_total_g':round(tot,2),'vendor_mass_overrides':parts,
                   'limitation':'battery_and_tray and other surrogate masses not validated'}))
