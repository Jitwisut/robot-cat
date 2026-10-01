"""3S drivetrain revision using BBB 22 mm gearmotors and dual brushed ESC."""
import adsk.core,adsk.fusion,json
def mm(v):return v/10
def pt(s,x,y,z):return s.modelToSketchSpace(adsk.core.Point3D.create(mm(x),mm(y),mm(z)))
def get(r,n):return next(o for o in r.allOccurrences if o.component.name==n)
def part(p,n):
 o=p.occurrences.addNewComponent(adsk.core.Matrix3D.create());o.component.name=n;return o
def box(p,n,x0,x1,y0,y1,z0,z1,mat=None):
 o=part(p,n);c=o.component;s=c.sketches.add(c.xYConstructionPlane);s.name=n+'_Outline'
 s.sketchCurves.sketchLines.addTwoPointRectangle(pt(s,x0,y0,0),pt(s,x1,y1,0))
 e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(z0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(z1-z0)))
 b=c.features.extrudeFeatures.add(e).bodies.item(0);b.name=n
 if mat:b.material=mat
 s.isVisible=False;return o
def x_cylinder(p,n,x0,x1,y,z,r,mat=None):
 o=part(p,n);c=o.component;s=c.sketches.add(c.yZConstructionPlane);s.name=n+'_Section'
 s.sketchCurves.sketchCircles.addByCenterRadius(pt(s,0,y,z),mm(r))
 e=c.features.extrudeFeatures.createInput(s.profiles.item(0),adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(x0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(x1-x0)))
 b=c.features.extrudeFeatures.add(e).bodies.item(0);b.name=n
 if mat:b.material=mat
 s.isVisible=False;return o
def x_holes(o,n,centers,r,x0,x1):
 c=o.component;s=c.sketches.add(c.yZConstructionPlane);s.name=n
 for y,z in centers:s.sketchCurves.sketchCircles.addByCenterRadius(pt(s,0,y,z),mm(r))
 ps=adsk.core.ObjectCollection.create()
 for p in s.profiles:ps.add(p)
 e=c.features.extrudeFeatures.createInput(ps,adsk.fusion.FeatureOperations.CutFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(x0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(x1-x0)))
 e.participantBodies=[c.bRepBodies.item(0)];f=c.features.extrudeFeatures.add(e);f.name=n+'_Cut';s.isVisible=False
def z_holes(o,n,centers,r,z0,z1):
 c=o.component;s=c.sketches.add(c.xYConstructionPlane);s.name=n
 for x,y in centers:s.sketchCurves.sketchCircles.addByCenterRadius(pt(s,x,y,0),mm(r))
 ps=adsk.core.ObjectCollection.create()
 for p in s.profiles:ps.add(p)
 e=c.features.extrudeFeatures.createInput(ps,adsk.fusion.FeatureOperations.CutFeatureOperation)
 e.startExtent=adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(mm(z0)))
 e.setDistanceExtent(False,adsk.core.ValueInput.createByReal(mm(z1-z0)))
 e.participantBodies=[c.bRepBodies.item(0)];f=c.features.extrudeFeatures.add(e);f.name=n+'_Cut';s.isVisible=False
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent
 if any(o.component.name=='09_3S_Drive_Upgrade' for o in r.occurrences):raise RuntimeError('already applied')
 mat=get(r,'Base_Plate_Al').component.bRepBodies.item(0).material
 group=part(r,'09_3S_Drive_Upgrade');g=group.component
 out={}
 for side in ['L','R']:
  if side=='L':
   body=(-58,-9);shaft=(-72,-58);plate=(-61,-58);cradle=(-58,-48);hub=(-67,-61);wheel='Left_Wheel';xc=-53
  else:
   body=(9,58);shaft=(58,72);plate=(58,61);cradle=(48,58);hub=(61,67);wheel='Right_Wheel';xc=53
  # Published motor body 49 mm by 22 mm, 4 mm shaft extending 14 mm.
  x_cylinder(g,'BBB_22mm_Gearmotor_'+side,*body,0,21,11,mat)
  x_cylinder(g,'BBB_4mm_Output_Shaft_'+side,*shaft,0,21,2,mat)
  face=box(g,'BBB_Face_Mount_26mm_'+side,*plate,-13,13,8,34,mat)
  x_holes(face,'Face_Motor_Bore_'+side,[(0,21)],11.1,plate[0]-.1,plate[1]+.1)
  boltpos=[(y,z) for y in [-10,10] for z in [11,31]]
  x_holes(face,'Face_M3_20mm_Pattern_'+side,boltpos,1.6,plate[0]-.1,plate[1]+.1)
  cradle_o=box(g,'BBB_Motor_Cradle_'+side,*cradle,-20,20,5,37,mat)
  x_holes(cradle_o,'Cradle_Motor_Bore_'+side,[(0,21)],11.1,cradle[0]-.1,cradle[1]+.1)
  x_holes(cradle_o,'Cradle_M3_Face_Tap_'+side,boltpos,1.25,cradle[0]-.1,cradle[1]+.1)
  z_holes(cradle_o,'Cradle_M3_Base_Tap_'+side,[(xc,-16),(xc,16)],1.25,4.9,17)
  z_holes(get(r,'Base_Plate_Al'),'Cradle_M3_Base_Clear_'+side,[(xc,-16),(xc,16)],1.7,2.9,5.1)
  w=get(r,wheel)
  if side=='L':
   x_holes(w,'Hub_16p2_Counterbore_'+side,[(0,21)],8.1,-67.2,-60.8)
   x_holes(w,'Shaft_4p2_Blind_Bore_'+side,[(0,21)],2.1,-72.2,-67.1)
   x_holes(w,'Wheel_M3_12p7_Pattern_'+side,[(-6.35,21),(6.35,21)],1.7,-80.1,-67.1)
  else:
   x_holes(w,'Hub_16p2_Counterbore_'+side,[(0,21)],8.1,60.8,67.2)
   x_holes(w,'Shaft_4p2_Blind_Bore_'+side,[(0,21)],2.1,67.1,72.2)
   x_holes(w,'Wheel_M3_12p7_Pattern_'+side,[(-6.35,21),(6.35,21)],1.7,67.1,80.1)
  hub_o=x_cylinder(g,'Repeat_4mm_Clamp_Hub_'+side,*hub,0,21,8,mat)
  x_holes(hub_o,'Hub_4mm_Shaft_Bore_'+side,[(0,21)],2.0,hub[0]-.1,hub[1]+.1)
  x_holes(hub_o,'Hub_M3_12p7_Tap_'+side,[(-6.35,21),(6.35,21)],1.25,hub[0]-.1,hub[1]+.1)
  out[side]={'motor_body_x_mm':body,'shaft_x_mm':shaft,'hub_x_mm':hub}
 # Old 6 V TT drive and 2.1 A driver are suppressed, with historical holes retained.
 for n in ['TT_Gearbox_L','TT_Gearbox_R','TT_Motor_Can_L','TT_Motor_Can_R',
           'TT_Motor_Mount_L','TT_Motor_Mount_R','TT_Motor_Clamp_L','TT_Motor_Clamp_R',
           'DRV8874_A','DRV8874_B','Holder_DRV8874_A','Holder_DRV8874_B']:
  get(r,n).isLightBulbOn=False
 elec=get(r,'05_Electronics').component
 esc=box(elec,'BBB_Dual_Brushed_ESC_v2_Envelope',-8,12,-84,-68,7,12)
 # The existing Hobbywing UBEC_5A is retained for 7.4 V servo and receiver rail.
 print(json.dumps({'drive':out,'esc_mm':[20,16,5],'old_tt_and_drv8874_hidden':True,
                   'note':'motor, hub, and ESC are dimensioned surrogate bodies; supplier mass is used in BOM'}))
