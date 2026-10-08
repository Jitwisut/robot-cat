"""Parametric R1 features, applied to the archived geometry, never its exports."""
import math
from cad_backend import *
from legacy_geometry import build_baseline, d_bore, CARRIER


def value(config, part_name, key):
    item = config['hardware'][part_name]
    actual = item['actual'].get(key)
    if actual is not None:
        return actual
    return item.get('catalog', {}).get('values', {}).get(key, item['nominal'][key])


def fit(config, key, default):
    result = config['fit'][key]
    return default if result is None else result


def rectangular_part(root,name,x0,x1,y0,y1,z0,z1,material='PETG'):
    return box(root,name,x0,x1,y0,y1,z0,z1,material).body


def replace_box(body, centre, dimensions):
    x,y,z=centre; dx,dy,dz=dimensions
    body.shape=block(x-dx/2,x+dx/2,y-dy/2,y+dy/2,z,z+dz)


def build(config):
    design=config['design']
    front,rear,motor_y=(design[k] for k in ('front_axle_y_mm','rear_axle_y_mm','weapon_motor_y_mm'))
    motor_z=design['weapon_motor_z_mm']
    if rear != -70:
        raise ValueError('R1 rear axle fixed at -70; a changed rear axle requires a new layout revision')
    root=build_baseline(front,motor_y)
    tub=find_body(root,'Tub_TPU'); lid=find_body(root,'Lid_TPU_2mm')
    wheel_dims=[]
    # Four independently measured motors; keep their mounting faces at x +/-56.
    for loc,wy in [('Front',front),('Rear',rear)]:
        for side,sgn in [('L',-1),('R',1)]:
            name='drive_'+loc+'_'+side
            length=value(config,name,'length_mm'); radius=value(config,name,'diameter_mm')/2
            body=find_body(root,'JGA25_370_400rpm_'+loc+'_'+side)
            x0,x1=sorted((sgn*56,sgn*(56-length)))
            body.shape=prism([('circle',(wy,32,radius))],'x',x0,x1)
            # Shaft and boss clearance are independently sized.
            boss=value(config,name,'boss_diameter_mm')
            xa,xb=sorted((sgn*55,sgn*61))
            bore(tub,(xa,wy,32),(1,0,0),max(boss/2+0.25,4.0),0,xb-xa)
            for dy,dz in value(config,name,'mount_holes_yz_mm'):
                bore(tub,(xa,wy+dy,32+dz),(1,0,0),value(config,name,'mount_thread_diameter_mm')/2+0.2,0,xb-xa)
                # Button head + washer must clear the 2 mm hub-to-wall gap.
                head_a,head_b=sorted((sgn*59,sgn*61))
                bore(tub,(head_a,wy+dy,32+dz),(1,0,0),3.75,0,head_b-head_a)
            wx0,wx1=sorted((sgn*62,sgn*78))
            d=value(config,name,'shaft_diameter_mm')+fit(config,'d_bore_diametral_clearance_mm',0.10)
            flat=value(config,name,'shaft_diameter_mm')/2-value(config,name,'shaft_flat_depth_mm')+fit(config,'d_bore_flat_clearance_mm',0.05)
            hub=find_body(root,'Wheel_Hub_PETG_'+loc+'_'+side)
            # Keep a solid spoke towards the D-flat; put the five holes elsewhere.
            loops=[('circle',(wy,32,22)),('poly',d_bore(wy,32,d/2,flat,144))]
            loops += [('circle',(wy+13*math.cos(math.radians(a)),32+13*math.sin(math.radians(a)),4)) for a in (60,120,180,240,300)]
            hub.shape=prism(loops,'x',wx0,wx1);hub.material='PETG'
            # Radial M3 set screw from the accessible outer rim, captive steel hex nut.
            xc=(wx0+wx1)/2
            nut_af=value(config,'nut_m3','across_flats_mm')+fit(config,'nut_pocket_clearance_mm',0.2)
            nut_h=value(config,'nut_m3','height_mm')+0.2
            nut_r=nut_af/math.sqrt(3)
            # Horizontal flats are parallel to the loading tunnel: its height
            # is across-flats, not across-corners, so the nut cannot rotate.
            poly=[(xc+nut_r*math.cos(math.radians(60*k)),32+nut_r*math.sin(math.radians(60*k))) for k in range(6)]
            hub.shape=hub.shape.cut(prism([('poly',poly)],'y',wy+7,wy+7+nut_h)).clean()
            # Nut-loading tunnel along the wheel axis, open on both wheel faces.
            hub.shape=hub.shape.cut(block(wx0-0.1,wx1+0.1,wy+7,wy+7+nut_h,32-nut_af/2,32+nut_af/2)).clean()
            bore(hub,(xc,wy+flat-0.2,32),(0,1,0),1.7,0,22-flat+0.4)
            tyre=find_body(root,'Wheel_Tyre_TPU_'+loc+'_'+side)
            tyre_id=fit(config,'tyre_inner_diameter_mm',43.8)
            tyre.shape=prism([('circle',(wy,32,32)),('circle',(wy,32,tyre_id/2))],'x',wx0,wx1)
            wheel_dims.append({'motor':name,'shaft_min_engagement_mm':10,'needed_projection_mm':16,'available_projection_mm':value(config,name,'shaft_projection_mm')})
    # Rebuild fixed hinge blocks. Bore grows with calibration; minimum wall is invariant.
    pin_d=value(config,'hinge_pin','diameter_mm')
    pin_bore=pin_d+fit(config,'hinge_diametral_clearance_mm',0.2)
    outer=pin_bore/2+design['hinge_min_wall_mm']
    insert_d=fit(config,'insert_hole_diameter_mm',design.get('insert_trial_hole_diameter_mm',value(config,'insert_m3','outer_diameter_mm')))
    insert_l=value(config,'insert_m3','length_mm')
    for side,sgn in [('L',-1),('R',1)]:
        def xr(a,b): return sorted((sgn*a,sgn*b))
        body=find_body(root,'Wedgelet_Block_PETG_'+side);body.material='PETG'
        f0,f1=xr(34,88)
        body.shape=block(f0,f1,100,104,4,11)
        for a,b in [(34,38),(60,66),(84,88)]:
            a,b=xr(a,b)
            body.shape=body.shape.fuse(prism([('circle',(102,14.5,outer))],'x',a,b)).clean()
            body.shape=body.shape.fuse(block(a,b,101,103,10,12)).clean()
            # Cylinder bottom overlaps the spine; reject disconnected solids below.
        a,b=xr(70,80)
        body.shape=body.shape.fuse(block(a,b,97,101,7,10),block(a,b,64,98,7,18)).clean()
        bore(body,(f0-1,102,14.5),(1,0,0),pin_bore/2,0,f1-f0+2)
        a,b=xr(72,80)
        for y in (80,92): bore(body,(a,y,13.5),(1,0,0),insert_d/2,0,b-a)
        for number,(a,b) in enumerate([(39,59),(67,83)],1):
            a,b=xr(a,b)
            carrier=find_body(root,'Wedgelet_Carrier_PETG_%s%d'%(side,number));carrier.material='PETG'
            carrier.shape=prism([('poly',CARRIER)],'x',a,b).fuse(prism([('circle',(102,14.5,outer))],'x',a,b)).clean()
            bore(carrier,(a-1,102,14.5),(1,0,0),pin_bore/2,0,b-a+2)
            x=sgn*(49 if number==1 else 75)
            bore(carrier,(x,109.05,14.03),(0,20/28,1),insert_d/2,insert_l,insert_l)
        # Fixed 57 mm stack is explicit; a different measured pin length is rejected.
        a,b=xr(33.5,90.5)
        pin=find_body(root,'Hinge_Pin_D3_'+side)
        pin.shape=prism([('circle',(102,14.5,pin_d/2))],'x',a,b)
        ga,gb=xr(89,89.6)
        pin.shape=pin.shape.cut(prism([('circle',(102,14.5,pin_d/2+0.1)),('circle',(102,14.5,1.15))],'x',ga,gb)).clean()
    # Lid insert geometry follows the actual insert/calibration, not the archived Ø4.
    for x in (-84,84):
        for y in (-95,-15,55): bore(tub,(x,y,58),(0,0,1),insert_d/2,insert_l+1,1)
    # Support one documented 608 stack; reject silent changes to shaft/clamp spacing.
    for key,expected in [('outer_diameter_mm',22),('inner_diameter_mm',8),('width_mm',7)]:
        if abs(value(config,'bearing_608',key)-expected)>0.1:
            raise ValueError('608 '+key+' incompatible with 7/26/7 mm spacer stack')
    if abs(value(config,'hinge_pin','length_mm')-57)>0.1:
        raise ValueError('Hinge pin must match the 57 mm stop/E-clip stack')
    # Model the full M8 fastener stack, not only its 66 mm shaft envelope.
    shaft=find_body(root,'Dead_Shaft_M8_12.9')
    washer_t=value(config,'dead_shaft','washer_thickness_mm')
    bolt_length=value(config,'dead_shaft','length_mm')
    diameter=value(config,'dead_shaft','diameter_mm')
    af=value(config,'dead_shaft','head_across_flats_mm')
    head_t=value(config,'dead_shaft','head_thickness_mm')
    start=-33-washer_t;end=start+bolt_length
    polygon=[(95+af/math.sqrt(3)*math.cos(math.radians(k*60)),32+af/math.sqrt(3)*math.sin(math.radians(k*60))) for k in range(6)]
    shaft.shape=prism([('circle',(95,32,diameter/2))],'x',start,end).fuse(prism([('poly',polygon)],'x',start-head_t,start)).clean()
    nut_t=value(config,'dead_shaft','nut_thickness_mm');nut_af=value(config,'dead_shaft','nut_across_flats_mm')
    polygon=[(95+nut_af/math.sqrt(3)*math.cos(math.radians(k*60)),32+nut_af/math.sqrt(3)*math.sin(math.radians(k*60))) for k in range(6)]
    solid_x(root,'Weapon_Axle_Nut_M8', [('poly',polygon),('circle',(95,32,diameter/2))],33+washer_t,33+washer_t+nut_t,'steel')
    for side,a,b in [('L',-33-washer_t,-33),('R',33,33+washer_t)]:
        cyl_x(root,'Weapon_Axle_Washer_'+side,a,b,95,32,value(config,'dead_shaft','washer_outer_diameter_mm')/2,'steel',bore=value(config,'dead_shaft','washer_inner_diameter_mm')/2)
    # Socket access through the adjacent TPU blocks; no drilling after print.
    for a,b in [(-51,-33),(33,51)]:bore(tub,(a,95,32),(1,0,0),10.5,0,b-a)
    # Actual weapon-motor can/shaft and mounting pattern.
    motor=find_body(root,'D3536_1250kV_Can_ENVELOPE')
    mr=value(config,'weapon_motor','diameter_mm')/2
    motor.shape=prism([('circle',(motor_y,motor_z,mr))],'x',-41,-41+value(config,'weapon_motor','length_mm'))
    # Relieve the front bulkhead around the raised motor with 1 mm radial clearance.
    bore(tub,(-42,motor_y,motor_z),(1,0,0),mr+1,0,value(config,'weapon_motor','length_mm')+2)
    # Small circular relief at the rear edge, away from all four upright screws.
    bore(find_body(root,'Upright_6061_6mm_L'),(-34,motor_y,motor_z),(1,0,0),mr+1,0,8)
    # Arched 2 mm TPU lid roof; 1 mm bell clearance, top below the inverted wheel plane.
    mx1=-41+value(config,'weapon_motor','length_mm')
    outer=prism([('circle',(motor_y,motor_z,mr+3))],'x',-41.5,mx1+0.5)
    roof=outer.intersect(block(-42,mx1+1,motor_y-mr-4,motor_y+mr+4,58,motor_z+mr+3.01))
    lid.shape=lid.shape.fuse(roof).clean()
    bore(lid,(-42,motor_y,motor_z),(1,0,0),mr+1,0,value(config,'weapon_motor','length_mm')+2)
    mount=find_body(root,'Weapon_Motor_Mount_3mm')
    mount.shape=block(-45,-42,22,62,10,58)
    bore(mount,(-46,motor_y,motor_z),(1,0,0),value(config,'weapon_motor','boss_diameter_mm')/2+0.25,0,6)
    for dy,dz in value(config,'weapon_motor','mount_holes_yz_mm'):
        bore(mount,(-46,motor_y+dy,motor_z+dz),(1,0,0),value(config,'weapon_motor','mount_thread_diameter_mm')/2+0.2,0,6)
    # The moved front drive motor needs a curved relief in the metal plate.
    front_radius=value(config,'drive_Front_L','diameter_mm')/2
    bore(mount,(-46,front,32),(1,0,0),front_radius+1,0,6)
    # Two PETG feet, nut pockets closed by the metal motor plate. Separate vertical bolts.
    for index,(ya,yb,hy,fy) in enumerate([(22,35,31,25),(50,61,53,59)],1):
        foot=rectangular_part(root,'Weapon_Mount_Foot_PETG_%d'%index,-53,-45,ya,yb,7,20)
        bore(foot,(-54,hy,15),(1,0,0),1.7,0,12)
        af=value(config,'nut_m3','across_flats_mm')+0.2
        poly=[(hy+af/math.sqrt(3)*math.cos(math.radians(30+60*k)),15+af/math.sqrt(3)*math.sin(math.radians(30+60*k))) for k in range(6)]
        foot.shape=foot.shape.cut(prism([('poly',poly)],'x',-45-value(config,'nut_m3','height_mm')-0.2,-44.9)).clean()
        bore(mount,(-46,hy,15),(1,0,0),1.7,0,6)
        for target in (foot,tub): bore(target,(-49,fy,4),(0,0,1),1.7,0,18)
    pulley=find_body(root,'Motor_Pulley_D26_Groove')
    pulley.name='Motor_Pulley_D30_1to1'
    pulley.shape=prism([('circle',(motor_y,motor_z,15)),('circle',(motor_y,motor_z,value(config,'weapon_motor','shaft_diameter_mm')/2))],'x',-4,4)
    pulley.shape=pulley.shape.cut(prism([('circle',(motor_y,motor_z,15.5)),('circle',(motor_y,motor_z,13.5))],'x',-3,3)).clean()
    boss=prism([('circle',(motor_y,motor_z,5)),('circle',(motor_y,motor_z,value(config,'weapon_motor','shaft_diameter_mm')/2))],'x',4,9)
    pulley.shape=pulley.shape.fuse(boss).clean()
    bore(pulley,(6,motor_y,motor_z),(0,1,0),1.25,0,6)
    # Balance holes for opposite-polarity Hall magnets on the exposed flange.
    for z in (motor_z-7,motor_z+7):bore(pulley,(2,motor_y,z),(1,0,0),1.5,0,2.1)
    # A through-notch must be present in the laser outline, not a blind belt pocket.
    top_brace=find_body(root,'Weapon_Top_Brace_3mm')
    top_brace.shape=block(-33,33,64,77,57,60)
    for x in (-30,-15,15,30):bore(top_brace,(x,69,56),(0,0,1),1.7,0,5)
    top_brace.shape=top_brace.shape.cut(block(-3.5,3.5,63,72.5,56,61)).clean()
    # Actual 5 mm round-belt straight runs with external tangency, not rectangular proxies.
    dy,dz=95-motor_y,32-motor_z
    dist=math.hypot(dy,dz);u=(dy/dist,dz/dist);base=(-u[1],u[0]);s=0
    for name,sign in [('Belt_Upper_ENVELOPE',1),('Belt_Lower_ENVELOPE',-1)]:
        n=(s*u[0]+sign*math.sqrt(1-s*s)*base[0],s*u[1]+sign*math.sqrt(1-s*s)*base[1])
        a=cq.Vector(0,motor_y+16*n[0],motor_z+16*n[1]);b=cq.Vector(0,95+16*n[0],32+16*n[1])
        belt=find_body(root,name);belt.shape=cq.Solid.makeCylinder(2.5,(b-a).Length,a,(b-a).normalized());belt.material='belt'
        clearance=cq.Solid.makeCylinder(3.5,(b-a).Length,a,(b-a).normalized())
        for target in (tub,lid,find_body(root,'Weapon_Top_Brace_3mm')):
            target.shape=target.shape.cut(clearance).clean()
    tub.shape=tub.shape.cut(block(-6,6,61,65,12,57)).clean()
    # Removable battery tray, two full-width strap paths through its edge slots.
    bd=value(config,'battery','dimensions_mm');bx,by,bz=bd
    battery_y=-55.5+by/2
    battery_z=9.6
    battery=find_body(root,'3S_850mAh_LiPo_ENVELOPE_75x35x25')
    replace_box(battery,(0,battery_y,battery_z),bd)
    x0,x1=-bx/2-4,bx/2+4;y0,y1=battery_y-by/2-1.5,battery_y+by/2+1.5
    tray=rectangular_part(root,'Battery_Tray_PETG',x0,x1,y0,y1,7,8.6)
    tray.shape=tray.shape.fuse(block(x0,x0+2,y0,y1,8.6,12),block(x1-2,x1,y0,y1,8.6,12),block(x0,x1,y0,y0+1.5,8.6,12),block(x0,x1,y1-1.5,y1,8.6,12)).clean()
    for y in (battery_y-by/4,battery_y+by/4):
        for a,b in [(x0+2.1,x0+3.5),(x1-3.5,x1-2.1)]:
            tray.shape=tray.shape.cut(block(a,b,y-5.5,y+5.5,6,13)).clean()
    for x in (-bx/2-2.5,bx/2+2.5):
        for y in (battery_y-by/2+3,battery_y+by/2-3):
            for target in (tray,tub): bore(target,(x,y,3),(0,0,1),1.7,0,11)
    # Deck and four separate standoffs print flat, without a large supported shelf.
    deck_z=battery_z+bz+4
    deck=rectangular_part(root,'Electronics_Platform_PETG',-61,61,-56,-3,deck_z,deck_z+2)
    standoff_number=0
    for x in (-57,57):
        for y in (-52,-8):
            standoff_number+=1
            comp=part(root,'Electronics_Standoff_PETG_%d'%standoff_number)
            comp.body=Body(cq.Solid.makeCylinder(4,deck_z-7,cq.Vector(x,y,7)),comp.name,'PETG',comp)
            for target in (deck,tub,comp.body): bore(target,(x,y,3),(0,0,1),1.7,0,deck_z+2)
    zboard=deck_z+3
    components=[('esc','Skywalker_40A_ESC_55x25x12',-28,-42.5),('esp32','ESP32_DevKit_51.5x28.3',28,-41.35),('drv_left','DRV8871_L',-42,-14),('drv_right','DRV8871_R',42,-14),('imu','IMU_GY521_MPU6050_21x16',0,-13)]
    for key,name,cx,cy in components:
        cx,cy=design.get('electronics_xy_mm',{}).get(key,[cx,cy])
        dims=value(config,key,'dimensions_mm')
        replace_box(find_body(root,name),(cx,cy,zboard),dims)
        dx,dy,dz=dims
        # Insulated deck + 1 mm foam; edge stops and two tie loops per module.
        for sign in (-1,1):
            a=cx+sign*(dx/2+0.75)
            deck.shape=deck.shape.fuse(block(a-0.5,a+0.5,cy-dy/2,cy+dy/2,deck_z+2,zboard+1)).clean()
            for yy in (cy-dy/4,cy+dy/4):
                deck.shape=deck.shape.cut(block(a-0.7,a+0.7,yy-1.8,yy+1.8,deck_z-0.1,zboard+2)).clean()
        keep=value(config,key,'connector_keepout_mm')
        offset=value(config,key,'connector_x_offset_mm')
        edge=value(config,key,'connector_side')
        if edge in ('front','rear'):
            ya,yb=(cy+dy/2,cy+dy/2+keep[1]) if edge=='front' else (cy-dy/2-keep[1],cy-dy/2)
            xa,xb=cx+offset-keep[0]/2,cx+offset+keep[0]/2
        elif edge in ('left','right'):
            xa,xb=(cx-dx/2-keep[0],cx-dx/2) if edge=='left' else (cx+dx/2,cx+dx/2+keep[0])
            ya,yb=cy+offset-keep[1]/2,cy+offset+keep[1]/2
        else: raise ValueError('Unsupported connector side: '+key)
        rectangular_part(root,name+'_CONNECTOR_KEEP_OUT',xa,xb,ya,yb,zboard,zboard+keep[2],'keepout')
        if edge in ('left','right'):
            # Side-exit wires need an opening through the module's edge stop.
            deck.shape=deck.shape.cut(block(xa,xb,ya,yb,zboard,zboard+keep[2])).clean()
        for index,extra in enumerate(design.get('extra_connector_keepouts',{}).get(key,[])):
            xa,xb,ya,yb,za,zb=extra
            rectangular_part(root,name+'_EXTRA_CONNECTOR_KEEP_OUT_%d'%index,xa,xb,ya,yb,zboard+za,zboard+zb,'keepout')
            deck.shape=deck.shape.cut(block(xa,xb,ya,yb,zboard+za,zboard+zb)).clean()
    # Battery wire exit through the gap between the upper modules.
    bore(deck,(0,-25,deck_z-1),(0,0,1),3,0,5)
    foam=find_body(root,'IMU_Foam_Pad_1mm')
    imu_x,imu_y=design.get('electronics_xy_mm',{}).get('imu',[0,-13])
    replace_box(foam,(imu_x,imu_y,deck_z+2),[21,16,1]);foam.material='foam'
    # Move Hall with the weapon pulley; keep sensing face 2.5 mm from its face.
    hall=find_body(root,'Hall_A3144_TO92')
    hs=value(config,'hall_sensor','dimensions_mm')
    hall.shape=block(6.5,6.5+hs[0],motor_y-hs[1]/2,motor_y+hs[1]/2,motor_z+7-hs[2]/2,motor_z+7+hs[2]/2)
    find_body(root,'Hall_Post_PETG').material='PETG'
    hall_post=find_body(root,'Hall_Post_PETG')
    post_end=61.5
    if post_end-(motor_y+8)<3:raise ValueError('Hall support has less than 3 mm space before bulkhead')
    hall_post.shape=block(8,12,motor_y+8,post_end,7,43+motor_z-32)
    hall_post.shape=hall_post.shape.fuse(block(8,12,motor_y-2,post_end,41+motor_z-32,43+motor_z-32),block(8,18,motor_y+8,post_end,7,9)).clean()
    for target in (hall_post,tub):bore(target,(15,(motor_y+8+post_end)/2,3),(0,0,1),1.1,0,8)
    # Replace Ø6 access hole with the measured removable-link opening and a PETG collar.
    dd=value(config,'disconnect','dimensions_mm');_,dy,dz=dd
    disconnect=find_body(root,'Power_Switch_Link')
    replace_box(disconnect,(73,-30,20),dd)
    opening=block(79,89,-30-dy/2-0.5,-30+dy/2+0.5,20-0.5,20+dz+0.5)
    tub.shape=tub.shape.cut(opening).clean()
    collar=rectangular_part(root,'Disconnect_Collar_PETG',77,80,-30-dy/2-4,-30+dy/2+4,17,23+dz)
    collar.shape=collar.shape.cut(block(76,81,-30-dy/2,-30+dy/2,20,20+dz)).clean()
    for y in (-30-dy/2-2,-30+dy/2+2):
        for target in (tub,collar): bore(target,(76,y,20+dz/2),(1,0,0),1.1,0,14)
    # Material labels must reflect real printed parts rather than the archived Nylon proxy.
    for body in root.bodies():
        if 'PETG' in body.name: body.material='PETG'
    constraints={'shaft_engagement':wheel_dims,'hinge_outer_radius_mm':pin_bore/2+design['hinge_min_wall_mm'],'hinge_bore_mm':pin_bore,'hinge_wall_mm':design['hinge_min_wall_mm'],'battery_y_mm':battery_y,'electronics_deck_z_mm':deck_z,'highest_electronics_mm':max(find_body(root,n).shape.BoundingBox().zmax for _,n,_,_ in components),'deck_to_battery_gap_mm':4,'lid_top_mm':lid.shape.BoundingBox().zmax,'weapon_motor_z_mm':motor_z}
    constraints['axle_nut_full_thread']=bool(end>=33+washer_t+nut_t+diameter*0.25 and end-value(config,'dead_shaft','thread_length_mm')<=33+washer_t)
    constraints['axle_socket_clearance_mm']=21
    constraints['front_drive_to_weapon_motor_clearance_mm']=math.hypot(motor_y-front,motor_z-32)-mr-max(value(config,'drive_Front_L','diameter_mm'),value(config,'drive_Front_R','diameter_mm'))/2
    return root,constraints
