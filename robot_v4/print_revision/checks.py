"""Geometry checks, mesh audit, mass properties and release evidence."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import trimesh
import manifold3d
import cadquery as cq
from OCP.BOPAlgo import BOPAlgo_CheckerSI
from OCP.TopTools import TopTools_ListOfShape
from revision import value
from inputs import geometry_payload

ROOT=Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_signature(config):
    # Shop/evidence fields are not geometry; recording evidence must not invalidate a build.
    geometry=geometry_payload(config)
    h=hashlib.sha256(json.dumps(geometry,sort_keys=True).encode())
    for name in ('cad_backend.py','legacy_geometry.py','revision.py','build.py','checks.py','coupons.py','inputs.py','simulate.py','readiness.py'):
        p=ROOT/name
        if p.exists(): h.update(p.read_bytes())
    for path in (ROOT.parent/'mujoco_sim/sim.py',ROOT.parent/'mujoco_sim/validate.py'):
        h.update(path.read_bytes())
    return h.hexdigest()


def input_signature(config):
    geometry=geometry_payload(config)
    return hashlib.sha256(json.dumps(geometry,sort_keys=True).encode()).hexdigest()


def overlap(a,b):
    aa,bb=a.BoundingBox(),b.BoundingBox()
    if any(getattr(aa,axis+'max')<=getattr(bb,axis+'min')+1e-7 or getattr(bb,axis+'max')<=getattr(aa,axis+'min')+1e-7 for axis in 'xyz'):
        return 0.0
    return max(0.0,a.intersect(b).Volume())


def cad_checks(root,config,constraints):
    bodies=root.bodies();bad=[];self_checks=[]
    for b in bodies:
        if not b.shape.isValid() or len(b.shape.Solids())!=1:
            bad.append({'part':b.name,'valid':b.shape.isValid(),'solid_count':len(b.shape.Solids())})
        # Exact CAD self-interference check. Mesh audit below is a separate check.
        args=TopTools_ListOfShape();args.Append(b.shape.wrapped)
        checker=BOPAlgo_CheckerSI();checker.SetArguments(args);checker.SetLevelOfCheck(9);checker.Perform()
        ds=checker.DS()
        interference_count=ds.Interferences().Extent()
        self_checks.append({'part':b.name,'algorithm_errors':bool(checker.HasErrors()),'self_interferences':interference_count})
    clashes=[];expected=[]
    for i,a in enumerate(bodies):
        for b in bodies[i+1:]:
            vol=overlap(a.shape,b.shape)
            if vol<=0.01: continue
            pair={'parts':[a.name,b.name],'volume_mm3':round(vol,3)}
            allowed=(a.name.startswith('Wheel_Hub') and b.name.replace('Wheel_Tyre_TPU','Wheel_Hub_PETG')==a.name) or (b.name.startswith('Wheel_Hub') and a.name.replace('Wheel_Tyre_TPU','Wheel_Hub_PETG')==b.name)
            (expected if allowed else clashes).append(pair)
    sweep=[]
    # The belt is deliberately in contact with the centre pulley groove.
    ignored=('Beater_','Bearing_608','Dead_Shaft','Spacer_','Belt_')
    for x0,x1,r in [(-26.5,-19.5,30.5),(19.5,26.5,30.5),(-19.5,19.5,15.5)]:
        volume=cq.Solid.makeCylinder(r,x1-x0,cq.Vector(x0,95,32),cq.Vector(1,0,0))
        for b in bodies:
            if b.name.startswith(ignored):continue
            v=overlap(volume,b.shape)
            if v>0.01:sweep.append({'part':b.name,'volume_mm3':round(v,3)})
    swing={}
    for side in ('L','R'):
        names=['Wedgelet_'+side]+['Wedgelet_Carrier_PETG_%s%d'%(side,n) for n in (1,2)]
        moving=[b for b in bodies if b.name in names]
        rows=[]
        # Every integer degree, not just a few sampled poses.
        for deg in range(-2,15):
            hits=[]
            for a in moving:
                moved=a.shape.rotate((0,102,14.5),(1,102,14.5),deg)
                for b in bodies:
                    if b.name in names or b.name.startswith(('Hinge_Pin_D3','Eclip_')): continue
                    v=overlap(moved,b.shape)
                    if v>0.01: hits.append({'moving':a.name,'fixed':b.name,'volume_mm3':round(v,3)})
            rows.append({'angle_deg':deg,'hits':hits})
        swing[side]=rows
    dimension_errors=[]
    for item in constraints['shaft_engagement']:
        if item['available_projection_mm']<item['needed_projection_mm']:
            dimension_errors.append('Short shaft: '+item['motor'])
    if constraints['highest_electronics_mm']>56:dimension_errors.append('Electronics violate 2 mm lid clearance')
    if constraints['lid_top_mm']>63.001:dimension_errors.append('Lid roof leaves less than 1 mm inverted wheel clearance')
    if constraints['hinge_wall_mm']<1.6:dimension_errors.append('Hinge wall below 1.6 mm')
    if not constraints['axle_nut_full_thread']:dimension_errors.append('M8 bolt does not engage the full locking nut plus 2 mm')
    if constraints['front_drive_to_weapon_motor_clearance_mm']<1:dimension_errors.append('Drive/weapon motors leave less than 1 mm clearance')
    if config.get('readiness_version') == 2:
        if constraints['weapon_shaft_engagement_mm']<10:dimension_errors.append('Weapon shaft engages less than 10 mm of pulley')
        if not constraints['weapon_set_screw_on_shaft']:dimension_errors.append('Weapon pulley set screw misses shaft')
    passed=not (bad or clashes or sweep or dimension_errors or any(r['hits'] for rows in swing.values() for r in rows) or any(r['algorithm_errors'] or r['self_interferences'] for r in self_checks))
    return {'passed':passed,'invalid_solids':bad,'self_interference':self_checks,'unexpected_interferences':clashes,'intentional_tyre_press_fit':expected,'rotor_sweep_hits':sweep,'wedgelet_swing':swing,'dimension_errors':dimension_errors}


def mesh_audit(path):
    mesh=trimesh.load_mesh(path,process=False)
    # Exact coordinate weld, deliberately no automatic repair.
    verts,inv=np.unique(mesh.vertices,axis=0,return_inverse=True)
    faces=inv[mesh.faces];tri=verts[faces]
    ed=np.vstack((faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]))
    canonical=np.sort(ed,axis=1)
    unique,indices,counts=np.unique(canonical,axis=0,return_inverse=True,return_counts=True)
    signs=np.where(ed[:,0]<ed[:,1],1,-1)
    signed_counts=np.bincount(indices,weights=signs,minlength=len(unique))
    parent=list(range(len(verts)))
    def find(a):
        while parent[a]!=a:parent[a]=parent[parent[a]];a=parent[a]
        return a
    for a,b in unique:parent[find(int(a))]=find(int(b))
    components=len({find(i) for i in range(len(verts))})
    twice_area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)
    welded=trimesh.Trimesh(vertices=verts,faces=faces,process=False)
    manifold=manifold3d.Manifold(manifold3d.Mesh(vert_properties=verts.astype(np.float32),tri_verts=faces.astype(np.uint32)))
    row={'file':Path(path).name,'sha256':sha(path),'triangles':len(faces),'boundary_edges':int((counts==1).sum()),'nonmanifold_edges':int((counts>2).sum()),'winding_errors':int((signed_counts!=0).sum()),'degenerate_faces':int((twice_area<1e-9).sum()),'duplicate_faces':len(faces)-len(np.unique(np.sort(faces,axis=1),axis=0)),'components':components,'positive_volume':bool(welded.volume>0),'manifold_status':str(manifold.status()),'dimensions_mm':np.ptp(verts,axis=0).round(3).tolist(),'min_z_mm':round(float(verts[:,2].min()),5)}
    row['passed']=all(row[k]==0 for k in ('boundary_edges','nonmanifold_edges','winding_errors','degenerate_faces','duplicate_faces')) and components==1 and row['positive_volume'] and manifold.status()==manifold3d.Error.NoError and abs(row['min_z_mm'])<0.001
    return row


def mass_properties(root,config,print_files):
    bought={}
    for loc in ('Front','Rear'):
        for side in ('L','R'): bought['JGA25_370_400rpm_'+loc+'_'+side]=('drive_'+loc+'_'+side,'mass_g')
    bought.update({'D3536_1250kV_Can_ENVELOPE':('weapon_motor','mass_g'),'3S_850mAh_LiPo_ENVELOPE_75x35x25':('battery','mass_g'),'Skywalker_40A_ESC_55x25x12':('esc','mass_g'),'ESP32_DevKit_51.5x28.3':('esp32','mass_g'),'DRV8871_L':('drv_left','mass_g'),'DRV8871_R':('drv_right','mass_g'),'IMU_GY521_MPU6050_21x16':('imu','mass_g'),'Power_Switch_Link':('disconnect','mass_g'),'Hall_A3144_TO92':('hall_sensor','mass_g')})
    density={'steel':0.00785,'6061':0.0027,'TPU95A':0.00121,'PETG':0.00127,'foam':0.00008}
    rows=[]
    axle_bodies=[b for b in root.bodies() if b.name.startswith(('Dead_Shaft','Weapon_Axle_'))]
    axle_volume=sum(b.shape.Volume() for b in axle_bodies)
    for b in root.bodies():
        if b.material=='keepout':continue
        center=list(b.shape.Center().toTuple());method='estimated CAD volume/density'
        if b.name in bought:
            part,key=bought[b.name];mass=value(config,part,key)
            item=config['hardware'][part]
            method='measured' if item['actual'][key] is not None else 'supplier published, physically unverified' if key in item.get('catalog',{}).get('values',{}) else 'nominal, unverified'
        elif b.name.startswith('Bearing_608'): mass=value(config,'bearing_608','mass_g_each');method='bearing measured/nominal'
        elif b.name.startswith(('Dead_Shaft','Weapon_Axle_')):mass=value(config,'dead_shaft','mass_g')*b.shape.Volume()/axle_volume;method='whole axle set measured/nominal, volume apportioned'
        elif b.name.startswith('Hinge_Pin'):mass=value(config,'hinge_pin','mass_g_pair')/2;method='pin measured/nominal'
        elif b.name.startswith('Eclip'):mass=value(config,'clips','mass_g')/2;method='clip measured/nominal'
        elif b.name.startswith('Belt_'):mass=value(config,'belt','mass_g')/2;method='belt measured/nominal'
        elif b.name=='Nose_Skid_UHMW_3mm':mass=value(config,'skid','mass_g');method='skid measured/nominal'
        else:
            fraction=0.85 if b.name in ('Tub_TPU','Lid_TPU_2mm') else 0.80 if b.material in ('PETG','TPU95A') else 1.0
            mass=b.shape.Volume()*density[b.material]*fraction
        if b.material in ('steel','6061','foam') and method=='estimated CAD volume/density' and b.name in config.get('manufacturing',{}).get('actual_mass_g_by_body',{}):
            mass=config['manufacturing']['actual_mass_g_by_body'][b.name];method='measured manufactured part'
        fname=print_files.get(b.name)
        if fname and fname in config['slicer']['printed_mass_g_by_file']:
            mass=config['slicer']['printed_mass_g_by_file'][fname];method='shop slicer'
        rows.append({'part':b.name,'material':b.material,'mass_g':round(mass,4),'centroid_mm':center,'method':method})
    # All unmodelled items remain explicit, not hidden in a tuned CoG constant.
    for name,key,center in [('rotor screws','rotor_screws',[0,95,32]),('original screws/inserts/washers','baseline_fasteners',[0,-5,25]),('R1 extra screws/nuts/washers','revision_fasteners',[0,-35,20]),('wiring/connectors','wiring',[0,-25,28]),('temperature sensors/magnets','aux_sensors',[0,0,30]),('straps/foam/insulation','straps',[0,-35,35])]:
        mass=value(config,key,'mass_g')
        measured=config['hardware'][key]['actual']['mass_g'] is not None
        rows.append({'part':name,'mass_g':mass,'centroid_mm':center,'method':'measured ancillary mass; layout centroid estimate' if measured else 'unverified allowance'})
    total=sum(r['mass_g'] for r in rows)
    center=sum(np.array(r['centroid_mm'])*r['mass_g'] for r in rows)/total
    front,rear=(config['design'][k] for k in ('front_axle_y_mm','rear_axle_y_mm'))
    rear_load=(front-center[1])/(front-rear)
    # Principal masses used in MuJoCo are separate; derive chassis CoG by subtraction.
    special=[r for r in rows if r['part'].startswith(('Wheel_','Beater_','Bearing_','Wedgelet_Carrier','Skirt_Plate')) or r['part'] in ('Wedgelet_L','Wedgelet_R','rotor screws')]
    sm=sum(r['mass_g'] for r in special)
    chassis_center=(center*total-sum(np.array(r['centroid_mm'])*r['mass_g'] for r in special))/(total-sm)
    chassis_rows=[r for r in rows if r not in special]
    tensor=np.zeros((3,3));body_map={b.name:b for b in root.bodies()}
    for row in chassis_rows:
        mass_kg=row['mass_g']/1000;delta=(np.array(row['centroid_mm'])-chassis_center)/1000
        b=body_map.get(row['part'])
        # Unit-density exact shape inertia, scaled to the measured/estimated mass.
        own=np.array(cq.Shape.matrixOfInertia(b.shape))*mass_kg/b.shape.Volume()/1e6 if b else np.zeros((3,3))
        tensor+=own+mass_kg*(np.dot(delta,delta)*np.eye(3)-np.outer(delta,delta))
    rotor_rows=[r for r in rows if r['part'].startswith(('Beater_','Bearing_')) or r['part']=='rotor screws']
    rotor_inertia=0.0
    for row in rotor_rows:
        b=body_map.get(row['part']);mass_kg=row['mass_g']/1000
        own=cq.Shape.matrixOfInertia(b.shape)[0][0]*mass_kg/b.shape.Volume()/1e6 if b else 0
        y,z=row['centroid_mm'][1:];rotor_inertia+=own+mass_kg*((y-95)**2+(z-32)**2)/1e6
    return {'rotor_inertia_kg_m2':rotor_inertia,'estimated':True,'total_g':round(total,2),'centroid_mm':center.round(4).tolist(),'rear_static_load_fraction':round(float(rear_load),5),'rear_load_passed':bool(rear_load>=config['design']['rear_load_min']),'mass_target_passed':bool(total<=config['design']['mass_target_g']),'mass_limit_passed':bool(total<=config['design']['mass_limit_g']),'chassis_centroid_mm':chassis_center.round(4).tolist(),'chassis_mass_g':round(total-sm,4),'chassis_inertia_kg_m2':tensor.tolist(),'parts':rows}


def evidence_file(path,base):
    if not isinstance(path,str) or not path.strip():return False
    p=Path(path);p=p if p.is_absolute() else Path(base)/p
    return p.is_file() and p.stat().st_size>0


def release_blockers(config,report,base=ROOT):
    blockers=[]
    for key,item in config['hardware'].items():
        if item['verified'] is not True or any(v is None for v in item['actual'].values()) or not evidence_file(item['evidence'],base): blockers.append('Unconfirmed hardware: '+key)
    sig=report['build_sha256']
    for key in (('fit','preflight') if config.get('readiness_version') == 2 else ('fit','assembly')):
        item=config[key]
        if not(item['verified'] is True and item['sha256_build']==sig and all(v is True for v in item['results'].values()) and evidence_file(item['evidence'],base)):
            blockers.append('Missing/current-build failed evidence: '+key)
    if any(config['fit'][key] is None for key in ('d_bore_diametral_clearance_mm','d_bore_flat_clearance_mm','hinge_diametral_clearance_mm','insert_hole_diameter_mm','tyre_inner_diameter_mm','nut_pocket_clearance_mm')):
        blockers.append('Calibrated fit dimensions missing')
    shop=config['shop']
    if not all(shop.get(k) for k in ('name','printer','usable_bed_mm','nozzle_mm','slicer')) or not all(shop['materials'].values()) or not evidence_file(shop['profile_evidence'],base): blockers.append('Shop/printer/material profile unconfirmed')
    bed=shop['usable_bed_mm'];brim=shop['brim_mm']
    if not(isinstance(bed,list) and len(bed)==3 and all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and v>0 for v in bed) and isinstance(brim,(int,float)) and not isinstance(brim,bool) and math.isfinite(brim) and brim>=0):
        blockers.append('Usable build volume/brim missing or invalid')
    else:
        for row in report['manifest']:
            x,y,z=row['dimensions_mm'];bx,by,bz=bed
            if z>bz or not((x+2*brim<=bx and y+2*brim<=by) or (y+2*brim<=bx and x+2*brim<=by)):
                blockers.append('Part plus brim exceeds printer: '+row['file'])
    slicer=config['slicer']
    required_files={r['file'] for r in report['manifest']}
    if not(slicer['verified'] is True and slicer['sha256_build']==sig and all(v is True for v in slicer['review'].values()) and evidence_file(slicer['project_file'],base) and evidence_file(slicer['report_file'],base) and required_files<=set(slicer['printed_mass_g_by_file'])): blockers.append('Slicer project/review/masses missing or stale')
    cost=config['cost']
    if not(cost['verified'] is True and evidence_file(cost['evidence'],base) and all(not isinstance(i['total_thb'],bool) and isinstance(i['total_thb'],(int,float)) and math.isfinite(i['total_thb']) and i['total_thb']>=0 for i in cost['items'])):
        blockers.append('Complete confirmed project cost missing')
    elif sum(i['total_thb'] for i in cost['items'])>config['design']['budget_thb']:blockers.append('Project cost exceeds 6000 THB')
    if not report['cad']['passed']:blockers.append('CAD interference/sweep/dimensions failed')
    if not all(r['passed'] for r in report['mesh']):blockers.append('Mesh audit failed')
    if not all(r['passed'] for r in report['trial_mesh']):blockers.append('Trial mesh audit failed')
    if not report['mass']['rear_load_passed']:blockers.append('Rear static wheel load below 10%')
    if not report['mass']['mass_limit_passed']:blockers.append('Robot mass exceeds 2000 g')
    sim=report.get('simulation',{})
    if not(sim.get('passed') is True and sim.get('build_sha256')==sig):blockers.append('Current revision dynamics verification missing/failed')
    if config.get('readiness_version') == 2:
        from readiness import additional_print_blockers
        blockers += additional_print_blockers(config, report, base)
    return blockers
