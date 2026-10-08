"""Run R1 static/spin/turn/inverted checks with CAD-derived mass and inertia.

Use the existing MuJoCo venv (no CAD dependencies needed):
  .venv/bin/python robot_v4/print_revision/simulate.py
Does not overwrite the archived V4 simulations.
Collision primitives remain reduced-order; this is not an FEA or slicer check.
"""
import json
import argparse
import math
import re
import hashlib
import sys
from pathlib import Path
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'mujoco_sim'))
import sim
import validate


def configure(report,config):
    old=sim.v4
    mass=report['mass'];rows={r['part']:r for r in mass['parts']}
    sim.V4_MASS=mass['total_g']/1000
    rotor_names=[n for n in rows if n.startswith(('Beater_','Bearing_')) or n=='rotor screws']
    sim.ROTOR_M=sum(rows[n]['mass_g'] for n in rotor_names)/1000
    front=config['design']['front_axle_y_mm']/1000
    rear=config['design']['rear_axle_y_mm']/1000
    cg=np.array(mass['chassis_centroid_mm'])/1000
    I=np.array(mass['chassis_inertia_kg_m2'])
    inertial='<inertial pos="%s" mass="%s" fullinertia="%s"/>' % (sim.f(*cg),sim.f(mass['chassis_mass_g']/1000),sim.f(I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]))
    def revised(*args):
        body,act,meta=old(*args);p=args[0]
        body=re.sub(r'<inertial[^>]+/>',lambda m:inertial,body,count=1)
        for n,(x,y) in enumerate([(-0.070,front),(0.070,front),(-0.070,rear),(0.070,rear)]):
            body=re.sub(r'(<body name="'+p+'_w'+str(n)+r'" pos=")[^"]+',lambda m:m.group(1)+sim.f(x,y,0.032),body)
            suffix=('Front_L','Front_R','Rear_L','Rear_R')[n]
            wm=(rows['Wheel_Hub_PETG_'+suffix]['mass_g']+rows['Wheel_Tyre_TPU_'+suffix]['mass_g'])/1000
            body=re.sub(r'(<geom name="'+p+'_w'+str(n)+r'_g"[^>]*mass=")[^"]+',lambda m:m.group(1)+sim.f(wm),body)
        for side in ('L','R'):
            wm=sum(rows[n]['mass_g'] for n in rows if n in ('Wedgelet_'+side,'Wedgelet_Carrier_PETG_'+side+'1','Wedgelet_Carrier_PETG_'+side+'2'))/1000
            body=re.sub(r'(<geom name="'+p+'_wedge'+side+r'_g"[^>]*mass=")[^"]+',lambda m:m.group(1)+sim.f(wm),body)
        for suffix,name in [('L','Skirt_Plate_08mm_L'),('R','Skirt_Plate_08mm_R'),('B','Skirt_Plate_08mm_Rear')]:
            body=re.sub(r'(<geom name="'+p+'_skirt'+suffix+r'_g"[^>]*mass=")[^"]+',lambda m:m.group(1)+sim.f(rows[name]['mass_g']/1000),body)
        roof=sim.box(p+'_lid_roof',-0.0415,-0.0045,0.032,0.060,0.058,report['constraints']['lid_top_mm']/1000,'mass="0" '+args[-1])
        index=body.index('<body name="'+p+'_w0"')
        body=body[:index]+roof+'\n'+body[index:]
        return body,act,meta
    sim.v4=revised
    sim.BUILDERS['V4']=revised


def wheel_loads(m,d):
    floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'floor')
    wheels={mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'A_w%d_g'%i):i for i in range(4)}
    loads=np.zeros(4);force=np.zeros(6)
    for i in range(d.ncon):
        contact=d.contact[i]
        if floor not in (contact.geom1,contact.geom2):continue
        g=contact.geom2 if contact.geom1==floor else contact.geom1
        if g not in wheels:continue
        mujoco.mj_contactForce(m,d,i,force)
        loads[wheels[g]]+=abs(float(contact.frame[2]*force[0]))
    return loads


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--inputs',type=Path,default=HERE/'inputs.json');parser.add_argument('--output',type=Path,default=HERE/'output')
    args=parser.parse_args()
    output=args.output;report=json.loads((output/'build_report.json').read_text());config=json.loads(args.inputs.read_text())
    geometry={'hardware':config['hardware'],'design':config['design'],'fit':{k:v for k,v in config['fit'].items() if k not in ('verified','evidence','sha256_build','results')}}
    if hashlib.sha256(json.dumps(geometry,sort_keys=True).encode()).hexdigest()!=report['input_sha256']:
        raise RuntimeError('Inputs changed after CAD build; run build.py before simulating')
    configure(report,config)
    result={'build_sha256':report['build_sha256'],'model':'MuJoCo primitives + CAD mass/centroid/inertia, unverified purchased masses','limitations':['Rigid-body contacts do not predict breakage','Collision roof is conservative box envelope','No physical-fit or printer calibration evidence']}
    m,d,r=validate.make('V4')
    # Confirm total mass/CG agrees with the CAD accounting before dynamics.
    expected=np.array(report['mass']['centroid_mm'])/1000
    initial=d.subtree_com[r.body]-d.xpos[r.body]
    result['mass_kg']=float(m.body_subtreemass[r.body])
    result['cad_mass_kg']=report['mass']['total_g']/1000
    result['initial_cg_mm']=(initial*1000).round(4).tolist()
    result['cg_error_mm']=float(np.linalg.norm(initial-expected)*1000)
    loads=[]
    def rest(t):
        r.wheel_cmd(0,0)
        if t>0.5:loads.append(wheel_loads(m,d))
    validate.run(m,d,0.8,rest)
    mean=np.mean(loads,axis=0);weight=result['mass_kg']*sim.G
    result['rest_wheel_loads_N']=mean.round(4).tolist()
    result['rear_load_fraction']=float(mean[2:].sum()/weight)
    result['rest_wheel_weight_fraction']=float(mean.sum()/weight)
    m,d,r=validate.make('V4');minimum=[1.0];floor_hits=[0]
    tooth_ids=[i for i in range(m.ngeom) if (mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith('A_tooth')]
    floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'floor')
    def spin(t):
        r.rotor_step(validate.CTRL);r.wheel_cmd(0,0)
        if t>0.2:
            minimum[0]=min(minimum[0],min(validate.geom_min_z(m,d,mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i)) for i in tooth_ids))
            floor_hits[0]+=sum(floor in (d.contact[i].geom1,d.contact[i].geom2) and any(g in tooth_ids for g in (d.contact[i].geom1,d.contact[i].geom2)) for i in range(d.ncon))
    validate.run(m,d,2.5,spin)
    result['spinup']={'rotor_rpm':r.rotor_rpm(),'minimum_tooth_floor_gap_mm':minimum[0]*1000,'tooth_floor_contacts':floor_hits[0]}
    peak=[0.0];finite=[True]
    def turn(t):
        r.rotor_step(validate.CTRL);r.wheel_cmd(-1,1)
        peak[0]=max(peak[0],abs(float(d.cvel[r.body][2])))
        finite[0]=finite[0] and bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
    validate.run(m,d,1.5,turn)
    result['turn']={'peak_yaw_rad_s':peak[0],'finite_state':finite[0]}
    m,d,r=validate.make('V4',inverted=True)
    validate.run(m,d,0.3,lambda t:r.wheel_cmd(0,0));start=d.xpos[r.body].copy()
    validate.run(m,d,1.5,lambda t:r.wheel_cmd(-1,-1))
    result['inverted_drive_m']=float(np.linalg.norm((d.xpos[r.body]-start)[:2]))
    result['checks']={'mass_matches':abs(result['mass_kg']-result['cad_mass_kg'])<0.0001,'cg_matches':result['cg_error_mm']<0.5,'rear_load_minimum':result['rear_load_fraction']>=config['design']['rear_load_min'],'wheels_carry_robot':result['rest_wheel_weight_fraction']>0.95,'no_tooth_floor_contact':floor_hits[0]==0 and minimum[0]>0,'turns':peak[0]>0.5 and finite[0],'inverted_drive':result['inverted_drive_m']>0.5}
    result['passed']=all(result['checks'].values())
    (output/'simulation.json').write_text(json.dumps(result,indent=2)+'\n')
    report['simulation']=result
    # Rebuild writes the final aggregate report and checks the current signature.
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
