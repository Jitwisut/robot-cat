"""Build draft CAD, print-oriented meshes, trials and the release audit.

Run from the repository: .venv-v4-print/bin/python robot_v4/print_revision/build.py
No final files are released without physical and shop evidence.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path
import cadquery as cq
from revision import build
from checks import cad_checks, mesh_audit, mass_properties, build_signature, input_signature, release_blockers, sha
from coupons import export_trials

HERE=Path(__file__).resolve().parent


def print_shape(body):
    shape=body.shape
    if body.name.startswith(('Wheel_','Wedgelet_Carrier','Hall_Post')):
        shape=shape.rotate((0,0,0),(0,1,0),-90)
    bb=shape.BoundingBox()
    return shape.translate((-bb.xmin,-bb.ymin,-bb.zmin))


def export_prints(root,directory,prefix='V4_R1'):
    directory.mkdir(parents=True,exist_ok=True)
    rows=[];body_files={}
    # Reuse only identical parts; left/right carriers and hinges remain explicit.
    seen={}
    for b in root.bodies():
        if b.material not in ('TPU95A','PETG') or b.name=='Nose_Skid_UHMW_3mm':continue
        key=b.name
        if key.startswith('Wheel_Hub'):key='Wheel_Hub_PETG'
        elif key.startswith('Wheel_Tyre'):key='Wheel_Tyre_TPU95A'
        elif key.startswith('Electronics_Standoff'):key='Electronics_Standoff_PETG'
        # Motors may have different shafts: do not deduplicate unequal geometry.
        oriented=print_shape(b)
        fingerprint=sha_bytes(oriented)
        pair=(key,fingerprint)
        if pair in seen:
            row=seen[pair];row['quantity']+=1;row['bodies'].append(b.name)
        else:
            count=sum(1 for k in seen if k[0]==key)
            filename=prefix+'_'+key+('_variant%d'%count if count else '')+'.stl'
            oriented.exportStl(str(directory/filename),tolerance=0.02,angularTolerance=0.05,relative=False)
            bb=oriented.BoundingBox()
            row={'file':filename,'quantity':1,'material':b.material,'bodies':[b.name],'dimensions_mm':[round(v,3) for v in (bb.xlen,bb.ylen,bb.zlen)],'sha256':sha(directory/filename),'orientation':'Z-up, bed Z=0','status':'DRAFT_NOT_RELEASED'}
            rows.append(row);seen[pair]=row
        body_files[b.name]=row['file']
    filenames={r['file'] for r in rows}
    for stale in directory.glob('*.stl'):
        if stale.name not in filenames:stale.unlink()
    return rows,body_files


def sha_bytes(shape):
    # Geometric signature, not unstable BREP serialization or face order.
    bb=shape.BoundingBox()
    return tuple(round(v,5) for v in (shape.Volume(),bb.xlen,bb.ylen,bb.zlen,shape.Center().x,shape.Center().y,shape.Center().z))


def export_metal(root,directory):
    directory.mkdir(parents=True,exist_ok=True)
    # Modified top brace now has a belt relief: it no longer shares the bottom DXF.
    prefixes=('Beater_Disc_L','Upright_6061_6mm_L','Upright_6061_6mm_R','Wedgelet_L','Skirt_Plate_08mm_R','Skirt_Plate_08mm_Rear','Weapon_Top_Brace','Weapon_Bottom_Brace','Weapon_Motor_Mount')
    for b in root.bodies():
        if b.name.startswith(('Beater_Hub','Motor_Pulley','Spacer_')):
            b.shape.exportStep(str(directory/(b.name+'.step')))
        if not b.name.startswith(prefixes):continue
        b.shape.exportStep(str(directory/(b.name+'.step')))
        faces=[f for f in b.shape.Faces() if f.geomType()=='PLANE']
        # Most wires first so the outline includes all through holes.
        face=max(faces,key=lambda f:(len(f.Wires()),f.Area()))
        n=face.normalAt();origin=face.Center()
        location=cq.Plane(origin=origin,normal=n).location.inverse
        planar=face.moved(location)
        cq.exporters.export(cq.Workplane('XY').add(planar.Wires()),str(directory/(b.name+'.dxf')))


def write_status(report,path):
    mass=report['mass'];cad=report['cad']
    lines=['# '+report['revision']+' — DRAFT / ยังไม่ปล่อยผลิต','',
           'ไฟล์ชุดนี้มีค่าขนาดต้นแบบที่ยังไม่วัดจริง ห้ามส่งพิมพ์ชุดใหญ่จากสถานะนี้',
           '',f"Build SHA256: `{report['build_sha256']}`",'',
           f"CAD ผ่าน: {cad['passed']} · mesh ผ่าน: {all(r['passed'] for r in report['mesh'])}",
           f"มวลประมาณ: {mass['total_g']} g · โหลดล้อหลังประมาณ: {100*mass['rear_static_load_fraction']:.1f}%",'',
           '## งานที่ต้องยืนยันก่อนปล่อยชุดผลิต','']
    warnings=[]
    if not mass['mass_target_passed']:warnings.append(f"มวลประมาณยังเกินเป้าหมาย 1900 g; เหลือเผื่อถึง 2000 g เพียง {2000-mass['total_g']:.2f} g ต้องแทนด้วยน้ำหนัก slicer และอะไหล่จริงก่อนตัดสินใจ")
    if mass['rear_static_load_fraction']<0.15:warnings.append('โหลดล้อหลังเชิงสถิตผ่านขั้นต่ำ 10% แต่ยังไม่ถึงเป้าหมาย 15%')
    simulation=report.get('simulation',{})
    if simulation.get('build_sha256')==report['build_sha256']:
        lines[8:8]=[f"MuJoCo ผ่าน: {simulation.get('passed')} · โหลดล้อหลังจำลอง: {100*simulation.get('rear_load_fraction',0):.1f}% · ระยะฟันต่ำสุดตอนเร่งใบ: {simulation.get('spinup',{}).get('minimum_tooth_floor_gap_mm',0):.2f} mm",'']
    if warnings:lines+=['- '+w for w in warnings]
    lines+=['- '+reason for reason in report['release_blockers']]
    lines+=['','## ชิ้นพิมพ์ต้นแบบ','', '| ไฟล์ | วัสดุ | จำนวน | ขนาด XYZ มม. |','|---|---|---:|---|']
    lines+=['| '+r['file']+' | '+r['material']+' | '+str(r['quantity'])+' | '+' × '.join(map(str,r['dimensions_mm']))+' |' for r in report['manifest']]
    path.write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--inputs',type=Path,default=HERE/'inputs.json');parser.add_argument('--output',type=Path,default=HERE/'output');parser.add_argument('--release',action='store_true')
    args=parser.parse_args();config=json.loads(args.inputs.read_text())
    from inputs import validate_inputs
    validate_inputs(config)
    output=args.output;draft=output/'DRAFT';draft.mkdir(parents=True,exist_ok=True)
    root,constraints=build(config)
    assembly=cq.Assembly(name=config['revision'])
    for b in root.bodies():
        if b.material!='keepout':assembly.add(b.shape,name=b.name)
    assembly.save(str(draft/('ROBOT_'+config['revision']+'.step')))
    print('CAD built; exporting and checking',flush=True)
    manifest,body_files=export_prints(root,draft/'STL',config['revision'].replace('_PRINT_','_'))
    export_metal(root,draft/'metal')
    trial_names=('Wheel_Hub_PETG_Front_L','Wheel_Tyre_TPU_Front_L','Wedgelet_Block_PETG_R','Wedgelet_Carrier_PETG_R1','Wedgelet_Carrier_PETG_R2')
    trials=[('Trial_'+b.name+'.stl',b.material,print_shape(b)) for b in root.bodies() if b.name in trial_names]
    trial_manifest=export_trials(config,output/'TRIAL_ONLY',trials)
    report={'schema_version':1,'revision':config['revision'],'input_sha256':input_signature(config),'build_sha256':build_signature(config),'status':'DRAFT_NOT_RELEASED','constraints':constraints,'manifest':manifest,'trial_manifest':trial_manifest,
            'cad':cad_checks(root,config,constraints),'mesh':[mesh_audit(draft/'STL'/r['file']) for r in manifest],
            'trial_mesh':[mesh_audit(output/'TRIAL_ONLY'/r['file']) for r in trial_manifest],
            'mass':mass_properties(root,config,body_files)}
    sim=output/'simulation.json'
    if sim.exists():report['simulation']=json.loads(sim.read_text())
    report['release_blockers']=release_blockers(config,report,args.inputs.parent)
    (output/'build_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    write_status(report,output/'STATUS.md')
    if args.release:
        if report['release_blockers']:
            print('RELEASE REFUSED: '+ '; '.join(report['release_blockers']),file=sys.stderr);return 2
        final=output/'RELEASED'
        if final.exists():raise RuntimeError('Do not overwrite an existing release; use a new revision')
        shutil.copytree(draft,final)
        report['status']='RELEASED'
        for row in report['manifest']:row['status']='RELEASED'
        (final/'manifest.json').write_text(json.dumps(report['manifest'],ensure_ascii=False,indent=2)+'\n')
        shutil.copy2(args.inputs,final/'verified_inputs.json')
        for name in ('ASSEMBLY.md','MACHINING.md','SHOP_HANDOFF.md','FIT_TEST.md'):
            document=args.inputs.parent/name
            if not document.exists():document=HERE/name
            text=document.read_text().replace('output_R2/DRAFT/metal','metal').replace('output_R2/manifest.json','manifest.json').replace('output_R2/TRIAL_ONLY','../TRIAL_ONLY').replace('output/DRAFT/metal','metal').replace('output/manifest.json','manifest.json').replace('output/TRIAL_ONLY','../TRIAL_ONLY')
            (final/name).write_text(text)
        for name in ('THAI_PARTS_RESEARCH.md','sources.json','hardware_source_map.json'):
            if (args.inputs.parent/name).exists():shutil.copy2(args.inputs.parent/name,final/name)
        (final/'README.md').write_text('# '+config['revision']+' — RELEASED FOR PRINT/ASSEMBLY\n\nBuild SHA256: `'+report['build_sha256']+'`\n\nApproved measurements, physical fit/assembly evidence, shop profile/slicer evidence, budget and dynamics checks passed. Use manifest.json for quantities/materials. This is a manufacturing release, not a combat or firmware certification. Supporting preparation documents retain their trial-stage wording.\n')
        (final/'release_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'status':report['status'],'CAD_passed':report['cad']['passed'],'mesh_passed':all(r['passed'] for r in report['mesh']),'trials_passed':all(r['passed'] for r in report['trial_mesh']),'mass_g':report['mass']['total_g'],'rear_load_fraction':report['mass']['rear_static_load_fraction'],'blocker_count':len(report['release_blockers'])},indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
