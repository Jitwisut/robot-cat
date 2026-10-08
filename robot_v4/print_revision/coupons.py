"""Small labelled fit coupons; no calibrated dimension is implied by export."""
import math
from pathlib import Path
import cadquery as cq
from cad_backend import block, prism
from legacy_geometry import d_bore
from revision import value


def coupons(config):
    result=[]
    shaft=value(config,'drive_Front_L','shaft_diameter_mm')
    flat=shaft/2-value(config,'drive_Front_L','shaft_flat_depth_mm')
    pin=value(config,'hinge_pin','diameter_mm')
    insert=value(config,'insert_m3','outer_diameter_mm')
    insert_trial=config['design'].get('insert_trial_hole_diameter_mm',insert)
    for clearance in (0.0,0.1,0.2,0.3):
        shape=prism([('poly',[(-5,-5),(5,-5),(5,5),(-5,5)]),('poly',d_bore(0,0,(shaft+clearance)/2,flat+clearance/2,144))],'x',0,6)
        shape=shape.rotate((0,0,0),(0,1,0),-90)
        result.append(('D_Shaft_Clearance_%03d_PETG.stl'%round(clearance*100),'PETG',shape))
        radius=(pin+clearance)/2
        shape=block(-6,6,-6,6,0,8).cut(cq.Solid.makeCylinder(radius,10,cq.Vector(0,0,-1)))
        result.append(('Pin_Vertical_Clearance_%03d_PETG.stl'%round(clearance*100),'PETG',shape))
        horizontal=block(0,8,-6,6,0,12).cut(cq.Solid.makeCylinder(radius,10,cq.Vector(-1,0,6),cq.Vector(1,0,0)))
        result.append(('Pin_Horizontal_Clearance_%03d_PETG.stl'%round(clearance*100),'PETG',horizontal))
    for delta in (-0.2,-0.1,0.0,0.1,0.2):
        d=insert_trial+delta
        for material in ('PETG','TPU95A'):
            shape=block(-6,6,-6,6,0,value(config,'insert_m3','length_mm')+2)
            shape=shape.cut(cq.Solid.makeCylinder(d/2,value(config,'insert_m3','length_mm')+1,cq.Vector(0,0,1)))
            result.append(('Insert_Hole_D%.1f_%s.stl'%(d,material),material,shape))
        horizontal=block(0,10,-6,6,0,8).cut(cq.Solid.makeCylinder(d/2,value(config,'insert_m3','length_mm')+1,cq.Vector(10,0,4),cq.Vector(-1,0,0)))
        result.append(('Insert_Horizontal_D%.1f_PETG.stl'%d,'PETG',horizontal))
    for inner in (43.4,43.6,43.8,44.0):
        shape=cq.Solid.makeCylinder(32,4).cut(cq.Solid.makeCylinder(inner/2,4))
        result.append(('Tyre_Ring_ID%.1f_TPU95A.stl'%inner,'TPU95A',shape))
    # A representative TPU lid wall and lid are separate pieces to test insert retention.
    wall=block(0,20,0,8,0,12).cut(cq.Solid.makeCylinder(insert_trial/2,value(config,'insert_m3','length_mm')+1,cq.Vector(10,4,12),cq.Vector(0,0,-1)))
    lid=block(0,20,0,8,0,2).cut(cq.Solid.makeCylinder(1.7,3,cq.Vector(10,4,-0.5)))
    result.extend([('Lid_Insert_Wall_TPU95A.stl','TPU95A',wall),('Lid_Fastener_TPU95A.stl','TPU95A',lid)])
    return result


def export_trials(config,destination,trial_parts):
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    rows=[]
    all_parts=coupons(config)+trial_parts
    for name,material,shape in all_parts:
        bb=shape.BoundingBox();shape=shape.translate((-bb.xmin,-bb.ymin,-bb.zmin))
        shape.exportStl(str(destination/name),tolerance=0.02,angularTolerance=0.05,relative=False)
        rows.append({'file':name,'material':material,'quantity':1,'status':'TRIAL_ONLY'})
    filenames={r['file'] for r in rows}
    for stale in destination.glob('*.stl'):
        if stale.name not in filenames:stale.unlink()
    return rows
