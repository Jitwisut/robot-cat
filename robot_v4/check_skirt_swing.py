"""Swing each hinged skirt (plate + its clips) about its wire and report interference.

Run in Fusion on the document built by build_robot_v4.py. Positive angle = bottom edge swings outward.
The clip meeting the wall is the inward stop (~1 deg), so small negative angles should touch the tub.
"""
import adsk.core, adsk.fusion, json, math

ANGLES = [-3, -1.5, -1, -0.5, 2, 10, 25, 40]
# conceptual (x, y, z) -> Fusion (x, z, -y), cm
SKIRTS = {
    'R': ('Skirt_Plate_08mm_R', 'Skirt_Clip_PETG_R', (8.9, 1.5, 0.0), (0, 0, 1), +1),
    'L': ('Skirt_Plate_08mm_L', 'Skirt_Clip_PETG_L', (-8.9, 1.5, 0.0), (0, 0, 1), -1),
    'Rear': ('Skirt_Plate_08mm_Rear', 'Skirt_Clip_PETG_Rear', (0.0, 1.5, 11.1), (1, 0, 0), +1),
}


def run(_):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    bodies = [b for occ in design.rootComponent.allOccurrences for b in occ.bRepBodies]
    tbm = adsk.fusion.TemporaryBRepManager.get()
    out = {}
    for key, (plate, clip, piv, ax, sgn) in SKIRTS.items():
        moving = [b for b in bodies if b.name == plate or
                  (b.name.startswith(clip) and b.name[len(clip):].isdigit())]   # 'R' must not match 'Rear'
        others = [b for b in bodies if b not in moving and not b.name.startswith('Skirt_Wire')]
        pivot = adsk.core.Point3D.create(*piv)
        axis = adsk.core.Vector3D.create(*ax)
        rows = []
        for deg in ANGLES:
            # sign so that +deg moves the bottom edge away from the robot
            m = adsk.core.Matrix3D.create()
            m.setToRotation(math.radians(deg) * (sgn if key != 'Rear' else -1), axis, pivot)
            hits = []
            for mb in moving:
                t = tbm.copy(mb)
                tbm.transform(t, m)
                for ob in others:
                    if not t.boundingBox.intersects(ob.boundingBox):
                        continue
                    i = tbm.copy(t)
                    tbm.booleanOperation(i, tbm.copy(ob), adsk.fusion.BooleanTypes.IntersectionBooleanType)
                    if i.volume * 1000 > 0.01:
                        hits.append([mb.name, ob.name, round(i.volume * 1000, 2)])
            plate_t = tbm.copy([b for b in moving if b.name == plate][0])
            tbm.transform(plate_t, m)
            bb = plate_t.boundingBox
            rows.append({'deg': deg, 'plate_x_mm': [round(bb.minPoint.x * 10, 1), round(bb.maxPoint.x * 10, 1)],
                         'plate_y_fusion_z_mm': [round(bb.minPoint.z * 10, 1), round(bb.maxPoint.z * 10, 1)],
                         'lowest_z_mm': round(bb.minPoint.y * 10, 2), 'hits': hits})
        out[key] = rows
    print(json.dumps(out))
