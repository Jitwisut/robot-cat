"""Swing each hinged wedgelet (plate + its 2 carriers) about its pin and report interference.

Run in Fusion on the document built by build_robot_v4.py. Positive angle = tip up.
Uses temporary BRep copies, so the model is not changed.
"""
import adsk.core, adsk.fusion, json, math

PIN_Y, PIN_Z = 102.0, 14.5          # conceptual mm (x right, y forward, z up)
ANGLES = [-2, 2, 5, 8, 10, 12, 14, 16]


def run(_):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    bodies = [b for occ in root.allOccurrences for b in occ.bRepBodies]
    tbm = adsk.fusion.TemporaryBRepManager.get()
    # conceptual (x, y, z) -> Fusion (x, z, -y), cm
    pivot = adsk.core.Point3D.create(0, PIN_Z / 10, -PIN_Y / 10)
    axis = adsk.core.Vector3D.create(1, 0, 0)
    out = {}
    for side in ('L', 'R'):
        moving = [b for b in bodies if b.name in ('Wedgelet_' + side, 'Wedgelet_Carrier_PETG_%s1' % side,
                                                    'Wedgelet_Carrier_PETG_%s2' % side)]
        others = [b for b in bodies if b not in moving and not b.name.startswith('Hinge_Pin_D3')]
        rows = []
        for deg in ANGLES:
            m = adsk.core.Matrix3D.create()
            m.setToRotation(math.radians(deg), axis, pivot)
            hits = []
            tip_z = None
            for mb in moving:
                t = tbm.copy(mb)
                tbm.transform(t, m)
                if mb.name.startswith('Wedgelet_' + side) and not 'Carrier' in mb.name:
                    tip_z = round(t.boundingBox.minPoint.y * 10, 2)     # lowest point = tip underside (mm)
                for ob in others:
                    if not t.boundingBox.intersects(ob.boundingBox):
                        continue
                    i = tbm.copy(t)
                    tbm.booleanOperation(i, tbm.copy(ob), adsk.fusion.BooleanTypes.IntersectionBooleanType)
                    if i.volume * 1000 > 0.01:
                        hits.append([mb.name, ob.name, round(i.volume * 1000, 2)])
            rows.append({'deg': deg, 'lowest_point_z_mm': tip_z, 'hits': hits})
        out[side] = rows
    print(json.dumps(out))
