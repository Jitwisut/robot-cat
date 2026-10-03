"""Create the V4 vertical-beater CAD study in Autodesk Fusion and export F3D/STEP.

Concept: invertible 4WD printed chassis (TPU), front dual-disc steel beater
on a dead M8 shaft between two 6 mm 6061 uprights, belt-driven 1:1 by a
D3536 outrunner, 2 mm steel wedgelets. Sizes come from design_v4.py.
Conceptual axes: x right, y forward (nose +Y), z up, mm, floor z = 0.
Bought parts are envelopes (ABS proxy), not supplier CAD.
"""
import adsk.core
import adsk.fusion
import json
import math
import os

OUT = '/Users/jitwisutthobut/Desktop/robot/robot_v4'
AXLE_Z = 32.0
ROTOR_Y = 95.0
MOTOR_Y = 44.0
WHEEL_R = 32.0
WHEEL_Y = (10.0, -70.0)          # wheelbase 80 mm
BASE_R, TIP_R, TOOTH_DEG = 23.0, 29.5, 60.0
# support block under each wedgelet: its top edge is the wedgelet's lower surface
WEDGE_SUPPORT = [(100, 4), (100, 20.5), (106, 20.5 - 6 * 20 / 28), (106, 4)]
BLOCK_WALL_SCREWS = [(80, 13.5), (92, 13.5)]                 # (y, z) side wall -> PETG block
LID_SCREWS_Y = (-95, -15, 55)
UPRIGHT_SCREWS = [(70, 14), (70, 50), (90, 14), (90, 46)]     # (y, z) through the uprights
MFG = os.path.join(OUT, 'manufacturing')


def cm(mm):
    return mm / 10.0


def point(sketch, x, y, z):
    # Conceptual Z-up (x, length, height) -> Fusion Y-up (x, height, -length).
    return sketch.modelToSketchSpace(adsk.core.Point3D.create(cm(x), cm(z), cm(-y)))


def material(app, exact):
    for library in app.materialLibraries:
        if library.name == 'Fusion Material Library':
            for item in library.materials:
                if item.name == exact:
                    return item
    raise RuntimeError('Missing Fusion material: ' + exact)


def part(parent, name):
    occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


group = part
NEW_BODY = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def extent(inp, distance_mm):
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(
        adsk.core.ValueInput.createByReal(cm(distance_mm))), adsk.fusion.ExtentDirections.PositiveExtentDirection)


def box(parent, name, x0, x1, y0, y1, z0, z1, mat):
    comp = part(parent, name)
    sketch = comp.sketches.add(comp.xZConstructionPlane)
    sketch.sketchCurves.sketchLines.addTwoPointRectangle(point(sketch, x0, y0, 0), point(sketch, x1, y1, 0))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), NEW_BODY)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(z0)))
    extent(inp, z1 - z0)
    body = extrudes.add(inp).bodies.item(0)
    body.name = name
    body.material = mat
    sketch.isVisible = False
    return comp


def extrude_yz(comp, name, loops, x0, x1, operation, body=None):
    """Extrude closed (y, z) polygons/circles along world X from x0 to x1.

    loops: list of ('poly', [(y, z), ...]) or ('circle', (y, z, r)).
    The profile with the largest area is used (outer minus inner loops).
    """
    sketch = comp.sketches.add(comp.yZConstructionPlane)
    sketch.name = name
    for kind, data in loops:
        if kind == 'poly':
            lines = sketch.sketchCurves.sketchLines
            for i, (y, z) in enumerate(data):
                y2, z2 = data[(i + 1) % len(data)]
                lines.addByTwoPoints(point(sketch, 0, y, z), point(sketch, 0, y2, z2))
        else:
            y, z, r = data
            sketch.sketchCurves.sketchCircles.addByCenterRadius(point(sketch, 0, y, z), cm(r))
    # Prefer the profile with the most loops (ring = outer minus bore), then the largest area.
    profs = [sketch.profiles.item(i) for i in range(sketch.profiles.count)]
    most = max(p.profileLoops.count for p in profs)
    best_profile = max((p for p in profs if p.profileLoops.count == most), key=lambda p: p.areaProperties().area)
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(best_profile, operation)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(x0)))
    extent(inp, x1 - x0)
    if body is not None:
        inp.participantBodies = [body]
    feature = extrudes.add(inp)
    sketch.isVisible = False
    return feature


def solid_x(parent, name, loops, x0, x1, mat):
    comp = part(parent, name)
    body = extrude_yz(comp, name, loops, x0, x1, NEW_BODY).bodies.item(0)
    body.name = name
    body.material = mat
    return comp, body


def cyl_x(parent, name, x0, x1, y, z, r, mat, bore=None):
    loops = [('circle', (y, z, r))]
    if bore:
        loops.append(('circle', (y, z, bore)))
    return solid_x(parent, name, loops, x0, x1, mat)


def cut_box(comp, body, name, x0, x1, y0, y1, z0, z1):
    extrude_yz(comp, name, [('poly', [(y0, z0), (y0, z1), (y1, z1), (y1, z0)])], x0, x1, CUT, body)


def join_box(comp, body, name, x0, x1, y0, y1, z0, z1):
    extrude_yz(comp, name, [('poly', [(y0, z0), (y0, z1), (y1, z1), (y1, z0)])], x0, x1, JOIN, body)


def cut_z(comp, body, name, x, y, r, z0, z1):
    """Vertical round hole (axis along conceptual z) from z0 to z1."""
    sketch = comp.sketches.add(comp.xZConstructionPlane)
    sketch.name = name
    sketch.sketchCurves.sketchCircles.addByCenterRadius(point(sketch, x, y, 0), cm(r))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), CUT)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(z0)))
    extent(inp, z1 - z0)
    inp.participantBodies = [body]
    extrudes.add(inp)
    sketch.isVisible = False


def plate_face(body, sign):
    """Largest planar face of a wedgelet whose normal points up (+1) or down (-1) the ramp."""
    n = adsk.core.Vector3D.create(0, 0.814 * sign, -0.581 * sign)     # conceptual (0, .581, .814) in Fusion axes
    best = None
    for f in body.faces:
        if f.geometry.objectType != 'adsk::core::Plane':
            continue
        ok, normal = f.evaluator.getNormalAtPoint(f.pointOnFace)
        if normal.dotProduct(n) > 0.99 and (best is None or f.area > best.area):
            best = f
    if best is None:
        raise RuntimeError('wedgelet face not found')
    return best


def face_holes(comp, body, name, centres, r, depth):
    """Holes into `body` normal to its up-ramp face (wedgelet top, or the support-block top under it)."""
    face = plate_face(body, +1)
    sketch = comp.sketches.add(face)
    sketch.name = name
    for x, y, z in centres:
        sketch.sketchCurves.sketchCircles.addByCenterRadius(point(sketch, x, y, z), cm(r))
    # Fusion may auto-project the face outline into the sketch; keep only the hole circles.
    hole_area = math.pi * cm(r) ** 2
    profiles = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        pr = sketch.profiles.item(i)
        if abs(pr.areaProperties().area - hole_area) < 0.05 * hole_area:
            profiles.add(pr)
    if profiles.count != len(centres):
        raise RuntimeError('%s: expected %d hole profiles, got %d' % (name, len(centres), profiles.count))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(profiles, CUT)
    # Symmetric about the face so the hole always goes into `body` whatever the sketch normal is;
    # only `body` is cut, so the half that leaves the material removes nothing.
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(depth)), False)   # depth on EACH side
    inp.participantBodies = [body]
    extrudes.add(inp)
    sketch.isVisible = False


def d_bore(y, z, r=2.05, flat=1.55, n=36):
    """4 mm D-shaft bore (flat 0.5 mm deep) as a polygon in the (y, z) plane."""
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n
        dy, dz = r * math.cos(a), r * math.sin(a)
        pts.append((y + min(dy, flat), z + dz))
    return pts


def tooth_profile(n_seg=72):
    """Disc outline: base circle with two 60 deg teeth to TIP_R, teeth at 0/180 deg."""
    pts = []
    for i in range(n_seg):
        a = 2 * math.pi * i / n_seg
        d = min(abs(math.atan2(math.sin(a - c), math.cos(a - c))) for c in (0.0, math.pi))
        r = TIP_R if d < math.radians(TOOTH_DEG / 2) else BASE_R
        pts.append((ROTOR_Y + r * math.cos(a), AXLE_Z + r * math.sin(a)))
    return pts


def largest_plane(body):
    return max((f for f in body.faces if f.geometry.objectType == 'adsk::core::Plane'), key=lambda f: f.area)


def find_body(root, name):
    for occ in root.allOccurrences:
        for b in occ.bRepBodies:
            if b.name == name:
                return b
    raise RuntimeError('body not found: ' + name)


# flat parts -> DXF (outline of the largest face, true size), printed parts -> STL
DXF_PARTS = [
    ('Beater_Disc_L', 'V4_Beater_Disc_6mm_steel_x2.dxf'),
    ('Upright_6061_6mm_L', 'V4_Upright_6mm_6061_x2.dxf'),
    ('Wedgelet_L', 'V4_Wedgelet_2mm_steel_x2_mirror.dxf'),
    ('Weapon_Top_Brace_3mm', 'V4_Brace_3mm_6061_x2.dxf'),
    ('Weapon_Motor_Mount_3mm', 'V4_Weapon_Motor_Mount_3mm_6061_x1.dxf'),
]
STL_PARTS = [
    ('Tub_TPU', 'V4_Tub_TPU95A_x1.stl'),
    ('Lid_TPU_3mm', 'V4_Lid_TPU95A_x1.stl'),
    ('Wheel_Hub_PETG_Front_L', 'V4_Wheel_Hub_PETG_x4.stl'),
    ('Wheel_Tyre_TPU_Front_L', 'V4_Wheel_Tyre_TPU95A_x4.stl'),
    ('Wedgelet_Block_PETG_L', 'V4_Wedgelet_Block_PETG_L_x1.stl'),
    ('Wedgelet_Block_PETG_R', 'V4_Wedgelet_Block_PETG_R_x1.stl'),
]


def export_manufacturing(design, root):
    os.makedirs(MFG, exist_ok=True)
    out = {'dxf': [], 'stl': []}
    for name, fname in DXF_PARTS:
        body = find_body(root, name)
        comp = body.parentComponent
        # largest face that carries the most loops (holes must show in the DXF)
        planes = [f for f in body.faces if f.geometry.objectType == 'adsk::core::Plane']
        most = max(f.loops.count for f in planes)
        face = max((f for f in planes if f.loops.count == most), key=lambda f: f.area)
        sketch = comp.sketches.add(face)
        sketch.name = 'DXF_' + name
        sketch.project(face)
        path = os.path.join(MFG, fname)
        if not sketch.saveAsDXF(path):
            raise RuntimeError('DXF failed: ' + name)
        bb = face.boundingBox
        out['dxf'].append({'file': fname, 'face_area_mm2': round(face.area * 100, 1),
                           'bbox_mm': [round((bb.maxPoint.x - bb.minPoint.x) * 10, 2),
                                       round((bb.maxPoint.y - bb.minPoint.y) * 10, 2),
                                       round((bb.maxPoint.z - bb.minPoint.z) * 10, 2)]})
        sketch.isVisible = False
    em = design.exportManager
    for name, fname in STL_PARTS:
        body = find_body(root, name)
        path = os.path.join(MFG, fname)
        opt = em.createSTLExportOptions(body, path)
        opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        opt.isBinaryFormat = True
        if not em.execute(opt):
            raise RuntimeError('STL failed: ' + name)
        out['stl'].append({'file': fname, 'bytes': os.path.getsize(path)})
    return out


def run(_):
    app = adsk.core.Application.get()
    aluminum = material(app, 'Aluminum 6061')
    steel = material(app, 'Steel')
    polymer = material(app, 'Nylon 6')        # stands in for printed TPU/PETG
    proxy = material(app, 'ABS Plastic')
    rubber = material(app, 'Rubber')

    for d in list(app.documents):
        if d.name.startswith('ROBOT_V4_Beater'):
            d.close(False)
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    doc.name = 'ROBOT_V4_Beater'
    design = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    root = design.rootComponent

    # --- 01 printed chassis: one tub body + bolt-on lid ------------------------
    ch = group(root, '01_Printed_Chassis_TPU')
    tub_comp = box(ch, 'Tub_TPU', -88, 88, -110, 64, 4, 7, polymer)        # floor first, everything joins it
    tub = tub_comp.bRepBodies.item(0)
    for side in (-1, 1):
        for wy in WHEEL_Y:
            x0, x1 = sorted((side * 60, side * 80))
            cut_box(tub_comp, tub, 'Floor_WheelWell', x0, x1, wy - 35, wy + 35, 3, 8)
    for x0, x1 in [(-88, -80), (80, 88)]:
        join_box(tub_comp, tub, 'Side_Wall', x0, x1, -110, 100, 7, 57)
    join_box(tub_comp, tub, 'Rear_Wall', -80, 80, -110, -104, 7, 57)
    join_box(tub_comp, tub, 'Front_Bulkhead', -80, 80, 62, 64, 7, 57)
    cut_box(tub_comp, tub, 'Belt_Slot', -6, 6, 61, 65, 12, 52)
    # drive motor walls: motor face bolts here, shaft passes to the wheel
    for wy in WHEEL_Y:
        for x0, x1 in [(-60, -56), (56, 60)]:
            join_box(tub_comp, tub, 'Motor_Wall', x0, x1, wy - 13, wy + 13, 7, 57)
        extrude_yz(tub_comp, 'Motor_Shaft_Hole', [('circle', (wy, AXLE_Z, 4.0))], -61, -55, CUT, tub)
        extrude_yz(tub_comp, 'Motor_Shaft_Hole', [('circle', (wy, AXLE_Z, 4.0))], 55, 61, CUT, tub)
    # front floor + support blocks under the wedgelets (outboard of the uprights)
    for x0, x1 in [(-88, -34), (34, 88)]:
        join_box(tub_comp, tub, 'Front_Floor', x0, x1, 64, 100, 4, 7)
    # side-wall clearance holes for the screws that hold the PETG wedgelet blocks (section 03)
    for xa, xb in [(-89, -79), (79, 89)]:
        for y, z in BLOCK_WALL_SCREWS:
            extrude_yz(tub_comp, 'Block_Screw_Clearance', [('circle', (y, z, 1.7))], xa, xb, CUT, tub)
    # blocks beside the uprights: 4 M3 x 20 per side go from the bay side of the block (x +-50),
    # through a clearance hole, into M3 threads tapped in the 6 mm upright (no inserts in TPU here)
    for x0, x1 in [(-50, -34), (34, 50)]:
        join_box(tub_comp, tub, 'Weapon_Side_Block', x0, x1, 64, 98, 7, 50)
        for y, z in UPRIGHT_SCREWS:
            extrude_yz(tub_comp, 'Upright_Screw_Clearance', [('circle', (y, z, 1.7))], x0 - 1, x1 + 1, CUT, tub)
    # lid screws: heat-set insert holes (D4 x 6) in the wall tops, clearance D3.4 in the lid
    for x in (-84, 84):
        for y in LID_SCREWS_Y:
            cut_z(tub_comp, tub, 'Lid_Insert', x, y, 2.0, 50, 58)
    # power switch key hole through the right wall, in the gap between the wheels
    extrude_yz(tub_comp, 'Switch_Key_Hole', [('circle', (-30, 25, 3.0))], 79, 89, CUT, tub)
    tub.name = 'Tub_TPU'

    lid_comp = box(ch, 'Lid_TPU_3mm', -88, 88, -110, 64, 57, 60, polymer)
    lid = lid_comp.bRepBodies.item(0)
    for side in (-1, 1):
        for wy in WHEEL_Y:
            x0, x1 = sorted((side * 60, side * 80))
            cut_box(lid_comp, lid, 'Lid_WheelWell', x0, x1, wy - 35, wy + 35, 56, 61)
    for x in (-84, 84):
        for y in LID_SCREWS_Y:
            cut_z(lid_comp, lid, 'Lid_Screw', x, y, 1.7, 56, 61)

    # --- 02 weapon -------------------------------------------------------------
    wp = group(root, '02_Weapon_Beater')
    for side, x0, x1 in [('L', -33, -27), ('R', 27, 33)]:
        # Lower front edge stays >= 1 mm above the wedgelet ramp, which runs under the upright.
        solid_x(wp, 'Upright_6061_6mm_' + side,
                [('poly', [(64, 7), (64, 57), (104, 57), (112, 40), (112, 16.5), (100, 25), (98, 25), (98, 7)]),
                 ('circle', (ROTOR_Y, AXLE_Z, 4.1))] + [('circle', (y, z, 1.25)) for y, z in UPRIGHT_SCREWS],  # M3 tap drill
                x0, x1, aluminum)
    # braces tie the two uprights: 2 holes into M3 taps in each upright's top/bottom edge
    for name, z0, z1 in [('Weapon_Top_Brace_3mm', 57, 60), ('Weapon_Bottom_Brace_3mm', 4, 7)]:
        c = box(wp, name, -33, 33, 64, 74, z0, z1, aluminum)
        for x in (-30, 30):
            cut_z(c, c.bRepBodies.item(0), name + '_Hole', x, 69, 1.7, z0 - 1, z1 + 1)
    # rotor: 2 laser-cut discs bolted (4x M3 countersunk) to an aluminium hub on two 608 bearings
    disc_loops = [('poly', tooth_profile()), ('circle', (ROTOR_Y, AXLE_Z, 7.0))]
    for a in (45, 135, 225, 315):
        t = math.radians(a)
        disc_loops.append(('circle', (ROTOR_Y + 13 * math.cos(t), AXLE_Z + 13 * math.sin(t), 1.7)))
    for a in (90, 270):
        t = math.radians(a)
        disc_loops.append(('circle', (ROTOR_Y + 17.5 * math.cos(t), AXLE_Z + 17.5 * math.sin(t), 4.0)))
    discs = [solid_x(wp, 'Beater_Disc_' + side, disc_loops, x0, x1, steel)[1]
             for side, x0, x1 in [('L', -26, -20), ('R', 20, 26)]]
    hub_comp, hub = cyl_x(wp, 'Beater_Hub_6061_D30', -20, 20, ROTOR_Y, AXLE_Z, 15.0, aluminum, bore=11.0)
    # round-belt groove in the middle, M3 tap holes (D2.5 x 8) in both ends
    extrude_yz(hub_comp, 'Belt_Groove', [('circle', (ROTOR_Y, AXLE_Z, 15.5)), ('circle', (ROTOR_Y, AXLE_Z, 13.5))], -3, 3, CUT, hub)
    for a in (45, 135, 225, 315):
        t = math.radians(a)
        c = ('circle', (ROTOR_Y + 13 * math.cos(t), AXLE_Z + 13 * math.sin(t), 1.25))
        extrude_yz(hub_comp, 'M3_Tap', [c], -20, -12, CUT, hub)
        extrude_yz(hub_comp, 'M3_Tap', [c], 12, 20, CUT, hub)
    for side, x0, x1 in [('L', -20, -13), ('R', 13, 20)]:
        cyl_x(wp, 'Bearing_608_' + side, x0, x1, ROTOR_Y, AXLE_Z, 11.0, steel, bore=4.0)
    # axial clamp stack on the dead shaft: upright | spacer | 608 inner race | spacer | inner race | spacer | upright
    for name, x0, x1 in [('Spacer_Outer_L', -27, -20), ('Spacer_Inner', -13, 13), ('Spacer_Outer_R', 20, 27)]:
        cyl_x(wp, name + '_ID8.2_OD11', x0, x1, ROTOR_Y, AXLE_Z, 5.5, aluminum, bore=4.1)
    cyl_x(wp, 'Dead_Shaft_M8_12.9', -33, 33, ROTOR_Y, AXLE_Z, 4.0, steel)
    cyl_x(wp, 'D3536_1250kV_Can_ENVELOPE', -41, -5, MOTOR_Y, AXLE_Z, 17.5, proxy)
    cyl_x(wp, 'Motor_Pulley_D26_Groove', -4, 4, MOTOR_Y, AXLE_Z, 13.0, aluminum, bore=2.5)
    c = box(wp, 'Weapon_Motor_Mount_3mm', -45, -42, 26, 62, 10, 54, aluminum)
    extrude_yz(c, 'Motor_Centre', [('circle', (MOTOR_Y, AXLE_Z, 5.0))], -46, -41, CUT, c.bRepBodies.item(0))
    for dy, dz in [(-8, 0), (8, 0), (0, -9.5), (0, 9.5)]:          # 16 x 19 cross pattern: CHECK on the real motor
        extrude_yz(c, 'Motor_M3', [('circle', (MOTOR_Y + dy, AXLE_Z + dz, 1.7))], -46, -41, CUT, c.bRepBodies.item(0))
    for name, z0, z1 in [('Belt_Upper_ENVELOPE', 45.5, 47.0), ('Belt_Lower_ENVELOPE', 17.0, 18.5)]:
        box(wp, name, -3, 3, 52, 86, z0, z1, rubber)

    # --- 03 wedgelets --------------------------------------------------------
    wd = group(root, '03_Steel_Wedgelets_2mm')
    wedgelets = []
    for side, x0, x1, holes in [('L', -88, -27, (-40, -75)), ('R', 27, 88, (40, 75))]:
        c, body = solid_x(wd, 'Wedgelet_' + side, [('poly', [(128, 0.5), (128, 2.5), (100, 22.5), (100, 20.5)])],
                          x0, x1, steel)
        wedgelets.append(body)
        # M3 countersunk through the plate
        face_holes(c, body, 'Wedgelet_M3', [(x, 103.0, 20.357) for x in holes], 1.7, 3.0)
    # PETG blocks under the wedgelets (heat-set inserts hold in PETG, not in TPU 95A):
    #  - a prism whose top is the wedgelet's underside (x 34..88), carrying 2 inserts on the screw axes
    #  - a rail along the side wall (x 70..80, y 64..100) that bears on the bulkhead and takes
    #    2 horizontal M3 through the side wall into inserts
    blocks = []
    for side, sgn, holes in [('L', -1, (-40, -75)), ('R', 1, (40, 75))]:
        x0, x1 = sorted((sgn * 34, sgn * 88))
        bc, bb = solid_x(wd, 'Wedgelet_Block_PETG_' + side, [('poly', WEDGE_SUPPORT)], x0, x1, polymer)
        r0, r1 = sorted((sgn * 70, sgn * 80))
        join_box(bc, bb, 'Rail', r0, r1, 64, 100, 7, 20)
        i0, i1 = sorted((sgn * 72, sgn * 80))
        for y, z in BLOCK_WALL_SCREWS:
            extrude_yz(bc, 'Wall_Insert', [('circle', (y, z, 2.0))], i0, i1, CUT, bb)
        face_holes(bc, bb, 'Wedgelet_Insert', [(x, 102.05, 19.03) for x in holes], 2.0, 7.0)
        blocks.append(bb)

    # --- 04 drive --------------------------------------------------------------
    dr = group(root, '04_Drive_4WD')
    for wy, pos in zip(WHEEL_Y, ('Front', 'Rear')):
        for side, mx, wx in [('L', (-56, -4), (-78, -62)), ('R', (4, 56), (62, 78))]:
            cyl_x(dr, 'JGA25_370_400rpm_%s_%s' % (pos, side), mx[0], mx[1], wy, AXLE_Z, 12.5, proxy)
            solid_x(dr, 'Wheel_Hub_PETG_%s_%s' % (pos, side),
                    [('circle', (wy, AXLE_Z, 22.0)), ('poly', d_bore(wy, AXLE_Z))], wx[0], wx[1], polymer)
            cyl_x(dr, 'Wheel_Tyre_TPU_%s_%s' % (pos, side), wx[0], wx[1], wy, AXLE_Z, WHEEL_R, rubber, bore=22.0)

    # --- 05 electronics -------------------------------------------------------
    el = group(root, '05_Electronics_ENVELOPES')
    # Free bay between the wheel pairs: x -56..56, y -57.5..-2.5 (drive motors bound it).
    box(el, '3S_1000mAh_LiPo_75x35x25', -37.5, 37.5, -45, -10, 8, 33, proxy)
    box(el, 'Skywalker_40A_ESC_55x25x12', -60, -5, -40, -15, 34, 46, proxy)
    box(el, 'ESP32_DevKit_51.5x28.3', 4, 55.5, -42, -13.7, 34, 48, proxy)
    box(el, 'DRV8871_L', 39, 59, -45, -25, 8, 14, proxy)
    box(el, 'DRV8871_R', 39, 59, -24, -4, 8, 14, proxy)
    # Switch sits in the 16 mm gap between front and rear wheels, keyed from the side wall.
    box(el, 'Power_Switch_Link', 66, 80, -34, -26, 20, 30, proxy)

    # --- checks -----------------------------------------------------------------
    bodies = [b for occ in root.allOccurrences for b in occ.bRepBodies]
    coll = adsk.core.ObjectCollection.create()
    for b in bodies:
        coll.add(b)
    inp = design.createInterferenceInput(coll)
    inp.areCoincidentFacesIncluded = False
    res = design.analyzeInterference(inp)
    tbm = adsk.fusion.TemporaryBRepManager.get()
    pairs = []
    for i in range(res.count):
        a, b = res.item(i).entityOne, res.item(i).entityTwo
        t = tbm.copy(a)
        tbm.booleanOperation(t, tbm.copy(b), adsk.fusion.BooleanTypes.IntersectionBooleanType)
        pairs.append([a.name, b.name, round(t.volume * 1000, 3)])

    # rotor sweep (tip radius + 1 mm, full rotor width) against everything that does not spin with it
    # Discs sweep the tip radius; between them only the tube/pulley (r 15) spins, so the belt may pass there.
    sweep_bodies = []
    for name, x0, x1, r in [('Sweep_Disc_L', -26.5, -19.5, TIP_R + 1.0), ('Sweep_Disc_R', 19.5, 26.5, TIP_R + 1.0),
                            ('Sweep_Centre', -19.5, 19.5, 15.5)]:
        sweep_bodies.append(cyl_x(root, name, x0, x1, ROTOR_Y, AXLE_Z, r, proxy))
    sweep_hits = []
    for comp, sweep in sweep_bodies:
        for b in bodies:
            if b.name.startswith(('Beater_', 'Bearing_608', 'Dead_Shaft', 'Spacer_')):
                continue
            t = tbm.copy(sweep)
            tbm.booleanOperation(t, tbm.copy(b), adsk.fusion.BooleanTypes.IntersectionBooleanType)
            if t.volume > 1e-6:
                sweep_hits.append([sweep.name, b.name, round(t.volume * 1000, 3)])
    for comp, _ in sweep_bodies:
        [o for o in root.occurrences if o.component == comp][0].deleteMe()

    def mass_g(prefixes):
        return round(sum(b.physicalProperties.mass for b in bodies
                         if any(b.parentComponent.name.startswith(p) or b.name.startswith(p) for p in prefixes)) * 1000, 1)

    report = {
        'bodies': len(bodies),
        'interferences': pairs,
        'rotor_sweep_hits_1mm_clearance': sweep_hits,
        'rotor_mass_g_discs_hub_bearings': mass_g(['Beater_Disc', 'Beater_Hub', 'Bearing_608']),
        'disc_mass_g_each': round(discs[0].physicalProperties.mass * 1000, 1),
        'hub_mass_g': round(hub.physicalProperties.mass * 1000, 1),
        'uprights_mass_g': mass_g(['Upright_']),
        'wedgelets_mass_g': mass_g(['Wedgelet_L', 'Wedgelet_R']),
        'tub_mass_g_100pct_nylon6': round(tub.physicalProperties.mass * 1000, 1),
        'lid_mass_g_100pct_nylon6': round(lid.physicalProperties.mass * 1000, 1),
        'wheel_hub_g_each_nylon6': mass_g(['Wheel_Hub_PETG_Front_L']),
        'wheel_tyre_g_each_rubber': mass_g(['Wheel_Tyre_TPU_Front_L']),
        'braces_and_motor_mount_g': mass_g(['Weapon_Top_Brace', 'Weapon_Bottom_Brace', 'Weapon_Motor_Mount']),
        'wedgelet_blocks_g_both_100pct_nylon6': mass_g(['Wedgelet_Block_PETG']),
        'bbox_mm': [[round(v * 10, 1) for v in (root.boundingBox.minPoint.x, -root.boundingBox.maxPoint.z, root.boundingBox.minPoint.y)],
                    [round(v * 10, 1) for v in (root.boundingBox.maxPoint.x, -root.boundingBox.minPoint.z, root.boundingBox.maxPoint.y)]],
    }

    export = design.exportManager
    f3d = os.path.join(OUT, 'ROBOT_V4_Beater.f3d')
    step = os.path.join(OUT, 'ROBOT_V4_Beater.step')
    if not export.execute(export.createFusionArchiveExportOptions(f3d)):
        raise RuntimeError('F3D export failed')
    if not export.execute(export.createSTEPExportOptions(step)):
        raise RuntimeError('STEP export failed')
    app.activeViewport.fit()
    report['f3d'], report['step'] = f3d, step
    report['manufacturing'] = export_manufacturing(design, root)
    print(json.dumps(report, ensure_ascii=False))
