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
PIN_Y, PIN_Z = 102.0, 14.5                                    # wedgelet hinge axis (along x)
# skirt hinge: knuckles on the tub and clips on the plate alternate along the wire (1.5 mm wire, 1.6 mm bores)
SKIRT_KNUCKLES_SIDE = [(-108, -98), (-55, -45), (0, 10), (80, 90)]
SKIRT_CLIPS_SIDE = [(-90, -75), (-35, -20), (25, 40), (55, 70)]
SKIRT_KNUCKLES_REAR = [(-86, -76), (-30, -20), (20, 30), (76, 86)]
SKIRT_CLIPS_REAR = [(-65, -50), (-8, 8), (50, 65)]
# carrier section: top edge = plate underside (z = 20.5 - (y - 100) * 20/28), clear of the spine (z <= 11 at y <= 104)
CARRIER = [(100, 11.5), (100, 20.5), (114, 20.5 - 14 * 20 / 28), (114, 8), (105, 8), (104.5, 11.5)]
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


def extrude_xz(comp, name, circles, y0, y1, operation, body=None):
    """Extrude circles (x, z, r) along conceptual y from y0 to y1 (sketch on Fusion XY = conceptual x-z)."""
    sketch = comp.sketches.add(comp.xYConstructionPlane)
    sketch.name = name
    for x, z, r in circles:
        sketch.sketchCurves.sketchCircles.addByCenterRadius(point(sketch, x, 0, z), cm(r))
    profiles = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        profiles.add(sketch.profiles.item(i))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(profiles, operation)
    # Fusion +Z is conceptual -y: start at -y1 and extrude +Z by (y1 - y0)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(-y1)))
    extent(inp, y1 - y0)
    if body is not None:
        inp.participantBodies = [body]
    feature = extrudes.add(inp)
    sketch.isVisible = False
    return feature


def cyl_y(parent, name, x, z, y0, y1, r, mat):
    comp = part(parent, name)
    body = extrude_xz(comp, name, [(x, z, r)], y0, y1, NEW_BODY).bodies.item(0)
    body.name = name
    body.material = mat
    return comp, body


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
    ('Skirt_Plate_1mm_R', 'V4_Skirt_Side_1mm_Al_x2.dxf'),
    ('Skirt_Plate_1mm_Rear', 'V4_Skirt_Rear_1mm_Al_x1.dxf'),
    ('Beater_Disc_L', 'V4_Beater_Disc_6mm_steel_x2.dxf'),
    ('Upright_6061_6mm_L', 'V4_Upright_6mm_6061_x2.dxf'),
    ('Wedgelet_L', 'V4_Wedgelet_2mm_steel_x2_mirror.dxf'),
    ('Weapon_Top_Brace_3mm', 'V4_Brace_3mm_6061_x2.dxf'),
    ('Weapon_Motor_Mount_3mm', 'V4_Weapon_Motor_Mount_3mm_6061_x1.dxf'),
]
STL_PARTS = [
    ('Skirt_Clip_PETG_R1', 'V4_Skirt_Clip_Side_PETG_x8.stl'),
    ('Skirt_Clip_PETG_Rear1', 'V4_Skirt_Clip_Rear_PETG_x3.stl'),
    ('Hall_Post_PETG', 'V4_Hall_Post_PETG_x1.stl'),
    ('Tub_TPU', 'V4_Tub_TPU95A_x1.stl'),
    ('Lid_TPU_2mm', 'V4_Lid_TPU95A_x1.stl'),
    ('Wheel_Hub_PETG_Front_L', 'V4_Wheel_Hub_PETG_x4.stl'),
    ('Wheel_Tyre_TPU_Front_L', 'V4_Wheel_Tyre_TPU95A_x4.stl'),
    ('Wedgelet_Block_PETG_L', 'V4_Wedgelet_Hinge_Block_PETG_L_x1.stl'),
    ('Wedgelet_Block_PETG_R', 'V4_Wedgelet_Hinge_Block_PETG_R_x1.stl'),
    ('Wedgelet_Carrier_PETG_R1', 'V4_Wedgelet_Carrier_Inner_PETG_x2_mirror.stl'),
    ('Wedgelet_Carrier_PETG_R2', 'V4_Wedgelet_Carrier_Outer_PETG_x2_mirror.stl'),
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
        # walls to z 58 under a 2 mm lid; front end at y 97 so the wedgelet's rear edge can swing back
        join_box(tub_comp, tub, 'Side_Wall', x0, x1, -110, 97, 7, 58)
    join_box(tub_comp, tub, 'Rear_Wall', -80, 80, -110, -104, 7, 58)
    join_box(tub_comp, tub, 'Front_Bulkhead', -80, 80, 62, 64, 7, 58)
    cut_box(tub_comp, tub, 'Belt_Slot', -6, 6, 61, 65, 12, 52)
    # drive motor walls: motor face bolts here, shaft passes to the wheel
    for wy in WHEEL_Y:
        for x0, x1 in [(-60, -56), (56, 60)]:
            join_box(tub_comp, tub, 'Motor_Wall', x0, x1, wy - 13, wy + 13, 7, 58)
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
        join_box(tub_comp, tub, 'Weapon_Side_Block', x0, x1, 64, 96, 7, 50)
        for y, z in UPRIGHT_SCREWS:
            extrude_yz(tub_comp, 'Upright_Screw_Clearance', [('circle', (y, z, 1.7))], x0 - 1, x1 + 1, CUT, tub)
    # lid screws: heat-set insert holes (D4 x 6) in the wall tops, clearance D3.4 in the lid
    for x in (-84, 84):
        for y in LID_SCREWS_Y:
            cut_z(tub_comp, tub, 'Lid_Insert', x, y, 2.0, 51, 59)
    # power switch key hole through the right wall, in the gap between the wheels
    extrude_yz(tub_comp, 'Switch_Key_Hole', [('circle', (-30, 25, 3.0))], 79, 89, CUT, tub)
    tub.name = 'Tub_TPU'

    lid_comp = box(ch, 'Lid_TPU_2mm', -88, 88, -110, 64, 58, 60, polymer)   # 2 mm (was 3) pays for the skirts
    lid = lid_comp.bRepBodies.item(0)
    for side in (-1, 1):
        for wy in WHEEL_Y:
            x0, x1 = sorted((side * 60, side * 80))
            cut_box(lid_comp, lid, 'Lid_WheelWell', x0, x1, wy - 35, wy + 35, 57, 61)
    for x in (-84, 84):
        for y in LID_SCREWS_Y:
            cut_z(lid_comp, lid, 'Lid_Screw', x, y, 1.7, 57, 61)

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

    # --- 03 hinged wedgelets ------------------------------------------------
    # Each 2 mm steel plate is screwed (2x M3 countersunk into heat-set inserts) to two PETG carriers that
    # pivot on a 3 mm steel pin along x at (y 102, z 14.5). The pin runs through fixed PETG knuckles on a
    # spine + rail that bolt to the tub. At rest the plate tip lies on the floor under its own weight
    # (drawn here 0.5 mm up; it settles ~1 deg). It can swing up ~10 deg before the plate meets the
    # upright's lower edge (rigid up-stop); the floor is the down-stop. three_way_sim: front edge ~0 mm.
    wd = group(root, '03_Hinged_Wedgelets')
    wedgelets = []
    plate = [(128, 0.5), (128, 2.5), (100, 22.5), (100, 20.5)]
    for side, x0, x1, holes in [('L', -88, -27, (-49, -75)), ('R', 27, 88, (49, 75))]:
        c, body = solid_x(wd, 'Wedgelet_' + side, [('poly', plate)], x0, x1, steel)
        wedgelets.append(body)
        face_holes(c, body, 'Wedgelet_M3', [(x, 110.0, 22.5 - 10 * 20 / 28) for x in holes], 1.7, 3.0)
    blocks = []
    for side, sgn, holes in [('L', -1, (-49, -75)), ('R', 1, (49, 75))]:
        def xr(a0, a1):
            return sorted((sgn * a0, sgn * a1))
        # fixed: spine + 3 knuckles + rail (rail takes 2 horizontal M3 through the side wall)
        f0, f1 = xr(34, 88)
        bc, bb = solid_x(wd, 'Wedgelet_Block_PETG_' + side, [('poly', [(100, 4), (100, 11), (104, 11), (104, 4)])],
                         f0, f1, polymer)
        for k0, k1 in [(34, 38), (60, 66), (84, 88)]:
            a0, a1 = xr(k0, k1)
            join_box(bc, bb, 'Knuckle', a0, a1, 100, 104, 11, 16.5)
        r0, r1 = xr(70, 80)
        join_box(bc, bb, 'Bridge', r0, r1, 97, 101, 7, 10)    # ties the rail to the spine, below the carrier's swing
        join_box(bc, bb, 'Rail', r0, r1, 64, 98, 7, 18)      # 2 mm clear of the swinging plate and carriers
        extrude_yz(bc, 'Pin_Bore', [('circle', (PIN_Y, PIN_Z, 1.6))], f0 - 1, f1 + 1, CUT, bb)
        i0, i1 = xr(72, 80)
        for y, z in BLOCK_WALL_SCREWS:
            extrude_yz(bc, 'Wall_Insert', [('circle', (y, z, 2.0))], i0, i1, CUT, bb)
        if bc.bRepBodies.count != 1:
            raise RuntimeError('wedgelet block %s split into %d bodies' % (side, bc.bRepBodies.count))
        blocks.append(bb)
        # moving carriers between the knuckles (1 mm gaps), top = plate underside
        for n, (c0, c1) in enumerate([(39, 59), (67, 83)]):
            a0, a1 = xr(c0, c1)
            cc, cb = solid_x(wd, 'Wedgelet_Carrier_PETG_%s%d' % (side, n + 1), [('poly', CARRIER)], a0, a1, polymer)
            extrude_yz(cc, 'Pin_Bore', [('circle', (PIN_Y, PIN_Z, 1.6))], a0 - 1, a1 + 1, CUT, cb)
            face_holes(cc, cb, 'Wedgelet_Insert', [(x, 109.05, 14.03) for x in holes if a0 < x < a1], 2.0, 6.0)
        # pin: inner end stops 0.5 mm from the upright (inward stop), outer end carries an E-clip (DIN 6799
        # size 2.3 for a 3 mm shaft) in a 0.6 mm groove just outside the last knuckle
        q0, q1 = xr(33.5, 90.5)
        pc, pb = cyl_x(wd, 'Hinge_Pin_D3_' + side, q0, q1, PIN_Y, PIN_Z, 1.5, steel)
        g0, g1 = xr(89.0, 89.6)
        extrude_yz(pc, 'Eclip_Groove', [('circle', (PIN_Y, PIN_Z, 1.5)), ('circle', (PIN_Y, PIN_Z, 1.15))], g0, g1, CUT, pb)
        cyl_x(wd, 'Eclip_DIN6799_2.3_' + side, g0, g1, PIN_Y, PIN_Z, 3.5, steel, bore=1.15)

    # --- 03b hinged skirts (sides + rear) ------------------------------------
    # 1 mm 5052/6061 plates hang from a 1.5 mm spring-steel wire hinge (printed knuckles on the wall,
    # PETG clips on the plate; a store piano hinge weighs ~100 g here) and rest on the floor,
    # so a wedge meets a ~0 mm edge instead of the 4 mm tub floor (three_way_sim: 98-100 % -> 7-24 %).
    # Hinge axis at z 14 keeps the strip below the switch key hole (z 22-28). Works right side up only:
    # nobody in the field can flip V4 (needs a 165 mm edge lift).
    # Hinge axis 1.5 mm outside the plate at z 15: the wall is the inward stop (a wedge pushes the skirt
    # flat against the wall), outward and upward the plate swings free over bumps.
    sk = group(root, '03b_Hinged_Skirts')
    for side, sgn in [('L', -1), ('R', 1)]:
        def xs(a0, a1):
            return sorted((sgn * a0, sgn * a1))
        p0, p1 = xs(88.5, 89.5)
        pc, plate_b = box(sk, 'Skirt_Plate_1mm_' + side, p0, p1, -110, 97, 0, 13, aluminum), None
        plate_b = pc.bRepBodies.item(0)
        cyl_y(sk, 'Skirt_Wire_D1.5_' + side, sgn * 90, 15, -110, 97, 0.75, steel)
        for k0, k1 in SKIRT_KNUCKLES_SIDE:                               # printed with the tub
            a0, a1 = xs(88, 92)
            join_box(tub_comp, tub, 'Skirt_Knuckle', a0, a1, k0, k1, 13, 17)
        extrude_xz(tub_comp, 'Skirt_Wire_Bore', [(sgn * 90, 15, 0.8)], -111, 98, CUT, tub)
        for n, (c0, c1) in enumerate(SKIRT_CLIPS_SIDE):
            a0, a1 = xs(89.5, 92)
            cc = box(sk, 'Skirt_Clip_PETG_%s%d' % (side, n + 1), a0, a1, c0, c1, 10, 17, polymer)
            cb = cc.bRepBodies.item(0)
            extrude_xz(cc, 'Wire_Bore', [(sgn * 90, 15, 0.8)], c0 - 1, c1 + 1, CUT, cb)
            ym = (c0 + c1) / 2
            m0, m1 = xs(88, 93)
            extrude_yz(cc, 'M2_Hole', [('circle', (ym, 11.5, 1.1))], m0, m1, CUT, cb)
            extrude_yz(pc, 'M2_Hole', [('circle', (ym, 11.5, 1.1))], m0, m1, CUT, plate_b)
    rc = box(sk, 'Skirt_Plate_1mm_Rear', -88, 88, -111.5, -110.5, 0, 13, aluminum)
    rplate = rc.bRepBodies.item(0)
    cyl_x(sk, 'Skirt_Wire_D1.5_Rear', -88, 88, -112, 15, 0.75, steel)
    for k0, k1 in SKIRT_KNUCKLES_REAR:
        join_box(tub_comp, tub, 'Skirt_Knuckle', k0, k1, -114, -110, 13, 17)
    extrude_yz(tub_comp, 'Skirt_Wire_Bore', [('circle', (-112, 15, 0.8))], -89, 89, CUT, tub)
    for n, (c0, c1) in enumerate(SKIRT_CLIPS_REAR):
        cc = box(sk, 'Skirt_Clip_PETG_Rear%d' % (n + 1), c0, c1, -114, -111.5, 10, 17, polymer)
        cb = cc.bRepBodies.item(0)
        extrude_yz(cc, 'Wire_Bore', [('circle', (-112, 15, 0.8))], c0 - 1, c1 + 1, CUT, cb)
        xm = (c0 + c1) / 2
        extrude_xz(cc, 'M2_Hole', [(xm, 11.5, 1.1)], -115, -110, CUT, cb)
        extrude_xz(rc, 'M2_Hole', [(xm, 11.5, 1.1)], -115, -110, CUT, rplate)

    # --- 04 drive --------------------------------------------------------------
    dr = group(root, '04_Drive_4WD')
    for wy, pos in zip(WHEEL_Y, ('Front', 'Rear')):
        for side, mx, wx in [('L', (-56, -4), (-78, -62)), ('R', (4, 56), (62, 78))]:
            cyl_x(dr, 'JGA25_370_400rpm_%s_%s' % (pos, side), mx[0], mx[1], wy, AXLE_Z, 12.5, proxy)
            solid_x(dr, 'Wheel_Hub_PETG_%s_%s' % (pos, side),
                    [('circle', (wy, AXLE_Z, 22.0)), ('poly', d_bore(wy, AXLE_Z))] +
                    [('circle', (wy + 13 * math.cos(math.radians(a)), AXLE_Z + 13 * math.sin(math.radians(a)), 4.0))
                     for a in range(0, 360, 72)],                     # 5 lightening holes
                    wx[0], wx[1], polymer)
            cyl_x(dr, 'Wheel_Tyre_TPU_%s_%s' % (pos, side), wx[0], wx[1], wy, AXLE_Z, WHEEL_R, rubber, bore=22.0)

    # --- 05 electronics -------------------------------------------------------
    el = group(root, '05_Electronics_ENVELOPES')
    # Free bay between the wheel pairs: x -56..56, y -57.5..-2.5 (drive motors bound it).
    # battery moved forward (0.5 mm from the front drive motors) to free a floor strip for the IMU
    box(el, '3S_850mAh_LiPo_ENVELOPE_75x35x25', -37.5, 37.5, -38, -3, 8, 33, proxy)
    box(el, 'Skywalker_40A_ESC_55x25x12', -60, -5, -40, -15, 34, 46, proxy)
    box(el, 'ESP32_DevKit_51.5x28.3', 4, 55.5, -42, -13.7, 34, 48, proxy)
    box(el, 'DRV8871_L', 39, 59, -45, -25, 8, 14, proxy)
    box(el, 'DRV8871_R', 39, 59, -24, -4, 8, 14, proxy)
    # --- sensors ---
    # IMU (GY-521 MPU6050, 21x16 board) flat on a 1 mm foam pad, Z axis up, X axis to the robot's right,
    # centred left-right and behind the battery, clear of the weapon motor and ESC
    box(el, 'IMU_Foam_Pad_1mm', -10.5, 10.5, -56, -40, 7, 8, rubber)
    box(el, 'IMU_GY521_MPU6050_21x16', -10.5, 10.5, -56, -40, 8, 11.5, proxy)
    # weapon RPM: A3144 hall sensor 2.5 mm from the motor pulley face (x = 4; flexible TPU floor, so
    # not closer), reading a 3x2 mm magnet pressed into that face at r = 7 mm; flat face toward the
    # pulley; sensor sits on a printed PETG post on the tub floor
    box(el, 'Hall_Post_PETG', 8.0, 12.0, 40, 48, 7, 43, polymer)
    box(el, 'Hall_A3144_TO92', 6.5, 8.0, 42, 46, 37, 41, proxy)
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
        'wedgelet_carriers_g_all_100pct_nylon6': mass_g(['Wedgelet_Carrier_PETG']),
        'wedgelet_pins_g': mass_g(['Hinge_Pin_D3']),
        'skirt_plates_g': mass_g(['Skirt_Plate']),
        'skirt_wires_g': mass_g(['Skirt_Wire']),
        'skirt_clips_g_all_100pct_nylon6': mass_g(['Skirt_Clip_PETG']),
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
