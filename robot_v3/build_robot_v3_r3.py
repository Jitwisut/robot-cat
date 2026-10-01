"""Create the V3 R3 CAD study in Autodesk Fusion and export local F3D/STEP.

Concept: 4WD low tub, fixed 168 mm plow wings and a servo-driven centre
lifting wedge. The servo sits mid-chassis and drives the wedge through a
crank and pushrod that are at toggle when the wedge is down.
All coordinates below are mm; Fusion's model API uses cm internally.
Purchased parts are envelopes, not supplier-accurate parts.
"""
import adsk.core
import adsk.fusion
import json
import math
import os

OUT = '/Users/jitwisutthobut/Desktop/robot/robot_v3'


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


def group(parent, name):
    occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def part(parent, name):
    occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def extent(extrude_input, distance_mm):
    definition = adsk.fusion.DistanceExtentDefinition.create(
        adsk.core.ValueInput.createByReal(cm(distance_mm)))
    extrude_input.setOneSideExtent(definition, adsk.fusion.ExtentDirections.PositiveExtentDirection)


def box(parent, name, x0, x1, y0, y1, z0, z1, mat):
    comp = part(parent, name)
    sketch = comp.sketches.add(comp.xZConstructionPlane)
    sketch.name = name + '_Footprint'
    sketch.sketchCurves.sketchLines.addTwoPointRectangle(point(sketch, x0, y0, 0), point(sketch, x1, y1, 0))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    # Fusion's start extent and positive direction on this plane both map
    # to positive world Y in this installation.
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(z0)))
    definition = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(cm(z1 - z0)))
    inp.setOneSideExtent(definition, adsk.fusion.ExtentDirections.PositiveExtentDirection)
    body = extrudes.add(inp).bodies.item(0)
    body.name = name
    body.material = mat
    sketch.isVisible = False
    return comp


def cylinder_x(parent, name, x0, x1, y, z, radius, mat):
    comp = part(parent, name)
    sketch = comp.sketches.add(comp.yZConstructionPlane)
    sketch.name = name + '_Section'
    sketch.sketchCurves.sketchCircles.addByCenterRadius(point(sketch, 0, y, z), cm(radius))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(x0)))
    extent(inp, x1 - x0)
    body = extrudes.add(inp).bodies.item(0)
    body.name = name
    body.material = mat
    sketch.isVisible = False
    return comp


def join_cylinder_x(comp, name, x0, x1, y, z, radius):
    sketch = comp.sketches.add(comp.yZConstructionPlane)
    sketch.name = name + '_Section'
    sketch.sketchCurves.sketchCircles.addByCenterRadius(point(sketch, 0, y, z), cm(radius))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(x0)))
    extent(inp, x1 - x0)
    inp.participantBodies = [comp.bRepBodies.item(0)]
    extrudes.add(inp)
    sketch.isVisible = False


def bore_x(comp, name, y, z, radius):
    sketch = comp.sketches.add(comp.yZConstructionPlane)
    sketch.name = name
    sketch.sketchCurves.sketchCircles.addByCenterRadius(point(sketch, 0, y, z), cm(radius))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(200)), True)
    inp.participantBodies = [comp.bRepBodies.item(0)]
    extrudes.add(inp)
    sketch.isVisible = False


def linkage_bar_x(parent, name, x0, x1, y1, z1, y2, z2, width, mat):
    """Create a flat diagonal linkage plate in the conceptual YZ plane."""
    comp = part(parent, name)
    sketch = comp.sketches.add(comp.yZConstructionPlane)
    dy, dz = y2 - y1, z2 - z1
    length = (dy * dy + dz * dz) ** 0.5
    ny, nz = -dz / length * width / 2.0, dy / length * width / 2.0
    polygon = [
        (y1 + ny, z1 + nz), (y2 + ny, z2 + nz),
        (y2 - ny, z2 - nz), (y1 - ny, z1 - nz),
    ]
    lines = sketch.sketchCurves.sketchLines
    for i, (y, z) in enumerate(polygon):
        yb, zb = polygon[(i + 1) % len(polygon)]
        lines.addByTwoPoints(point(sketch, 0, y, z), point(sketch, 0, yb, zb))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(x0)))
    extent(inp, x1 - x0)
    body = extrudes.add(inp).bodies.item(0)
    body.name = name
    body.material = mat
    sketch.isVisible = False
    return comp


def occurrence_for(parent, component):
    for occurrence in parent.occurrences:
        if occurrence.component == component:
            return occurrence
    raise RuntimeError('Occurrence not found: ' + component.name)


def all_occurrence_for(root, component):
    for occurrence in root.allOccurrences:
        if occurrence.component == component:
            return occurrence
    raise RuntimeError('Assembly occurrence not found: ' + component.name)


def cylindrical_face(component, radius_mm):
    for body in component.bRepBodies:
        for face in body.faces:
            geometry = face.geometry
            if geometry.objectType == 'adsk::core::Cylinder' and abs(geometry.radius * 10 - radius_mm) < 0.05:
                return face
    raise RuntimeError('Cylindrical face not found: ' + component.name)




NEW_BODY = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def extrude_polygon_x(comp, name, polygon, x0, x1, operation, body=None):
    """Extrude a closed (y, z) polygon along world X from x0 to x1."""
    sketch = comp.sketches.add(comp.yZConstructionPlane)
    sketch.name = name
    lines = sketch.sketchCurves.sketchLines
    for i, (y, z) in enumerate(polygon):
        y2, z2 = polygon[(i + 1) % len(polygon)]
        lines.addByTwoPoints(point(sketch, 0, y, z), point(sketch, 0, y2, z2))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), operation)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(x0)))
    extent(inp, x1 - x0)
    if body is not None:
        inp.participantBodies = [body]
    feature = extrudes.add(inp)
    sketch.isVisible = False
    return feature


def cut_box(comp, body, name, x0, x1, y0, y1, z0, z1):
    extrude_polygon_x(comp, name, [(y0, z0), (y0, z1), (y1, z1), (y1, z0)], x0, x1, CUT, body)


def bore_x_in(comp, body, name, y, z, radius):
    sketch = comp.sketches.add(comp.yZConstructionPlane)
    sketch.name = name
    sketch.sketchCurves.sketchCircles.addByCenterRadius(point(sketch, 0, y, z), cm(radius))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), CUT)
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(200)), True)
    inp.participantBodies = [body]
    extrudes.add(inp)
    sketch.isVisible = False


def bore_x_span(comp, body, name, y, z, radius, x0, x1):
    """Cut a hole along X only between x0 and x1 (not through-all)."""
    sketch = comp.sketches.add(comp.yZConstructionPlane)
    sketch.name = name
    sketch.sketchCurves.sketchCircles.addByCenterRadius(point(sketch, 0, y, z), cm(radius))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), CUT)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(x0)))
    extent(inp, x1 - x0)
    inp.participantBodies = [body]
    extrudes.add(inp)
    sketch.isVisible = False


def bore_y_span(comp, body, name, x, z, radius, y0, y1):
    """Cut a hole along the length only between y0 and y1."""
    sketch = comp.sketches.add(comp.xYConstructionPlane)
    sketch.name = name
    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        sketch.modelToSketchSpace(adsk.core.Point3D.create(cm(x), cm(z), 0)), cm(radius))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), CUT)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(-y1)))
    extent(inp, y1 - y0)
    inp.participantBodies = [body]
    extrudes.add(inp)
    sketch.isVisible = False


def cylinder_y(parent, name, x, z, y0, y1, radius, mat):
    """Cylinder whose axis runs along the conceptual length (world -Z)."""
    comp = part(parent, name)
    sketch = comp.sketches.add(comp.xYConstructionPlane)
    sketch.name = name + '_Section'
    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        sketch.modelToSketchSpace(adsk.core.Point3D.create(cm(x), cm(z), 0)), cm(radius))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), NEW_BODY)
    # World Z = -length, so the solid spans world Z -y1..-y0.
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(-y1)))
    extent(inp, y1 - y0)
    body = extrudes.add(inp).bodies.item(0)
    body.name = name
    body.material = mat
    sketch.isVisible = False
    return comp


def bore_y_in(comp, body, name, x, z, radius):
    sketch = comp.sketches.add(comp.xYConstructionPlane)
    sketch.name = name
    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        sketch.modelToSketchSpace(adsk.core.Point3D.create(cm(x), cm(z), 0)), cm(radius))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), CUT)
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(300)), True)
    inp.participantBodies = [body]
    extrudes.add(inp)
    sketch.isVisible = False


# ---------------------------------------------------------------------------
# Weapon geometry (conceptual mm: y forward, z up, ground at z=2).
# The ramp line, steel edge and tip heights are identical to R2; only the
# centre 79 mm now pivots. Everything behind the pivot clears the bulkhead
# (y=74) through the full 0..60 deg stroke.
PLOW_CENTER = [(74, 6), (74, 23), (104, 8.79), (104, 4.5), (94, 4.5), (82, 6)]
PLOW_WING = [(86, 4.5), (86, 17.3), (104, 8.79), (104, 4.5)]
STEEL_EDGE = [(94, 2.5), (94, 4.5), (104, 4.5), (104, 8.79), (115, 3.5), (115, 2.5)]
PIVOT = (80.5, 13.0)
LIFTER = [(80.5, 7), (80.5, 19.5), (104, 8.79), (104, 4.5), (94, 4.5), (86, 5.5)]
LUG_POINT = (84.0, 25.0)
SERVO_AXIS = (30.0, 18.0)
CRANK = 12.0
LIFT_MAX_DEG = 60
WEDGE_HALF = 39.5
WING_INNER = 40.0
PIN_HALF = 46.0  # pin runs through a 6 mm clevis in each wing
BOLT_X = (49.5, 58.5)  # >=1.5 mm wall to the pin bore end and the 62 mm face
BOLT_Z = 10.0
UP_STOP_FACE_Y = 78.54  # 0.006 mm clear of the tab at exactly 60 deg


def ramp_z(y):
    """Top of the plow ramp between y=74 and y=104."""
    return 23 + (8.79 - 23) * (y - 74) / 30.0


def rotate_yz(p, centre, angle):
    dy, dz = p[0] - centre[0], p[1] - centre[1]
    c, s = math.cos(angle), math.sin(angle)
    return (centre[0] + dy * c - dz * s, centre[1] + dy * s + dz * c)


def pushrod_length():
    return math.dist(SERVO_AXIS, LUG_POINT) - CRANK


def solve_linkage(lift):
    """Return (servo angle from rest, lug point, crank pin) for a lift angle.

    At rest the crank points straight at the lug (toggle), so a downward load
    on the wedge pushes along the crank into the servo bearing, not its gears.
    The crank folds upward as the servo turns.
    """
    b = pushrod_length()
    lug = rotate_yz(LUG_POINT, PIVOT, lift)
    d = math.dist(SERVO_AXIS, lug)
    cos_gamma = max(-1.0, min(1.0, (CRANK * CRANK + d * d - b * b) / (2 * CRANK * d)))
    base = math.atan2(lug[1] - SERVO_AXIS[1], lug[0] - SERVO_AXIS[0])
    phi = base + math.acos(cos_gamma)
    rest = math.atan2(LUG_POINT[1] - SERVO_AXIS[1], LUG_POINT[0] - SERVO_AXIS[0])
    pin = (SERVO_AXIS[0] + CRANK * math.cos(phi), SERVO_AXIS[1] + CRANK * math.sin(phi))
    return phi - rest, lug, pin


def linkage_table(servo_torque_nm=3.92):
    tip = (115.0, 3.0)
    tip_radius_m = math.dist(PIVOT, tip) / 1000.0
    rows = []
    for deg in (1, 5, 10, 20, 30, 40, 50, 60):
        lift = math.radians(deg)
        h = 1e-4
        dphi = (solve_linkage(lift + h)[0] - solve_linkage(lift - h)[0]) / (2 * h)
        torque = servo_torque_nm * dphi
        tip_now = rotate_yz(tip, PIVOT, lift)
        rows.append({
            'lift_deg': deg,
            'servo_deg': round(math.degrees(solve_linkage(lift)[0]), 1),
            'tip_above_ground_mm': round(tip_now[1] - 2.0, 1),
            'tip_force_N_ideal_stall': round(torque / tip_radius_m, 0),
        })
    return rows


def run(_):
    app = adsk.core.Application.get()
    if not os.path.isdir(OUT):
        raise RuntimeError('Output directory missing: ' + OUT)
    aluminum = material(app, 'Aluminum 6061')
    steel = material(app, 'Steel')
    polymer = material(app, 'Nylon 6')
    proxy = material(app, 'ABS Plastic')
    rubber = material(app, 'Rubber')

    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    root = design.rootComponent

    # --- 01 tub ---------------------------------------------------------
    tub = group(root, '01_Low_Tub_Chassis')
    # Floor stops at the bulkhead in the middle so the wedge heel can swing
    # down; two tabs still carry the fixed wings.
    box(tub, 'Base_6061_3mm', -68, 68, -82, 74, 3, 6, aluminum)
    for side, x0, x1 in [('L', -68, -41), ('R', 41, 68)]:
        box(tub, 'Base_Wing_Tab_' + side, x0, x1, 74, 82, 3, 6, aluminum)
    for side, x0, x1 in [('L', -68, -65), ('R', 65, 68)]:
        box(tub, 'Side_Rail_' + side + '_6061', x0, x1, -82, 82, 6, 29, aluminum)
    box(tub, 'Rear_Cross_Member_6061', -65, 65, -82, -79, 6, 29, aluminum)
    bulkhead = box(tub, 'Front_Impact_Bulkhead_6061', -62, 62, 70, 74, 6, 29, aluminum)
    bulkhead_body = bulkhead.bRepBodies.item(0)
    cut_box(bulkhead, bulkhead_body, 'Pushrod_Slot', 3, 12, 69, 75, 14, 30)

    # --- 02 drive: rear axle moved back to y=-60 to free the centre bay ----
    drive = group(root, '02_Four_Wheel_Drive')
    for side in ('L', 'R'):
        for location, y in [('Rear', -60), ('Front', 58)]:
            tag = side + '_' + location
            mx0, mx1 = (-65, -16) if side == 'L' else (16, 65)
            wx0, wx1 = (-80, -68) if side == 'L' else (68, 80)
            cylinder_x(drive, 'BBB_22mm_Gearmotor_ENVELOPE_' + tag, mx0, mx1, y, 23, 11, proxy)
            cylinder_x(drive, 'Wheel_42mm_ENVELOPE_' + tag, wx0, wx1, y, 23, 21, rubber)
    for side, x0, x1 in [('L', -82, -80), ('R', 80, 82)]:
        box(drive, 'Replaceable_Side_Guard_' + side, x0, x1, -29, 29, 8, 28, polymer)

    # --- 03 fixed front: wings + their steel edges + M3 through-bolts -----
    front = group(root, '03_Front_Fixed_Wings')
    wings = part(front, 'Fixed_Plow_Wings_6061')
    wing_bodies = []
    for sign in (-1, 1):
        inner = sorted((sign * WING_INNER, sign * 62))
        outer = sorted((sign * 62, sign * 84))
        body = extrude_polygon_x(wings, 'Wing_Root_Profile', PLOW_CENTER, inner[0], inner[1], NEW_BODY).bodies.item(0)
        extrude_polygon_x(wings, 'Wing_Outer_Profile', PLOW_WING, outer[0], outer[1], JOIN, body)
        body.name = 'Plow_Wing_' + ('L' if sign < 0 else 'R')
        body.material = aluminum
        # Up-stop: a block on the wing root that the wedge's rest tab meets at
        # 60 deg (tab corner reaches y=78.534, z=22.8), so the servo is never
        # the end stop.
        sx0, sx1 = sorted((sign * WING_INNER, sign * 45))
        extrude_polygon_x(wings, 'Up_Stop_Block', [(74, 23), (74, 27), (UP_STOP_FACE_Y, 27), (UP_STOP_FACE_Y, ramp_z(UP_STOP_FACE_Y))], sx0, sx1, JOIN, body)
        px0, px1 = sorted((sign * WING_INNER, sign * PIN_HALF))
        bore_x_span(wings, body, 'Pivot_Bore_6p4mm', PIVOT[0], PIVOT[1], 3.2, px0 - 0.5, px1)
        wing_bodies.append(body)
    wing_edges = part(front, 'Wing_Steel_Edges')
    for sign in (-1, 1):
        x0, x1 = sorted((sign * WING_INNER, sign * 84))
        body = extrude_polygon_x(wing_edges, 'Wing_Edge_Profile', STEEL_EDGE, x0, x1, NEW_BODY).bodies.item(0)
        body.name = 'Wing_Steel_Edge_' + ('L' if sign < 0 else 'R')
        body.material = steel
    # M3 x 20 from inside the bulkhead (head at y 67..70, below the front
    # motors) into 12 mm tapped holes in the wing roots. Holes are modelled at
    # the 3 mm nominal size; the real tap drill is 2.5 mm.
    for sign, body in zip((-1, 1), wing_bodies):
        for bx in BOLT_X:
            x = sign * bx
            cylinder_y(front, 'M3x20_Bolt_ENVELOPE_x%+.1f' % x, x, BOLT_Z, 67, 86, 1.5, steel)
            bore_y_span(bulkhead, bulkhead_body, 'Bolt_Clearance_x%+.1f' % x, x, BOLT_Z, 1.7, 69.5, 74.5)
            bore_y_span(wings, body, 'Bolt_Tapped_x%+.1f' % x, x, BOLT_Z, 1.5, 74, 86)

    # --- 04 lifting wedge --------------------------------------------------
    weapon = group(root, '04_Servo_Lifting_Wedge')
    lifter = part(weapon, 'Lifting_Wedge_6061')
    lifter_body = extrude_polygon_x(lifter, 'Lifter_Profile', LIFTER, -WEDGE_HALF, WEDGE_HALF, NEW_BODY).bodies.item(0)
    lifter_body.name = 'Lifting_Wedge_79mm'
    lifter_body.material = aluminum
    join_cylinder_x(lifter, 'Pivot_Boss', -WEDGE_HALF, WEDGE_HALF, PIVOT[0], PIVOT[1], 6)
    for x0, x1 in [(4, 6), (9, 11)]:
        extrude_polygon_x(lifter, 'Pushrod_Clevis', [(81, 17), (81, 28), (87, 28), (87, 17)], x0, x1, JOIN, lifter_body)
    # Rest tabs overhang the wings: impacts that push the wedge down land on
    # the wings and bulkhead, not on the servo.
    # A 3.5 mm web inside the wedge edge carries each tab; without it the tab
    # would only touch the wedge along a line.
    for sign in (-1, 1):
        wx0, wx1 = sorted((sign * (WEDGE_HALF - 3.5), sign * WEDGE_HALF))
        extrude_polygon_x(lifter, 'Down_Stop_Web', [(88, 12), (88, 19.6), (92, 19.6), (92, 12)], wx0, wx1, JOIN, lifter_body)
        tx0, tx1 = sorted((sign * WEDGE_HALF, sign * 45))
        extrude_polygon_x(lifter, 'Down_Stop_Tab', [(88, ramp_z(88)), (88, 19.6), (92, 19.6), (92, ramp_z(92))], tx0, tx1, NEW_BODY)
    # The tabs only share a face with the wedge, so the extrude-join leaves
    # them as separate bodies; combine them into the one machined part.
    tools = adsk.core.ObjectCollection.create()
    for body in lifter.bRepBodies:
        if body != lifter_body:
            tools.add(body)
    if tools.count:
        combine = lifter.features.combineFeatures.createInput(lifter_body, tools)
        combine.operation = JOIN
        lifter.features.combineFeatures.add(combine)
    bore_x_in(lifter, lifter_body, 'Pivot_Bore_6p4mm', PIVOT[0], PIVOT[1], 3.2)
    bore_x_in(lifter, lifter_body, 'Pushrod_Pin_Bore_3p2mm', LUG_POINT[0], LUG_POINT[1], 1.6)
    edge = part(weapon, 'Lifter_Steel_Edge')
    edge_body = extrude_polygon_x(edge, 'Lifter_Edge_Profile', STEEL_EDGE, -WEDGE_HALF, WEDGE_HALF, NEW_BODY).bodies.item(0)
    edge_body.name = 'Lifter_Steel_Edge_1mm_Tip'
    edge_body.material = steel
    pin = cylinder_x(weapon, 'Pivot_Pin_6mm_ENVELOPE', -PIN_HALF, PIN_HALF, PIVOT[0], PIVOT[1], 3, steel)
    # Servo lies on its side in the centre bay; output face at x=3. Size is
    # BBB Shop's published maximum for this servo: 56.2 x 44.5 x 20 mm.
    servo = box(weapon, 'Repeat_40kg_Servo_56x44p5x20_MAX', -41.5, 3, -10, 46.2, 8, 28, proxy)
    # 2 mm foam pad under the servo and an aluminium strap over it, bolted
    # to two posts on the floor, hold it without loading the case ears.
    box(weapon, 'Servo_Foam_Pad_2mm', -41.5, 3, -10, 46.2, 6, 8, proxy)
    strap = box(weapon, 'Servo_Hold_Down_Strap_6061', -45, 8, 2, 10, 28, 30, aluminum)
    post_l = box(weapon, 'Servo_Strap_Post_L_6061', -45, -42, 2, 10, 6, 28, aluminum)
    post_r = box(weapon, 'Servo_Strap_Post_R_6061', 4, 8, 2, 10, 6, 28, aluminum)
    rest_pin = solve_linkage(0.0)[2]
    crank = linkage_bar_x(weapon, 'Servo_Crank_Arm_6061', 3, 6, SERVO_AXIS[0], SERVO_AXIS[1], rest_pin[0], rest_pin[1], 7, aluminum)
    join_cylinder_x(crank, 'Crank_Hub', 3, 6, SERVO_AXIS[0], SERVO_AXIS[1], 6)
    pushrod = linkage_bar_x(weapon, 'Pushrod_Steel_2mm', 6.5, 8.5, rest_pin[0], rest_pin[1], LUG_POINT[0], LUG_POINT[1], 6, steel)

    occurrence_for(root, tub).isGrounded = True
    for parent_c, child_c, name in [(bulkhead, wings, 'Wings_to_Bulkhead_Rigid'),
                                    (wings, wing_edges, 'Wing_Edges_to_Wings_Rigid'),
                                    (bulkhead, servo, 'Servo_to_Tub_Rigid'),
                                    (bulkhead, strap, 'Servo_Strap_to_Tub_Rigid'),
                                    (bulkhead, post_l, 'Servo_Post_L_to_Tub_Rigid'),
                                    (bulkhead, post_r, 'Servo_Post_R_to_Tub_Rigid'),
                                    (wings, pin, 'Pivot_Pin_to_Wings_Rigid'),
                                    (lifter, edge, 'Lifter_Edge_to_Lifter_Rigid')]:
        rigid = root.asBuiltJoints.createInput(all_occurrence_for(root, parent_c), all_occurrence_for(root, child_c), None)
        rigid.setAsRigidJointMotion()
        root.asBuiltJoints.add(rigid).name = name
    hinge = adsk.fusion.JointGeometry.createByNonPlanarFace(
        cylindrical_face(lifter, 3.2), adsk.fusion.JointKeyPointTypes.MiddleKeyPoint)
    joint_input = root.asBuiltJoints.createInput(all_occurrence_for(root, lifter), all_occurrence_for(root, wings), hinge)
    joint_input.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    lift_joint = root.asBuiltJoints.add(joint_input)
    lift_joint.name = 'Lifter_Revolute_0_to_%ddeg' % LIFT_MAX_DEG
    # Positive rotation raises the tip (checked in Fusion).
    limits = lift_joint.jointMotion.rotationLimits
    limits.isMinimumValueEnabled = True
    limits.minimumValue = 0
    limits.isMaximumValueEnabled = True
    limits.maximumValue = math.radians(LIFT_MAX_DEG)

    # --- 05 electronics: all in the centre bay, clear of the motor bands ---
    electronics = group(root, '05_Protected_Electronics_ENVELOPES')
    box(electronics, '3S_Battery_104x35x27_ENVELOPE', -52, 52, -48, -13, 8, 35, proxy)
    # Sizes from the makers: RadioMaster ER6 43x25x15 (stood on its 15 mm
    # edge, antennas up), BBB Dual ESC v2 20x16x5 PCB (envelope allows for
    # wires), Hobbywing UBEC 5A 50x17x10 set to 7.4 V for the servo and RX.
    box(electronics, 'RadioMaster_ER6_Receiver_43x25x15', -60, -45, -4, 39, 6, 31, proxy)
    box(electronics, 'BBB_Dual_ESC_v2_ENVELOPE', 18, 42, 24, 44, 6, 11, proxy)
    box(electronics, 'Hobbywing_UBEC_5A_50x17x10', 12, 62, -12, 5, 6, 16, proxy)
    box(electronics, 'Power_Switch_ENVELOPE', 44, 62, 18, 38, 9, 25, proxy)

    cover = group(root, '06_Removable_Covers')
    for side, x in [('L', -55), ('R', 55)]:
        for position, y in [('Rear', -75), ('Front', 43)]:
            box(cover, 'Cover_Standoff_' + side + '_' + position, x - 3, x + 3, y - 3, y + 3, 6, 37, polymer)
    box(cover, 'Rear_Service_Lid_2mm', -61, 61, -78, -39, 37, 39, polymer)
    box(cover, 'Center_Service_Lid_2mm', -61, 61, 4, 52, 37, 39, polymer)

    export = design.exportManager
    f3d = os.path.join(OUT, 'ROBOT_V3_R3_Lifter.f3d')
    step = os.path.join(OUT, 'ROBOT_V3_R3_Lifter.step')
    if not export.execute(export.createFusionArchiveExportOptions(f3d)):
        raise RuntimeError('F3D export failed')
    if not export.execute(export.createSTEPExportOptions(step)):
        raise RuntimeError('STEP export failed')
    app.activeViewport.fit()
    print(json.dumps({
        'f3d': f3d, 'step': step,
        'all_occurrences': root.allOccurrences.count,
        'cad_total_mass_g_including_envelopes': round(root.physicalProperties.mass * 1000, 3),
        'bbox_mm': [
            [round(v * 10, 3) for v in (root.boundingBox.minPoint.x, root.boundingBox.minPoint.y, root.boundingBox.minPoint.z)],
            [round(v * 10, 3) for v in (root.boundingBox.maxPoint.x, root.boundingBox.maxPoint.y, root.boundingBox.maxPoint.z)]
        ],
        'pushrod_length_mm': round(pushrod_length(), 2),
        'linkage': linkage_table(),
    }, ensure_ascii=False))
