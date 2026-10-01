"""Create the V3 CAD study in Autodesk Fusion and export local F3D/STEP.

Concept: 4WD low tub, separate full-width impact plow, protected clamp.
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


def plow(parent, mat):
    comp = part(parent, 'Bolt_On_Full_Width_Plow_6061')
    sketch = comp.sketches.add(comp.yZConstructionPlane)
    sketch.name = 'Plow_Ramp_Profile'
    polygon = [(74, 6), (74, 23), (112, 5), (112, 3), (104, 3)]
    lines = sketch.sketchCurves.sketchLines
    for i, (y, z) in enumerate(polygon):
        y2, z2 = polygon[(i + 1) % len(polygon)]
        lines.addByTwoPoints(point(sketch, 0, y, z), point(sketch, 0, y2, z2))
    extrudes = comp.features.extrudeFeatures
    inp = extrudes.createInput(sketch.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(-60)))
    extent(inp, 120)
    body = extrudes.add(inp).bodies.item(0)
    body.name = 'Plow_120mm_Replaceable'
    body.material = mat
    sketch.isVisible = False
    return comp


def run(_):
    app = adsk.core.Application.get()
    if not os.path.isdir(OUT):
        raise RuntimeError('Output directory missing: ' + OUT)
    aluminum = material(app, 'Aluminum 6061')
    steel = material(app, 'Steel')
    polymer = material(app, 'Nylon 6')
    proxy = material(app, 'ABS Plastic')
    rubber = material(app, 'Rubber')

    doc = app.activeDocument
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not (doc and not doc.isSaved and doc.name.startswith('Untitled') and design and design.rootComponent.allOccurrences.count == 0):
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    root = design.rootComponent

    tub = group(root, '01_Low_Tub_Chassis')
    box(tub, 'Base_6061_3mm', -68, 68, -82, 82, 3, 6, aluminum)
    for side, x0, x1 in [('L', -68, -65), ('R', 65, 68)]:
        box(tub, 'Side_Rail_' + side + '_6061', x0, x1, -82, 82, 6, 29, aluminum)
    box(tub, 'Rear_Cross_Member_6061', -65, 65, -82, -79, 6, 29, aluminum)
    front_bulkhead = box(tub, 'Front_Impact_Bulkhead_6061', -62, 62, 70, 74, 6, 29, aluminum)
    for side, x0, x1 in [('L', -60, -48), ('R', 48, 60)]:
        box(tub, 'Front_Shear_Key_' + side, x0, x1, 62, 70, 6, 24, aluminum)

    drive = group(root, '02_Four_Wheel_Drive')
    for side in ('L', 'R'):
        for location, y in [('Rear', -58), ('Front', 58)]:
            tag = side + '_' + location
            mx0, mx1 = (-65, -16) if side == 'L' else (16, 65)
            wx0, wx1 = (-80, -68) if side == 'L' else (68, 80)
            ax0, ax1 = (-71, -62) if side == 'L' else (62, 71)
            cylinder_x(drive, 'BBB_22mm_Gearmotor_ENVELOPE_' + tag, mx0, mx1, y, 23, 11, proxy)
            cylinder_x(drive, 'Wheel_42mm_ENVELOPE_' + tag, wx0, wx1, y, 23, 21, rubber)
            cylinder_x(drive, 'Axle_4mm_ENVELOPE_' + tag, ax0, ax1, y, 23, 2, steel)
    for side, x0, x1 in [('L', -82, -80), ('R', 80, 82)]:
        box(drive, 'Replaceable_Side_Guard_' + side, x0, x1, -29, 29, 8, 28, polymer)

    front = group(root, '03_Front_Impact_Module')
    plow(front, aluminum)
    for side, x0, x1 in [('L', -54, -44), ('R', 44, 54)]:
        box(front, 'Plow_Bolt_Land_' + side, x0, x1, 65, 74, 6, 22, aluminum)

    clamp = group(root, '04_Protected_Clamp_Weapon')
    jaw = box(clamp, 'Clamp_Jaw_Plate_6061', -32, 32, 65, 106, 38, 42, aluminum)
    join_cylinder_x(jaw, 'Hinge_Boss', -32, 32, 65, 38.5, 6)
    bore_x(jaw, 'Hinge_Bore_6p4mm', 65, 38.5, 3.2)
    # A downturned crossbar and replaceable teeth make the moving member an
    # obvious active clamp/lifter rather than a decorative top plate.
    clamp_tip = box(clamp, 'Clamp_Downturned_Tip_6061', -32, 32, 101, 108, 18, 42, aluminum)
    clamp_teeth = []
    for index, x in enumerate((-24, 0, 24), start=1):
        clamp_teeth.append(box(clamp, 'Replaceable_Clamp_Tooth_%d_Steel' % index,
                               x - 4, x + 4, 106, 112, 9, 20, steel))
    torque_arm_l = box(clamp, 'Clamp_Torque_Arm_L', -34.5, -32.5, 61, 78, 36, 49, aluminum)
    torque_arm_r = box(clamp, 'Clamp_Torque_Arm_R', 32.5, 34.5, 61, 78, 36, 49, aluminum)
    bore_x(torque_arm_l, 'Torque_Arm_Pin_Bore_L', 65, 38.5, 3.2)
    bore_x(torque_arm_r, 'Torque_Arm_Pin_Bore_R', 65, 38.5, 3.2)
    cheeks = {}
    hard_stops = {}
    for side, x0, x1 in [('L', -42, -35), ('R', 35, 42)]:
        cheek = box(clamp, 'Hinge_Cheek_' + side, x0, x1, 56, 75, 28, 48, aluminum)
        cheeks[side] = cheek
        bore_x(cheek, 'Pin_Bore_6p4mm', 65, 38.5, 3.2)
        hard_stops[side] = box(clamp, 'Downward_Hard_Stop_' + side, x0, x1, 77, 87, 26, 36, aluminum)
    pin = cylinder_x(clamp, 'Hinge_Pin_6mm_ENVELOPE', -45, 45, 65, 38.5, 3, steel)
    servo = box(clamp, 'Repeat_40kg_Servo_ENVELOPE', -28, 28, 8, 53, 8, 28, proxy)
    horn = cylinder_x(clamp, 'Servo_Output_Horn_6061', -34, 34, 51, 24, 6, aluminum)
    bore_x(horn, 'Servo_Output_Bore_3mm', 51, 24, 1.5)
    linkage_l = linkage_bar_x(clamp, 'Clamp_Linkage_L', -40, -36, 51, 28, 72, 45, 5, steel)
    linkage_r = linkage_bar_x(clamp, 'Clamp_Linkage_R', 36, 40, 51, 28, 72, 45, 5, steel)
    box(clamp, 'Servo_Linkage_Sweep_ENVELOPE', -9, 9, 48, 75, 28, 50, proxy).bRepBodies.item(0).isVisible = False

    # Define the primary weapon degree of freedom. The linkage plates are
    # manufacturing placeholders until the real servo horn geometry is known.
    occurrence_for(root, tub).isGrounded = True
    jaw_occ = all_occurrence_for(root, jaw)
    cheek_occ = all_occurrence_for(root, cheeks['L'])
    frame_pairs = [
        (front_bulkhead, cheeks['L']), (cheeks['L'], cheeks['R']),
        (cheeks['L'], servo), (cheeks['L'], pin),
        (cheeks['L'], hard_stops['L']), (cheeks['R'], hard_stops['R']),
    ]
    for parent_component, child_component in frame_pairs:
        rigid_input = root.asBuiltJoints.createInput(
            all_occurrence_for(root, parent_component),
            all_occurrence_for(root, child_component), None)
        rigid_input.setAsRigidJointMotion()
        rigid_joint = root.asBuiltJoints.add(rigid_input)
        rigid_joint.name = child_component.name + '_Frame_Rigid'
    hinge_geometry = adsk.fusion.JointGeometry.createByNonPlanarFace(
        cylindrical_face(jaw, 3.2), adsk.fusion.JointKeyPointTypes.MiddleKeyPoint)
    # Put the moving jaw first so changing rotationValue moves the jaw while
    # the cheek remains in the rigid frame chain.
    joint_input = root.asBuiltJoints.createInput(jaw_occ, cheek_occ, hinge_geometry)
    # The bore-derived joint geometry uses its local Z axis along the physical
    # hinge cylinder (world X), so ZAxisJointDirection is the hinge axis.
    joint_input.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    weapon_joint = root.asBuiltJoints.add(joint_input)
    weapon_joint.name = 'Clamp_Main_Revolute_0_to_55deg'
    limits = weapon_joint.jointMotion.rotationLimits
    limits.isMinimumValueEnabled = True
    limits.minimumValue = 0
    limits.isMaximumValueEnabled = True
    limits.maximumValue = math.radians(55)
    for child in [clamp_tip, torque_arm_l, torque_arm_r] + clamp_teeth:
        rigid_input = root.asBuiltJoints.createInput(jaw_occ, all_occurrence_for(root, child), None)
        rigid_input.setAsRigidJointMotion()
        rigid_joint = root.asBuiltJoints.add(rigid_input)
        rigid_joint.name = child.name + '_to_Jaw_Rigid'

    electronics = group(root, '05_Protected_Electronics_ENVELOPES')
    box(electronics, '3S_Battery_104x35x27_ENVELOPE', -52, 52, -35, 0, 8, 35, proxy)
    box(electronics, 'Dual_ESC_ENVELOPE', -12, 12, -72, -52, 8, 13, proxy)
    box(electronics, 'Receiver_ENVELOPE', -48, -20, -72, -54, 8, 14, proxy)
    box(electronics, 'UBEC_ENVELOPE', 20, 48, -72, -54, 8, 15, proxy)
    box(electronics, 'Power_Switch_ENVELOPE', 38, 56, -65, -45, 9, 25, proxy)

    cover = group(root, '06_Removable_Covers')
    for side, x in [('L', -55), ('R', 55)]:
        for position, y in [('Rear', -74), ('Front', 43)]:
            box(cover, 'Cover_Standoff_' + side + '_' + position, x - 3, x + 3, y - 3, y + 3, 6, 37, polymer)
    box(cover, 'Rear_Service_Lid_2mm', -61, 61, -77, -39, 37, 39, polymer)
    box(cover, 'Center_Service_Lid_2mm', -61, 61, 4, 52, 37, 39, polymer)

    # Every sketch and extrusion is constructed directly in Y-up world
    # coordinates. All occurrence transforms therefore remain identity.

    export = design.exportManager
    f3d = os.path.join(OUT, 'ROBOT_V3_Competition.f3d')
    step = os.path.join(OUT, 'ROBOT_V3_Competition.step')
    if not export.execute(export.createFusionArchiveExportOptions(f3d)):
        raise RuntimeError('F3D export failed')
    if not export.execute(export.createSTEPExportOptions(step)):
        raise RuntimeError('STEP export failed')
    app.activeViewport.fit()
    print(json.dumps({
        'f3d': f3d, 'step': step,
        'root': root.name,
        'root_occurrences': root.occurrences.count,
        'all_occurrences': root.allOccurrences.count,
        'cad_total_mass_g_including_envelopes': round(root.physicalProperties.mass * 1000, 3),
        'bbox_mm': [
            [round(v * 10, 3) for v in (root.boundingBox.minPoint.x, root.boundingBox.minPoint.y, root.boundingBox.minPoint.z)],
            [round(v * 10, 3) for v in (root.boundingBox.maxPoint.x, root.boundingBox.maxPoint.y, root.boundingBox.maxPoint.z)]
        ],
        'envelopes_are_not_manufacturing_parts': True,
    }, ensure_ascii=False))
