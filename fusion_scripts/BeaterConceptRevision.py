"""One-time conceptual beater rotor for the saved robot2 assembly.

This is a packaging and mass study. The 4 mm shaft fit, torque transfer,
impact strength, balance and purchased hardware are deliberately unverified.
"""

import adsk.core
import adsk.fusion
import json
import math


def cm(mm):
    return mm / 10.0


def point(sketch, x_mm, y_mm, z_mm):
    return sketch.modelToSketchSpace(
        adsk.core.Point3D.create(cm(x_mm), cm(y_mm), cm(z_mm)))


def box_sketch(component, name, x0, y0, x1, y1):
    sketch = component.sketches.add(component.xYConstructionPlane)
    sketch.name = name
    sketch.sketchCurves.sketchLines.addTwoPointRectangle(
        point(sketch, x0, y0, 0), point(sketch, x1, y1, 0))
    return sketch


def world_box(body):
    box = body.boundingBox
    return [[round(box.minPoint.x*10, 2), round(box.minPoint.y*10, 2),
             round(box.minPoint.z*10, 2)],
            [round(box.maxPoint.x*10, 2), round(box.maxPoint.y*10, 2),
             round(box.maxPoint.z*10, 2)]]


def base_box(root):
    for occ in root.allOccurrences:
        for body in occ.bRepBodies:
            if body.name == 'Base_Plate_Al':
                return world_box(body)
    raise RuntimeError('Base_Plate_Al not found')


def re_align_if_needed(root):
    current = base_box(root)
    aligned = [[-80.0, 3.0, -90.0], [80.0, 5.0, 90.0]]
    native = [[-80.0, -90.0, 3.0], [80.0, 90.0, 5.0]]
    if current == aligned:
        return 'already_aligned'
    if current != native:
        raise RuntimeError('Unexpected base orientation after editing: %r' % current)
    transform = adsk.core.Matrix3D.create()
    transform.setToRotation(-math.pi/2, adsk.core.Vector3D.create(1, 0, 0),
                            adsk.core.Point3D.create(0, 0, 0))
    groups = list(root.occurrences)
    if not root.transformOccurrences(groups, [transform for _ in groups], True):
        raise RuntimeError('Could not align root groups')
    if base_box(root) != aligned:
        raise RuntimeError('Alignment verification failed')
    return 'aligned_after_edit'


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design or app.activeDocument.name != 'robot2':
        raise RuntimeError('Open robot2 first')
    root = design.rootComponent
    groups = [o for o in root.occurrences if o.component.name == '03_Front_Spinner']
    if len(groups) != 1:
        raise RuntimeError('Expected one front spinner group')
    spinner = groups[0].component
    old_bar = [o for o in spinner.occurrences if o.component.name == 'Spinner_Bar']
    old_sweep = [o for o in spinner.occurrences if o.component.name == 'Spinner_Sweep_Envelope']
    if len(old_bar) != 1 or len(old_sweep) != 1:
        raise RuntimeError('Expected one old bar and one sweep envelope')
    if any(o.component.name.startswith('Beater_') for o in spinner.occurrences):
        raise RuntimeError('Beater concept already exists')
    bar_body = old_bar[0].component.bRepBodies.itemByName('Spinner_Bar')
    if not bar_body:
        raise RuntimeError('Old spinner bar body missing')
    old_mass_g = bar_body.physicalProperties.mass * 1000
    bar_material = bar_body.material

    # Native coordinates: x is shaft axis, (y,z) is the rotation plane.
    # Left support ends at x=-22.5; right-hand pulley begins at x=16.
    # The 34 mm rotor leaves 2.5 mm left and 2 mm right packaging clearances.
    rotor_occ = spinner.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    rotor_comp = rotor_occ.component
    rotor_comp.name = 'Beater_Rotor_Concept'
    outline = box_sketch(rotor_comp, 'Beater_Outer_65x20', -20, 42.5, 14, 107.5)
    outer = rotor_comp.features.extrudeFeatures.createInput(
        outline.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    outer.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(cm(28.5)))
    outer.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(20)))
    rotor = rotor_comp.features.extrudeFeatures.add(outer).bodies.item(0)
    rotor.name = 'Beater_Rotor_Concept'

    # Through-window leaves 4 mm side cheeks and two 10 mm radial impact rails.
    # It is open on both faces, rather than a trapped internal void.
    window = box_sketch(rotor_comp, 'Beater_Through_Window', -16, 52.5, 10, 97.5)
    opening = rotor_comp.features.extrudeFeatures.createInput(
        window.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    opening.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(cm(28.0)))
    opening.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(21)))
    opening.participantBodies = [rotor]
    rotor_comp.features.extrudeFeatures.add(opening)

    # The hole gives a preliminary fit on the existing shaft; no torque joint
    # is implied by the CAD bore.
    bore_sketch = rotor_comp.sketches.add(rotor_comp.yZConstructionPlane)
    bore_sketch.name = 'Beater_Shaft_Bore_4mm_Provisional'
    bore_sketch.sketchCurves.sketchCircles.addByCenterRadius(
        point(bore_sketch, 0, 75, 38.5), cm(2.0))
    bore = rotor_comp.features.extrudeFeatures.createInput(
        bore_sketch.profiles.item(0),
        adsk.fusion.FeatureOperations.CutFeatureOperation)
    bore.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(80)), True)
    bore.participantBodies = [rotor]
    rotor_comp.features.extrudeFeatures.add(bore)
    rotor.material = bar_material

    sweep_occ = spinner.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    sweep_comp = sweep_occ.component
    sweep_comp.name = 'Beater_Sweep_Envelope'
    circle = sweep_comp.sketches.add(sweep_comp.yZConstructionPlane)
    circle.name = 'Beater_Rotation_Radius_32p5'
    circle.sketchCurves.sketchCircles.addByCenterRadius(
        point(circle, 0, 75, 38.5), cm(32.5))
    swept = sweep_comp.features.extrudeFeatures.createInput(
        circle.profiles.item(0),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    swept.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(cm(-20)))
    swept.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(34)))
    sweep = sweep_comp.features.extrudeFeatures.add(swept).bodies.item(0)
    sweep.name = 'Beater_Sweep_Envelope'
    sweep_occ.isLightBulbOn = False

    old_bar[0].deleteMe()
    old_sweep[0].deleteMe()
    alignment = re_align_if_needed(root)
    print(json.dumps({
        'document': app.activeDocument.name,
        'rotor_bbox_world_mm': world_box(rotor),
        'sweep_bbox_world_mm': world_box(sweep),
        'old_bar_mass_cad_g': round(old_mass_g, 3),
        'new_rotor_mass_cad_g': round(rotor.physicalProperties.mass*1000, 3),
        'material': rotor.material.name,
        'alignment': alignment,
        'note': 'concept only; torque joint, shaft, bearings, balance and impact loads unverified',
    }))
