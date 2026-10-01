"""Read-only assembly facts for an impact-readiness review."""
import adsk.core
import adsk.fusion
import json


KEY_BODIES = {
    'Base_Plate_Al', 'Left_Side_Plate_Al', 'Right_Side_Plate_Al',
    'Top_Cover', 'Front_Wedge_L', 'Front_Wedge_R',
    'Lifter_Spatula_62mm', 'Lifter_Hinge_Boss',
    'Lifter_Pivot_Support_L', 'Lifter_Pivot_Support_R',
    'Lifter_Pivot_Pin_6mm', 'Servo_30kgcm_Envelope_Only',
    'Lifter_Servo_Horn_Envelope_Only', 'Lifter_Linkage_Envelope_Only',
}


def mm_box(body):
    box = body.boundingBox
    return [[round(getattr(point, axis) * 10, 2) for axis in 'xyz']
            for point in (box.minPoint, box.maxPoint)]


def run(_context):
    app = adsk.core.Application.get()
    if app.activeDocument.name != 'robot2':
        raise RuntimeError('robot2 must be active')
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    groups = []
    parts = []
    for group in root.occurrences:
        groups.append({'name': group.component.name,
                       'visible': group.isVisible,
                       'joints': group.component.joints.count,
                       'as_built_joints': group.component.asBuiltJoints.count})
        for child in group.childOccurrences:
            for body in child.bRepBodies:
                if body.name not in KEY_BODIES:
                    continue
                parts.append({
                    'name': body.name,
                    'group': group.component.name,
                    'bbox_mm': mm_box(body),
                    'material': body.material.name if body.material else None,
                    'mass_g': round(body.physicalProperties.mass * 1000, 2),
                })
    print(json.dumps({
        'document_saved': not app.activeDocument.isModified,
        'groups': groups,
        'root_joint_count': root.joints.count,
        'root_as_built_joint_count': root.asBuiltJoints.count,
        'parts': parts,
    }))
