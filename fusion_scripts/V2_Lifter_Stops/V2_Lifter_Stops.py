"""robot2 (V2): make the lifter stops carry load instead of the servo.

1. Lower stop: raise Lifter_Lower_Hard_Stop so its top matches the slope of
   the blade underside and the blade rests on it at 0 deg (it was 4.41 mm
   below, so hits on the tip went into the servo gears; see
   exports/v2_linkage_2026-09-30/V2_Linkage_Report.md).
2. Upper stop: new block Lifter_Upper_Stop_45deg on the RIGHT pivot support
   (x 16..32; the linkage is on the left). The front part of the hinge boss
   top face lands flat on it at exactly 45 deg of lift.

robot2 native axes, mm: x across, y toward the nose, z above the floor.
Lifter pivot P = (y 68, z 31.5). Boss y 64..74, z 27..36 at rest.
Run on a COPY of robot2_items_1_3_2026-09-24.f3d (Save As first). The script
checks the geometry before changing anything, then drives the lifter joint
through 0..45 deg, checks clearances and interference, returns it to 0 deg
and writes V2_Lifter_Stops_report.json next to this script. It does not save.
"""
import json
import math
import os
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
P = (68.0, 31.5)
LIFT_MAX = 45.0
# Blade underside at rest: line (70, 27) -> (112, 5.8).
def blade_under_z(y):
    return 27.0 - 21.2 * (y - 70.0) / 42.0
# Lower-stop top: follow the blade underside between y 82 and 89. The new
# block starts 1 mm inside the old stop (z 12) so the join merges them.
LOWER = [(82.0, 12.0), (89.0, 12.0), (89.0, blade_under_z(89.0)), (82.0, blade_under_z(82.0))]
LOWER_X = (-10.0, 10.0)


def rot(p, a_deg):
    a = math.radians(a_deg)
    dy, dz = p[0] - P[0], p[1] - P[1]
    return (P[0] + dy * math.cos(a) - dz * math.sin(a), P[1] + dy * math.sin(a) + dz * math.cos(a))


# Upper stop in the y-z plane. Q1->Q2 is the boss top face (z 36) at 45 deg,
# from where it rises above z 36.5 to the front-top corner (74, 36). The rest
# of the outline stays >= 0.5 mm above everything the boss/blade sweeps
# through 0..45 deg (max boss z 38.93 is that corner at 45 deg).
_Q2 = rot((74.0, 36.0), LIFT_MAX)
_k = (36.5 - P[1]) / math.sin(math.radians(45)) - 4.5          # boss-frame y' where face reaches z 36.5
_Q1 = rot((P[0] + _k, 36.0), LIFT_MAX)
# Top at z 46 (5 mm above the support) for bending stiffness; nothing else
# occupies x 16..32, y 63.5..72, z 41..46 (the lever and linkage are at x < -25).
UPPER = [(63.5, 36.5), _Q1, _Q2, (72.0, 39.5), (72.0, 46.0), (63.5, 46.0)]
UPPER_X = (16.0, 32.0)       # touches the support face at x 32; clear of clamp post R (x <= 15)


def cm(v):
    return v / 10.0


def occ(root, name):
    found = [o for o in root.allOccurrences if o.component.name == name]
    if len(found) != 1:
        raise RuntimeError('expected one component named %s, found %d' % (name, len(found)))
    return found[0]


def first_body(o):
    if o.bRepBodies.count < 1:
        raise RuntimeError(o.component.name + ' has no body')
    return o.bRepBodies.item(0)          # proxy in root context


def bbox_mm(body):
    b = body.boundingBox
    return [[round(b.minPoint.x * 10, 2), round(b.minPoint.y * 10, 2), round(b.minPoint.z * 10, 2)],
            [round(b.maxPoint.x * 10, 2), round(b.maxPoint.y * 10, 2), round(b.maxPoint.z * 10, 2)]]


def near(a, b, tol=0.3):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def is_identity(o):
    m = o.transform2
    return all(abs(m.getCell(i, j) - (1.0 if i == j else 0.0)) < 1e-4 for i in range(3) for j in range(4))


def find_joint(root, name):
    comps = [root] + [o.component for o in root.allOccurrences]
    seen = set()
    for c in comps:
        if c.entityToken in seen:
            continue
        seen.add(c.entityToken)
        for j in c.asBuiltJoints:
            if j.name == name:
                return j
    raise RuntimeError('joint %s not found' % name)


def dist_mm(app, a, b):
    return app.measureManager.measureMinimumDistance(a, b).value * 10.0


def set_joint(joint, value_rad):
    joint.jointMotion.rotationValue = value_rad
    adsk.doEvents()


def prism_x(comp, name, poly_yz, x0, x1, operation, target=None):
    """Extrude a y-z polygon from x0 to x1 (mm) in comp."""
    planes = comp.constructionPlanes
    pin = planes.createInput()
    pin.setByOffset(comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(cm((x0 + x1) / 2)))
    plane = planes.add(pin)
    plane.name = name + '_Plane'
    plane.isLightBulbOn = False
    sk = comp.sketches.add(plane)
    sk.name = name + '_Profile'
    xm = (x0 + x1) / 2
    pts = [sk.modelToSketchSpace(adsk.core.Point3D.create(cm(xm), cm(y), cm(z))) for y, z in poly_yz]
    lines = sk.sketchCurves.sketchLines
    for i in range(len(pts)):
        lines.addByTwoPoints(pts[i], pts[(i + 1) % len(pts)])
    if sk.profiles.count != 1:
        raise RuntimeError(name + ': expected 1 profile, got %d' % sk.profiles.count)
    ext = comp.features.extrudeFeatures.createInput(sk.profiles.item(0), operation)
    ext.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(x1 - x0)), True)
    if target is not None:
        ext.participantBodies = [target]
    feat = comp.features.extrudeFeatures.add(ext)
    feat.name = name
    sk.isVisible = False
    return feat


def interferences(design, bodies, names):
    col = adsk.core.ObjectCollection.create()
    for b in bodies:
        col.add(b)
    inp = design.createInterferenceInput(col)
    inp.areCoincidentFacesIncluded = False
    out = []
    for r in design.analyzeInterference(inp):
        n1, n2 = r.entityOne.name, r.entityTwo.name
        if n1 in names or n2 in names:
            out.append([n1, n2, round(r.interferenceBody.volume * 1000, 4)])   # mm^3
    return out


def run(_):
    app = adsk.core.Application.get()
    ui = app.userInterface
    report = {'document': app.activeDocument.name if app.activeDocument else None}
    joint = None
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            raise RuntimeError('Open the robot2 design in the Design workspace first.')
        if design.designType != adsk.fusion.DesignTypes.ParametricDesignType:
            raise RuntimeError('Design must be parametric (timeline on).')
        root = design.rootComponent
        if any(b.name == 'Lifter_Upper_Stop_45deg' for o in root.allOccurrences for b in o.component.bRepBodies):
            raise RuntimeError('Already applied: Lifter_Upper_Stop_45deg exists.')

        stop_o = occ(root, 'Lifter_Lower_Hard_Stop')
        sup_o = occ(root, 'Lifter_Pivot_Support_R')
        blade_o = occ(root, 'Lifter_Spatula_62mm')
        boss_o = occ(root, 'Lifter_Hinge_Boss')
        joint = find_joint(root, 'Lifter_Main_Revolute')
        set_joint(joint, 0.0)

        # --- checks before any change ------------------------------------
        for o in (stop_o, sup_o, blade_o, boss_o):
            if not is_identity(o):
                raise RuntimeError(o.component.name + ' is not at its as-built position (transform not identity).')
        stop, sup, blade, boss = first_body(stop_o), first_body(sup_o), first_body(blade_o), first_body(boss_o)
        if not near(bbox_mm(stop)[0], [-10, 82, 5]) or not near(bbox_mm(stop)[1], [10, 89, 13]):
            raise RuntimeError('Lower stop is not the expected x-10..10 y82..89 z5..13: %s' % bbox_mm(stop))
        bb = bbox_mm(boss)
        if not (near([bb[1][0], bb[1][1], bb[0][2]], [30, 74, 27])):
            raise RuntimeError('Hinge boss not where expected: %s' % bb)
        gap0 = dist_mm(app, blade, stop)
        report['before'] = {'blade_to_lower_stop_mm': round(gap0, 3)}
        # 4.41 mm vertical gap at the stop's front corner; Fusion measures the
        # shortest distance, normal to the 26.8 deg blade underside: 3.94 mm.
        expected = (blade_under_z(89.0) - 13.0) * math.cos(math.atan(21.2 / 42.0))
        if abs(gap0 - expected) > 0.2:
            raise RuntimeError('Blade-to-stop distance is %.2f mm, expected %.2f. Geometry differs from the analysed CAD.' % (gap0, expected))

        # Which joint sign lifts the tip?
        z_rest = bbox_mm(blade)[1][2]
        set_joint(joint, math.radians(-10))
        lift_sign = -1.0 if bbox_mm(blade)[1][2] > z_rest + 1 else 1.0
        set_joint(joint, 0.0)
        report['lift_sign_fusion'] = lift_sign

        # --- 1. lower stop ------------------------------------------------
        stop_native = stop_o.component.bRepBodies.item(0)
        prism_x(stop_o.component, 'Lower_Stop_Raised_To_Blade', LOWER, LOWER_X[0], LOWER_X[1],
                adsk.fusion.FeatureOperations.JoinFeatureOperation, stop_native)

        # --- 2. upper stop ------------------------------------------------
        sup_native = sup_o.component.bRepBodies.item(0)
        feat = prism_x(sup_o.component, 'Lifter_Upper_Stop_45deg', UPPER, UPPER_X[0], UPPER_X[1],
                       adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        up_native = feat.bodies.item(0)
        up_native.name = 'Lifter_Upper_Stop_45deg'
        up_native.material = sup_native.material
        up = next(b for b in sup_o.bRepBodies if b.name == 'Lifter_Upper_Stop_45deg')
        stop = first_body(stop_o)
        ub = bbox_mm(up)
        if not near([ub[0][0], ub[1][0]], list(UPPER_X)):
            raise RuntimeError('Upper stop landed at x %.1f..%.1f, expected 16..32 - undo (Ctrl+Z) and report.' % (ub[0][0], ub[1][0]))
        report['new_bodies'] = {'lower_stop_bbox_mm': bbox_mm(stop), 'upper_stop_bbox_mm': ub,
                                'upper_stop_yz_mm': [[round(a, 3), round(b, 3)] for a, b in UPPER]}

        # --- 3. sweep the joint and check -----------------------------------
        visible = [b for o in root.allOccurrences for b in o.bRepBodies if b.isVisible]
        visible += [b for b in root.bRepBodies if b.isVisible]
        watch = {'Lifter_Lower_Hard_Stop', 'Lifter_Upper_Stop_45deg'}
        stop_names = {stop.name, 'Lifter_Upper_Stop_45deg'}
        checks = []
        for deg in (0, 1, 15, 30, 44, 45):
            set_joint(joint, lift_sign * math.radians(deg))
            row = {'lift_deg': deg,
                   'blade_to_lower_stop_mm': round(dist_mm(app, first_body(blade_o), stop), 3),
                   'boss_to_upper_stop_mm': round(dist_mm(app, first_body(boss_o), up), 3),
                   'blade_to_upper_stop_mm': round(dist_mm(app, first_body(blade_o), up), 3),
                   'interference_with_stops_mm3': interferences(design, visible, stop_names | watch)}
            checks.append(row)
        report['checks'] = checks
        ok = (checks[0]['blade_to_lower_stop_mm'] < 0.05 and checks[1]['blade_to_lower_stop_mm'] > 0.2
              and checks[-2]['boss_to_upper_stop_mm'] > 0.02 and checks[-1]['boss_to_upper_stop_mm'] < 0.05
              and all(not r['interference_with_stops_mm3'] for r in checks))
        report['result'] = 'OK' if ok else 'CHECK - see checks'
    except Exception:
        report['error'] = traceback.format_exc()
    finally:
        if joint is not None:
            try:
                set_joint(joint, 0.0)
            except Exception:
                pass
        path = os.path.join(HERE, 'V2_Lifter_Stops_report.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2)
    if 'error' in report:
        ui.messageBox('V2 lifter stops: FAILED (nothing saved).\n\n' + report['error'][-900:])
        return
    lines = ['V2 lifter stops: %s' % report['result'], '',
             'deg | blade-lower | boss-upper | interference']
    for r in report['checks']:
        lines.append('%3d | %6.3f | %6.3f | %s' % (r['lift_deg'], r['blade_to_lower_stop_mm'],
                                                  r['boss_to_upper_stop_mm'], r['interference_with_stops_mm3'] or 'none'))
    lines += ['', 'Report: ' + path, 'Not saved - check the model, then Save As.']
    ui.messageBox('\n'.join(lines))
