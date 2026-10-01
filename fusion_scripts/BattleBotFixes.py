# Author-Claude (ESP32 Mini BattleBot project)
# LEGACY SCRIPT — for the pre-competition model only. The current robot2 design
# has a FingerTech switch envelope and no top hammer. Running the old build steps
# there would recreate obsolete mock geometry and exports. A guard in apply_all
# stops this script before it changes that design.
# Description-Applies the historical BattleBot_Master fixes and re-exports STEP/STL/DXF.
#
# HOW TO RUN
#   Fusion 360 -> Utilities tab -> ADD-INS -> Scripts and Add-Ins -> Scripts
#   -> select "BattleBotFixes" -> Run.
#   The BattleBot_Master design (document "robot2") must be the ACTIVE document.
#
# WHAT IT DOES (each step is independent; a failure in one is logged and the rest continue)
#   1. Builds Spinner_Motor_Cradle  - the A2212 motor mount that was removed earlier.
#   2. Cuts M2 base-plate mounting holes under every floor-level electronics module.
#   3. Re-materials Spinner_Bar to 7075-T6 and recomputes weapon kinetic energy.
#   4. Recomputes mass + centre of mass, runs a full interference check.
#   5. Re-exports STEP / STL / DXF from the CURRENT geometry (DXF via face projection,
#      so the screw and roll-pin holes are actually in the flat patterns).
#   6. Saves the document and writes exports/fusion_run_report.json
#      + exports/Mass_Report_Generated.md
#
# The same code body also runs unchanged through the Autodesk Fusion MCP connector.

import adsk.core, adsk.fusion
import traceback, json, os, math, datetime

PROJECT_DIR = '/Users/jitwisutthobut/Desktop/robot'
EXPORT_DIR = os.path.join(PROJECT_DIR, 'exports')

# ------------------------------------------------------------------ helpers

def MM(v):
    """millimetres -> Fusion internal centimetres"""
    return v / 10.0


def CM(v):
    """Fusion internal centimetres -> millimetres"""
    return v * 10.0


class Report(object):
    def __init__(self):
        self.data = {
            'generated': datetime.datetime.now().isoformat(timespec='seconds'),
            'steps': {},
            'errors': [],
            'summary': [],
        }

    def step(self, name, payload):
        self.data['steps'][name] = payload

    def error(self, name, exc_text):
        self.data['errors'].append({'step': name, 'traceback': exc_text})

    def say(self, line):
        self.data['summary'].append(line)


def bbox_mm(body):
    bb = body.boundingBox
    return {
        'x': [round(CM(bb.minPoint.x), 2), round(CM(bb.maxPoint.x), 2)],
        'y': [round(CM(bb.minPoint.y), 2), round(CM(bb.maxPoint.y), 2)],
        'z': [round(CM(bb.minPoint.z), 2), round(CM(bb.maxPoint.z), 2)],
    }


def all_bodies(root):
    """[(componentName, bodyName, proxyBody)] in assembly (world) context."""
    out = []
    for b in root.bRepBodies:
        out.append((root.name, b.name, b))
    for occ in root.allOccurrences:
        for b in occ.bRepBodies:
            out.append((occ.component.name, b.name, b))
    return out


def find_body(root, name):
    for cn, bn, b in all_bodies(root):
        if bn == name:
            return b
    return None


def find_occ(root, comp_name):
    for occ in root.allOccurrences:
        if occ.component.name == comp_name:
            return occ
    return None


def boxes_overlap(a, b, tol_mm=0.0):
    """True when the two bboxes share more than tol_mm on every axis.
    tol_mm > 0 means faces that merely touch do not count as a collision."""
    for ax in ('x', 'y', 'z'):
        shared = min(a[ax][1], b[ax][1]) - max(a[ax][0], b[ax][0])
        if shared <= tol_mm:
            return False
    return True


def get_lib_material(app, name_substr, exact=None):
    libs = app.materialLibraries
    lib = None
    for i in range(libs.count):
        if libs.item(i).name == 'Fusion Material Library':
            lib = libs.item(i)
            break
    if lib is None:
        return None
    if exact:
        for j in range(lib.materials.count):
            if lib.materials.item(j).name == exact:
                return lib.materials.item(j)
        return None
    for j in range(lib.materials.count):
        if name_substr.lower() in lib.materials.item(j).name.lower():
            return lib.materials.item(j)
    return None


def sketch_pt(sk, wx_mm, wy_mm, wz_mm):
    """World mm -> sketch-space Point3D. Uses Fusion's own mapping, which is why
    this script does not repeat the u/v axis-mapping bugs of the earlier build."""
    p = adsk.core.Point3D.create(MM(wx_mm), MM(wy_mm), MM(wz_mm))
    return sk.modelToSketchSpace(p)


def has_sketch(comp, name):
    for s in comp.sketches:
        if s.name == name:
            return True
    return False


def space_delta_mm(root, comp_name, body_name=None):
    """Max gap between a component's own coordinates and its world (assembly
    context) coordinates.  0 means component space == world space, which is what
    every sketch_pt() call in this script assumes.  If a component ever picks up
    a real occurrence transform, hole positions would silently land in the wrong
    place - so the cutting steps check this first and abort rather than drill."""
    occ = find_occ(root, comp_name)
    if occ is None:
        return None
    comp = occ.component
    if comp.bRepBodies.count == 0:
        return None
    native = comp.bRepBodies.itemByName(body_name) if body_name else None
    if native is None:
        native = comp.bRepBodies.item(0)
    proxy = occ.bRepBodies.itemByName(native.name)
    if proxy is None:
        return None
    a, b = bbox_mm(native), bbox_mm(proxy)
    d = 0.0
    for ax in ('x', 'y', 'z'):
        d = max(d, abs(a[ax][0] - b[ax][0]), abs(a[ax][1] - b[ax][1]))
    return round(d, 4)


SPACE_TOL_MM = 0.05

# --------------------------------------------------- STEP 1: spinner cradle

CRADLE_NAME = 'Spinner_Motor_Cradle'
CRADLE_WALL = 3.0        # mm of aluminium around the motor can
CRADLE_FIT = 0.4         # mm radial clearance so the can drops in
CRADLE_BOLT_TAP_D = 2.5  # mm M3 tap drill (blind, tapped in the cradle)
CRADLE_BOLT_DEPTH = 8.0  # mm blind depth
CRADLE_BOLT_CLR_D = 3.4  # mm M3 clearance through the base plate
CRADLE_BOLT_INSET = 11.0 # mm bolt x offset from centreline
CRADLE_TOP_DROP = 8.0    # mm the saddle stops BELOW the motor axis, so the two
                         # legs keep real thickness instead of tapering to a
                         # feather edge at the equator (~55 deg of wrap each side)
CRADLE_SWEEP_GAP = 1.5   # mm minimum gap to the weapon sweep envelope
CRADLE_REAR_MARGIN = 0.0 # mm. Zero on purpose: the TT drive motor cans end at
                         # y = 10 mm and the spinner can starts at y = 12 mm, so
                         # there is no room for a rear wall. The cradle is flush
                         # with the back of the can and leaves a 2 mm gap.
CRADLE_FRONT_MARGIN = 3.0


def step_cradle(app, design, root, rep):
    """A2212 spinner-motor mount.

    The three earlier attempts all put a bracket BEHIND the motor (y < 12 mm),
    which is where Spinner_ESC (y -16..10) and Main_Disconnect (y -8..12) live -
    that corridor is full, which is why they kept colliding.  The volume directly
    UNDER the can is empty, so this builds a saddle cradle that rises from the
    base plate instead.  The region is re-probed at runtime and the build is
    abandoned (not forced) if anything is actually in the way.
    """
    out = {}

    motor = find_body(root, 'Spinner_Motor')
    base = find_body(root, 'Base_Plate_Al')
    if motor is None or base is None:
        out['status'] = 'skipped - Spinner_Motor or Base_Plate_Al not found'
        rep.step('1_cradle', out)
        return

    d = space_delta_mm(root, 'Base_Plate_Al')
    out['base_plate_space_delta_mm'] = d
    if d is not None and d > SPACE_TOL_MM:
        out['status'] = ('ABORTED - Base_Plate_Al sits %.3f mm off its own component '
                         'origin; bolt holes would be drilled in the wrong place' % d)
        rep.step('1_cradle', out)
        return

    mb = bbox_mm(motor)
    bb = bbox_mm(base)
    base_top = bb['z'][1]
    axis_y = (mb['y'][0] + mb['y'][1]) / 2.0
    axis_z = (mb['z'][0] + mb['z'][1]) / 2.0
    can_r = (mb['z'][1] - mb['z'][0]) / 2.0

    # A bbox Z-extent only equals the can diameter while Spinner_Motor is a bare
    # cylinder. The design already carries an A2212 bolt pattern, so the day
    # someone models the mounting lugs the bbox grows and the saddle would
    # silently open up around a loose can. Trust the stated parameter instead.
    par = design.userParameters.itemByName('spinner_motor_D')
    if par is not None:
        stated_r = CM(par.value) / 2.0
        out['can_radius_check'] = {
            'from_bbox_mm': round(can_r, 3),
            'from_spinner_motor_D_mm': round(stated_r, 3),
            'delta_mm': round(abs(can_r - stated_r), 3),
        }
        if abs(can_r - stated_r) > 0.25:
            out['can_radius_check']['action'] = (
                'bbox disagrees with spinner_motor_D - using the parameter; check '
                'whether Spinner_Motor has grown features beyond the bare can')
            can_r = stated_r
        else:
            out['can_radius_check']['action'] = 'agree - using measured value'
    x0, x1 = mb['x'][0], mb['x'][1]          # cradle spans the can length
    block_top = axis_z - CRADLE_TOP_DROP

    # y: flush at the back (TT motor cans are right there), a wall at the front,
    # and never reaching the weapon sweep
    y0 = mb['y'][0] - CRADLE_REAR_MARGIN
    y1 = mb['y'][1] + CRADLE_FRONT_MARGIN
    sweep = find_body(root, 'Spinner_Sweep_Envelope')
    if sweep is not None:
        y1 = min(y1, bbox_mm(sweep)['y'][0] - CRADLE_SWEEP_GAP)

    out['measured'] = {
        'motor_bbox_mm': mb, 'base_plate_top_mm': base_top,
        'axis_y_mm': round(axis_y, 2), 'axis_z_mm': round(axis_z, 2),
        'can_radius_mm': round(can_r, 2),
        'block_top_mm': round(block_top, 2),
        'block_y_mm': [round(y0, 2), round(y1, 2)],
    }

    if axis_z - can_r <= base_top + 2.0:
        out['status'] = 'skipped - no room between base plate top and motor can'
        rep.step('1_cradle', out)
        return
    if block_top <= base_top + 2.0 or y1 - y0 < can_r:
        out['status'] = 'skipped - not enough room for a sensible cradle footprint'
        rep.step('1_cradle', out)
        return

    # ---- runtime clearance probe of the target volume -------------------
    target = {'x': [x0, x1], 'y': [y0, y1], 'z': [base_top, block_top]}
    ignore = {'Spinner_Motor', 'Base_Plate_Al', CRADLE_NAME}
    blockers = []
    for cn, bn, b in all_bodies(root):
        if bn in ignore:
            continue
        if boxes_overlap(target, bbox_mm(b), tol_mm=0.05):
            blockers.append({'body': bn, 'bbox_mm': bbox_mm(b)})
    out['probe_volume_mm'] = target
    out['blockers'] = blockers
    if blockers:
        out['status'] = ('ABANDONED - %d body/bodies occupy the volume under the '
                         'motor; cradle NOT built (see blockers)' % len(blockers))
        rep.step('1_cradle', out)
        rep.say('Cradle: abandoned, %s in the way' % ', '.join(b['body'] for b in blockers))
        return

    spin = find_occ(root, '03_Front_Spinner')
    if spin is None:
        out['status'] = 'skipped - 03_Front_Spinner component not found'
        rep.step('1_cradle', out)
        return
    spin_comp = spin.component

    # rebuild from scratch on every run so the result is deterministic
    for o in list(spin_comp.occurrences):
        if o.component.name == CRADLE_NAME:
            o.deleteMe()

    t = adsk.core.Matrix3D.create()
    occ = spin_comp.occurrences.addNewComponent(t)
    occ.component.name = CRADLE_NAME
    comp = occ.component

    # ---- 1a. the block ---------------------------------------------------
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Cradle_Block_Sketch'
    p0 = sketch_pt(sk, x0, y0, 0.0)
    p1 = sketch_pt(sk, x1, y1, 0.0)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(p0, p1)
    prof = sk.profiles.item(0)

    height = block_top - base_top

    def extrude_block(dist_mm):
        e = comp.features.extrudeFeatures.createInput(
            prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        e.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(MM(base_top)))
        e.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(dist_mm)))
        return comp.features.extrudeFeatures.add(e)

    def block_is_right(b):
        z = bbox_mm(b)['z']
        return abs(z[0] - base_top) < 0.5 and abs(z[1] - block_top) < 0.5

    feat = extrude_block(height)
    body = feat.bodies.item(0)
    if not block_is_right(body):
        # one flip attempt, then give up rather than leave a wrong solid behind
        feat.deleteMe()
        feat = extrude_block(-height)
        body = feat.bodies.item(0)
        if not block_is_right(body):
            out['block_bbox_mm'] = bbox_mm(body)
            out['status'] = ('FAILED - cradle block landed at the wrong height '
                             '(expected z %.1f..%.1f mm); nothing else was cut'
                             % (base_top, block_top))
            occ.deleteMe()
            rep.step('1_cradle', out)
            rep.say('Cradle: FAILED to orient the block extrude')
            return
    body.name = CRADLE_NAME
    out['block_bbox_mm'] = bbox_mm(body)

    # ---- 1b. saddle groove (cylinder along X, cut) -----------------------
    sk2 = comp.sketches.add(comp.yZConstructionPlane)
    sk2.name = 'Cradle_Saddle_Sketch'
    c = sketch_pt(sk2, 0.0, axis_y, axis_z)
    sk2.sketchCurves.sketchCircles.addByCenterRadius(c, MM(can_r + CRADLE_FIT))
    prof2 = sk2.profiles.item(0)
    cut = comp.features.extrudeFeatures.createInput(
        prof2, adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.setSymmetricExtent(
        adsk.core.ValueInput.createByReal(MM(abs(x1 - x0) + 20.0)), True)
    cut.participantBodies = [body]
    comp.features.extrudeFeatures.add(cut)
    sr = can_r + CRADLE_FIT
    half_at_top = math.sqrt(max(sr * sr - CRADLE_TOP_DROP ** 2, 0.0))
    out['saddle_radius_mm'] = round(sr, 2)
    out['leg_thickness_at_top_mm'] = [
        round((axis_y - half_at_top) - y0, 2),
        round(y1 - (axis_y + half_at_top), 2),
    ]
    out['wrap_half_angle_deg'] = round(
        math.degrees(math.acos(min(CRADLE_TOP_DROP / sr, 1.0))), 1)

    # ---- 1c. blind tapped M3 holes in the cradle bottom -------------------
    bolt_xy = [(-CRADLE_BOLT_INSET, y0 + 4.0), (CRADLE_BOLT_INSET, y0 + 4.0),
               (-CRADLE_BOLT_INSET, y1 - 4.0), (CRADLE_BOLT_INSET, y1 - 4.0)]
    sk3 = comp.sketches.add(comp.xYConstructionPlane)
    sk3.name = 'Cradle_Bolt_Sketch'
    for (bx, by) in bolt_xy:
        sk3.sketchCurves.sketchCircles.addByCenterRadius(
            sketch_pt(sk3, bx, by, 0.0), MM(CRADLE_BOLT_TAP_D / 2.0))
    hcol = adsk.core.ObjectCollection.create()
    for p in sk3.profiles:
        hcol.add(p)
    hcut = comp.features.extrudeFeatures.createInput(
        hcol, adsk.fusion.FeatureOperations.CutFeatureOperation)
    hcut.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(base_top)))
    hcut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(CRADLE_BOLT_DEPTH)))
    hcut.participantBodies = [body]
    comp.features.extrudeFeatures.add(hcut)

    mat = get_lib_material(app, '', exact='Aluminum 6061')
    if mat:
        body.material = mat

    # ---- 1d. matching M3 clearance holes through the base plate ----------
    base_occ = find_occ(root, 'Base_Plate_Al')
    drilled = 'no'
    if base_occ is not None:
        bcomp = base_occ.component
        if has_sketch(bcomp, 'Cradle_Bolt_Clearance_Sketch'):
            drilled = 'already present'
        else:
            bsk = bcomp.sketches.add(bcomp.xYConstructionPlane)
            bsk.name = 'Cradle_Bolt_Clearance_Sketch'
            for (bx, by) in bolt_xy:
                bsk.sketchCurves.sketchCircles.addByCenterRadius(
                    sketch_pt(bsk, bx, by, 0.0), MM(CRADLE_BOLT_CLR_D / 2.0))
            col = adsk.core.ObjectCollection.create()
            for p in bsk.profiles:
                col.add(p)
            bcut = bcomp.features.extrudeFeatures.createInput(
                col, adsk.fusion.FeatureOperations.CutFeatureOperation)
            bcut.setSymmetricExtent(adsk.core.ValueInput.createByReal(MM(30.0)), True)
            _bb = bcomp.bRepBodies.itemByName('Base_Plate_Al') or bcomp.bRepBodies.item(0)
            bcut.participantBodies = [_bb]
            bcomp.features.extrudeFeatures.add(bcut)
            drilled = 'yes'

    out['base_plate_clearance_holes'] = drilled
    out['bolt_positions_mm'] = [[round(a, 2), round(b, 2)] for a, b in bolt_xy]
    out['final_bbox_mm'] = bbox_mm(body)
    out['status'] = 'BUILT'
    rep.step('1_cradle', out)
    rep.say('Cradle: built, saddle r=%.2fmm, 4x M3 into base plate' % (can_r + CRADLE_FIT))


# ------------------------------------- STEP 2: electronics mounting in base

MODULES = ['ESP32_Control_Board', 'RC_Receiver', 'Wheel_Motor_Drivers',
           'Power_Buck_BEC', 'Main_Disconnect']
M2_CLEAR_D = 2.2         # mm, matches the m2_clear_d parameter already in the design
FLOOR_TOL = 1.5          # mm - how close to the base plate counts as "sitting on it"

# Modules that are already dealt with, so "not floor mounted" is the right
# answer for them rather than a gap to fill.
ALREADY_HANDLED = {
    'ESP32_Control_Board':
        'already carried on the 4 PCB_Standoff parts, with matching base-plate '
        'clearance holes cut earlier - nothing to add here',
    'Main_Disconnect':
        'deliberately raised onto a bracket to clear the TT motor cans - it needs '
        'that bracket fastened, not a base-plate hole underneath it',
}


def find_vertical_holes(body, dia_mm, tol_mm=0.3):
    """Centres (x_mm, y_mm) of vertical cylindrical faces of the given diameter."""
    want_r = MM(dia_mm) / 2.0
    seen = {}
    for f in body.faces:
        g = f.geometry
        if g.objectType != adsk.core.Cylinder.classType():
            continue
        cyl = adsk.core.Cylinder.cast(g)
        if abs(cyl.radius - want_r) > MM(tol_mm) / 2.0:
            continue
        if abs(cyl.axis.z) < 0.9:      # only holes drilled through the Z axis
            continue
        key = (round(CM(cyl.origin.x), 1), round(CM(cyl.origin.y), 1))
        seen[key] = True
    return sorted(seen.keys())


def step_module_mounts(app, design, root, rep):
    """The small electronics modules had their own M2 holes but nothing on the
    base plate to fasten into.

    A machined boss is not an option here: the base plate is 2 mm aluminium, so
    there is no material to raise a boss from, and a *printed* boss would itself
    need fastening to the plate - circular.  The buildable answer at this scale
    is a through-hole pair per module: M2 countersunk screw up from underneath,
    nut or nylock on top.  That is what this step cuts.
    """
    out = {'modules': {}}
    base = find_body(root, 'Base_Plate_Al')
    base_occ = find_occ(root, 'Base_Plate_Al')
    if base is None or base_occ is None:
        out['status'] = 'skipped - Base_Plate_Al not found'
        rep.step('2_module_mounts', out)
        return
    base_top = bbox_mm(base)['z'][1]

    d = space_delta_mm(root, 'Base_Plate_Al')
    out['base_plate_space_delta_mm'] = d
    if d is not None and d > SPACE_TOL_MM:
        out['status'] = ('ABORTED - Base_Plate_Al sits %.3f mm off its own component '
                         'origin; mounting holes would be drilled in the wrong place' % d)
        rep.step('2_module_mounts', out)
        return

    to_drill = []
    for name in MODULES:
        b = find_body(root, name)
        if b is None:
            out['modules'][name] = {'status': 'body not found'}
            continue
        mb = bbox_mm(b)
        holes = find_vertical_holes(b, M2_CLEAR_D)
        gap = mb['z'][0] - base_top
        info = {'bbox_mm': mb, 'holes_found': len(holes),
                'hole_centres_mm': [list(h) for h in holes],
                'gap_above_base_plate_mm': round(gap, 2)}
        if not holes:
            info['status'] = 'no M2 holes detected in this body - nothing to line up with'
        elif gap > FLOOR_TOL:
            if name in ALREADY_HANDLED:
                info['status'] = ('sits %.1f mm above the base plate - %s'
                                  % (gap, ALREADY_HANDLED[name]))
            else:
                info['status'] = ('NOT floor-mounted (sits %.1f mm above the base plate) - '
                                  'needs its own bracket or standoffs; no base-plate hole cut'
                                  % gap)
        else:
            info['status'] = 'floor-mounted - base-plate holes cut'
            to_drill.extend(holes)
        out['modules'][name] = info

    # dedupe across modules
    uniq = sorted(set(to_drill))
    out['total_holes_cut'] = len(uniq)
    out['hole_positions_mm'] = [list(h) for h in uniq]

    if not uniq:
        out['status'] = 'nothing to cut'
        rep.step('2_module_mounts', out)
        return

    bcomp = base_occ.component
    if has_sketch(bcomp, 'Module_Mount_Holes_Sketch'):
        out['status'] = 'already applied on a previous run - base plate left alone'
        rep.step('2_module_mounts', out)
        return

    sk = bcomp.sketches.add(bcomp.xYConstructionPlane)
    sk.name = 'Module_Mount_Holes_Sketch'
    for (hx, hy) in uniq:
        sk.sketchCurves.sketchCircles.addByCenterRadius(
            sketch_pt(sk, hx, hy, 0.0), MM(M2_CLEAR_D / 2.0))
    col = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        col.add(p)
    cut = bcomp.features.extrudeFeatures.createInput(
        col, adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.setSymmetricExtent(adsk.core.ValueInput.createByReal(MM(30.0)), True)
    _bb = bcomp.bRepBodies.itemByName('Base_Plate_Al') or bcomp.bRepBodies.item(0)
    cut.participantBodies = [_bb]
    bcomp.features.extrudeFeatures.add(cut)

    out['status'] = 'BUILT'
    out['assembly_note'] = ('M2 countersunk screw from below + nut on top. Counterbore '
                            'the underside 90 deg x 3.8 mm dia (<=1.2 mm deep in the 2 mm '
                            'plate) - not modelled; ground clearance is only 3 mm so a '
                            'pan head will protrude.')
    rep.step('2_module_mounts', out)
    rep.say('Module mounts: %d M2 holes cut through the base plate' % len(uniq))


# ------------------------------------------- STEP 3: weapon bar material

BAR_RPM_NO_LOAD = 15540.0   # A2212 1400 KV on 3S (11.1 V), unloaded


def step_bar_material(app, design, root, rep):
    """Spinner_Bar was left on mock Aluminium 6061.  Move it to 7075-T6 - the
    first option the safety checklist itself names - and recompute the energy.

    Steel is deliberately NOT chosen here: it roughly triples stored energy and
    puts the robot over its mass target.  Both numbers are reported so the
    trade is visible rather than made silently.
    """
    out = {}
    bar = find_body(root, 'Spinner_Bar')
    if bar is None:
        out['status'] = 'skipped - Spinner_Bar not found'
        rep.step('3_bar_material', out)
        return

    before = bar.physicalProperties.mass * 1000.0
    out['mass_before_g'] = round(before, 2)
    out['material_before'] = bar.material.name if bar.material else None

    # Already done? Then report and touch nothing - this keeps the step safe to
    # re-run, including from a read-only context where assignment would throw.
    if bar.material is not None and '7075' in bar.material.name:
        mat = bar.material
        how = 'already applied - left alone'
        out['material_after'] = mat.name
        out['resolved_by'] = how
        after = before
        out['mass_after_g'] = round(after, 2)
        _finish_bar_report(out, bar, before, after, rep)
        return

    mat = get_lib_material(app, '7075')
    how = 'library'
    if mat is None:
        base = get_lib_material(app, '', exact='Aluminum 6061')
        if base is None:
            out['status'] = 'FAILED - neither 7075 nor Aluminum 6061 found in the library'
            rep.step('3_bar_material', out)
            return
        mat = design.materials.addByCopy(base, 'Aluminum 7075-T6 (custom)')
        how = 'custom copy of 6061'
        set_ok = False
        props = mat.materialProperties
        # Scale whatever density value the copy already carries by 2810/2700.
        # Doing it as a ratio means we never have to know Fusion's internal
        # density unit - guessing there would silently corrupt every mass number.
        for i in range(props.count):
            pr = props.item(i)
            if 'density' not in pr.name.lower():
                continue
            fp = adsk.core.FloatProperty.cast(pr)
            if fp is None:
                continue
            try:
                before_val = fp.value
                fp.value = before_val * (2810.0 / 2700.0)
                set_ok = abs(fp.value - before_val) > 1e-12
                out['density_scaled_from_to'] = [before_val, fp.value]
            except Exception:
                pass
        how += (', density scaled 6061->7075 (x2810/2700)' if set_ok
                else ', DENSITY OVERRIDE FAILED - mass below is still 6061 density')
    bar.material = mat
    out['material_after'] = mat.name
    out['resolved_by'] = how

    after = bar.physicalProperties.mass * 1000.0
    out['mass_after_g'] = round(after, 2)

    _finish_bar_report(out, bar, before, after, rep)


def _finish_bar_report(out, bar, before, after, rep):
    bb = bbox_mm(bar)
    span = max(bb['x'][1] - bb['x'][0], bb['y'][1] - bb['y'][0], bb['z'][1] - bb['z'][0])
    out['bar_span_mm'] = round(span, 2)

    def ke(mass_g, length_mm, rpm):
        m = mass_g / 1000.0
        L = length_mm / 1000.0
        I = m * L * L / 12.0
        w = rpm * 2.0 * math.pi / 60.0
        return I, 0.5 * I * w * w

    I6, ke6 = ke(before, span, BAR_RPM_NO_LOAD)
    I7, ke7 = ke(after, span, BAR_RPM_NO_LOAD)
    steel_mass = before * (7.85 / 2.70)
    Is, kes = ke(steel_mass, span, BAR_RPM_NO_LOAD)

    out['kinetic_energy'] = {
        'assumed_rpm_no_load': BAR_RPM_NO_LOAD,
        'model': 'thin bar about its centre, I = m*L^2/12',
        '6061_mock': {'mass_g': round(before, 2), 'I_kgm2': I6, 'KE_J': round(ke6, 2)},
        '7075_T6': {'mass_g': round(after, 2), 'I_kgm2': I7, 'KE_J': round(ke7, 2)},
        'steel_if_chosen': {'mass_g': round(steel_mass, 2), 'I_kgm2': Is,
                            'KE_J': round(kes, 2),
                            'note': 'not applied - ~3x the energy and ~+40 g over budget'},
        'caveat': ('no-load RPM, no gearing/belt ratio applied, bar treated as a '
                   'uniform thin bar. Loaded RPM is lower. This is an order-of-'
                   'magnitude figure for rule-checking, not a validated number.'),
    }
    out['status'] = 'APPLIED'
    rep.step('3_bar_material', out)
    rep.say('Spinner_Bar: %s -> %s, %.1f g -> %.1f g, KE %.1f J -> %.1f J'
            % (out.get('material_before'), out.get('material_after'),
               before, after, ke6, ke7))


# ------------------------------------- STEP 4: mass, centre of mass, checks

def step_mass_and_checks(app, design, root, rep):
    out = {}
    rows = []
    totals = {}

    def rec(occ, top_name):
        s = 0.0
        for b in occ.bRepBodies:
            m = b.physicalProperties.mass * 1000.0
            rows.append({'subsystem': top_name, 'body': b.name,
                         'material': b.material.name if b.material else '-',
                         'mass_g': round(m, 2)})
            s += m
        for c in occ.childOccurrences:
            s += rec(c, top_name)
        return s

    for top in root.occurrences:
        totals[top.component.name] = round(rec(top, top.component.name), 2)

    out['by_body'] = rows
    out['by_subsystem_g'] = totals

    props = root.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
    com = props.centerOfMass
    out['model_total_mass_g'] = round(props.mass * 1000.0, 2)
    out['centre_of_mass_mm'] = [round(CM(com.x), 2), round(CM(com.y), 2), round(CM(com.z), 2)]

    # envelopes are analysis volumes, not parts - report the honest subtotal too
    ENVELOPES = ('Spinner_Sweep_Envelope', 'PCB_Keepout_Envelope',
                 'ESP32_Dupont_Keepout_A', 'ESP32_Dupont_Keepout_B')
    out['envelope_mass_g'] = round(
        sum(r['mass_g'] for r in rows if r['body'] in ENVELOPES), 2)
    out['real_body_mass_g'] = round(
        out['model_total_mass_g'] - out['envelope_mass_g'], 2)

    # ---- interference ----
    coll = adsk.core.ObjectCollection.create()
    for cn, bn, b in all_bodies(root):
        coll.add(b)
    ii = design.createInterferenceInput(coll)
    ii.areCoincidentFacesIncluded = False
    res = design.analyzeInterference(ii)
    # NOTE: r.interferenceBody.physicalProperties.volume reads back 0 here, which
    # for most of this session was misreported as "touching only". The real
    # overlap is measured with a temporary boolean intersection instead.
    tbm = adsk.fusion.TemporaryBRepManager.get()
    pairs = []
    for i in range(res.count):
        r = res.item(i)
        vol = None
        try:
            ta = tbm.copy(r.entityOne)
            tb = tbm.copy(r.entityTwo)
            tbm.booleanOperation(ta, tb, adsk.fusion.BooleanTypes.IntersectionBooleanType)
            vol = round(ta.volume * 1000.0, 3)
        except Exception:
            pass
        pairs.append({'a': r.entityOne.name, 'b': r.entityTwo.name, 'volume_mm3': vol})
    out['interference_count'] = res.count
    out['interference_pairs'] = pairs
    out['bodies_checked'] = coll.count

    rep.step('4_mass_and_checks', out)
    rep.say('Mass: %.1f g real bodies (%.1f g incl. envelopes), COM z=%.1f mm, %d interference pair(s)'
            % (out['real_body_mass_g'], out['model_total_mass_g'],
               out['centre_of_mass_mm'][2], res.count))
    return out


# ------------------------------------------------------ STEP 5: re-exports

PLATES = ['Base_Plate_Al', 'Left_Side_Plate_Al', 'Right_Side_Plate_Al',
          'Front_Wedge_L', 'Front_Wedge_R']


def flat_face(body):
    """Largest planar face whose normal runs along the body's thinnest axis -
    i.e. the flat-pattern face of a plate, holes and all."""
    bb = body.boundingBox
    ext = [bb.maxPoint.x - bb.minPoint.x,
           bb.maxPoint.y - bb.minPoint.y,
           bb.maxPoint.z - bb.minPoint.z]
    thin = ext.index(min(ext))
    best, best_area = None, -1.0
    for f in body.faces:
        g = f.geometry
        if g.objectType != adsk.core.Plane.classType():
            continue
        n = adsk.core.Plane.cast(g).normal
        comps = [abs(n.x), abs(n.y), abs(n.z)]
        if comps[thin] < 0.9:
            continue
        if f.area > best_area:
            best_area, best = f.area, f
    return best


def step_exports(app, design, root, rep):
    """Re-export everything from the CURRENT geometry.

    The old DXFs were exported from each plate's original profile sketch, which
    by definition cannot contain the clearance holes that were cut afterwards as
    separate features.  These are projected off the real face instead, so the
    screw holes are actually in the flat pattern.
    """
    out = {}
    em = design.exportManager
    os.makedirs(EXPORT_DIR, exist_ok=True)
    os.makedirs(os.path.join(EXPORT_DIR, 'STL'), exist_ok=True)
    os.makedirs(os.path.join(EXPORT_DIR, 'DXF'), exist_ok=True)

    # ---- STEP ----
    try:
        p = os.path.join(EXPORT_DIR, 'ESP32_Mini_BattleBot_Master.step')
        out['step'] = {'path': p, 'ok': em.execute(em.createSTEPExportOptions(p))}
    except Exception:
        out['step'] = {'error': traceback.format_exc()}

    # ---- STL (printed parts) ----
    stl = []
    try:
        printed = find_occ(root, '06_3D_Printed')
        if printed is None:
            stl.append({'error': '06_3D_Printed not found'})
        else:
            # walk the whole subtree: some printed parts sit a level down
            # (Rear_Panel lives under Non_Structural_Covers) and a single-level
            # loop silently skipped them.
            def descendants(o):
                for c in o.childOccurrences:
                    yield c
                    for g in descendants(c):
                        yield g

            for occ in descendants(printed):
                comp = occ.component
                if comp.bRepBodies.count == 0:
                    stl.append({'component': comp.name, 'skipped': 'no bodies'})
                    continue
                p = os.path.join(EXPORT_DIR, 'STL', comp.name + '.stl')
                o = em.createSTLExportOptions(occ, p)
                o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                stl.append({'component': comp.name, 'ok': em.execute(o), 'path': p})
    except Exception:
        stl.append({'error': traceback.format_exc()})
    # drop STLs of parts that no longer exist, so nobody prints a dead part
    try:
        live = set(s['component'] + '.stl' for s in stl if s.get('ok'))
        stl_dir = os.path.join(EXPORT_DIR, 'STL')
        stale = [f for f in os.listdir(stl_dir) if f.endswith('.stl') and f not in live]
        for f in stale:
            os.remove(os.path.join(stl_dir, f))
        out['stale_stl_removed'] = stale
    except Exception:
        out['stale_stl_removed'] = 'ERROR ' + traceback.format_exc().splitlines()[-1]
    out['stl'] = stl

    # ---- DXF (flat patterns, projected off the real face) ----
    dxf = []
    for name in PLATES:
        try:
            occ = find_occ(root, name)
            if occ is None:
                dxf.append({'part': name, 'error': 'component not found'})
                continue
            comp = occ.component
            body = comp.bRepBodies.itemByName(name)
            if body is None:
                body = comp.bRepBodies.item(0)
            face = flat_face(body)
            if face is None:
                dxf.append({'part': name, 'error': 'no suitable planar face'})
                continue
            skname = 'DXF_' + name
            for s in list(comp.sketches):
                if s.name == skname:
                    s.deleteMe()
            sk = comp.sketches.add(face)
            sk.name = skname
            try:
                sk.project(face)
            except Exception:
                pass
            if sk.sketchCurves.count == 0:
                for e in face.edges:
                    try:
                        sk.project(e)
                    except Exception:
                        pass
            p = os.path.join(EXPORT_DIR, 'DXF', name + '.dxf')
            ok = em.execute(em.createDXFSketchExportOptions(p, sk))
            dxf.append({'part': name, 'ok': ok, 'curves': sk.sketchCurves.count,
                        'loops': face.loops.count, 'path': p})
        except Exception:
            dxf.append({'part': name, 'error': traceback.format_exc()})
    out['dxf'] = dxf

    rep.step('5_exports', out)
    rep.say('Exports: STEP + %d STL + %d DXF rewritten'
            % (sum(1 for s in stl if s.get('ok')), sum(1 for d in dxf if d.get('ok'))))


# ---------------------------------------------- generated markdown report

def write_mass_markdown(rep):
    m = rep.data['steps'].get('4_mass_and_checks')
    if not m:
        return None
    bar = rep.data['steps'].get('3_bar_material', {})
    lines = []
    lines.append('# Mass & Interference — generated from the live Fusion model')
    lines.append('')
    lines.append('Written by `fusion_scripts/BattleBotFixes.py` on %s.'
                 % rep.data['generated'])
    lines.append('These numbers come from the model as it stands right now and')
    lines.append('supersede the CAD-computed table in `BOM_and_Mass_Report.md`.')
    lines.append('')
    lines.append('## Totals')
    lines.append('')
    lines.append('| Figure | Value |')
    lines.append('|---|---:|')
    lines.append('| Real bodies (excludes analysis envelopes) | %.1f g |' % m['real_body_mass_g'])
    lines.append('| Analysis envelopes (not parts) | %.1f g |' % m['envelope_mass_g'])
    lines.append('| Model total as Fusion reports it | %.1f g |' % m['model_total_mass_g'])
    com = m['centre_of_mass_mm']
    lines.append('| Centre of mass x / y / z | %.1f / %.1f / %.1f mm |' % (com[0], com[1], com[2]))
    lines.append('| Interference pairs | %d |' % m['interference_count'])
    lines.append('')
    lines.append('> Purchased parts are modelled as solid blocks, so their CAD mass is')
    lines.append('> not their real mass. Use the real masses in `BOM_and_Mass_Report.md`')
    lines.append('> for those and the figures here for fabricated parts.')
    lines.append('')
    lines.append('## By subsystem')
    lines.append('')
    lines.append('| Subsystem | Mass |')
    lines.append('|---|---:|')
    for k, v in sorted(m['by_subsystem_g'].items()):
        lines.append('| %s | %.1f g |' % (k, v))
    lines.append('')
    lines.append('## By body')
    lines.append('')
    lines.append('| Subsystem | Body | Material | Mass |')
    lines.append('|---|---|---|---:|')
    for r in m['by_body']:
        lines.append('| %s | %s | %s | %.2f g |'
                     % (r['subsystem'], r['body'], r['material'], r['mass_g']))
    lines.append('')

    if m['interference_pairs']:
        lines.append('## Interference pairs')
        lines.append('')
        lines.append('| A | B | Volume |')
        lines.append('|---|---|---:|')
        for p in m['interference_pairs']:
            lines.append('| %s | %s | %s mm³ |' % (p['a'], p['b'], p['volume_mm3']))
        lines.append('')

    ke = bar.get('kinetic_energy')
    if ke:
        lines.append('## Weapon energy (Spinner_Bar)')
        lines.append('')
        lines.append('Model: %s, at %.0f RPM no-load.' % (ke['model'], ke['assumed_rpm_no_load']))
        lines.append('')
        lines.append('| Material | Bar mass | Kinetic energy |')
        lines.append('|---|---:|---:|')
        for key, label in (('6061_mock', 'Al 6061 (old mock)'),
                           ('7075_T6', 'Al 7075-T6 (applied)'),
                           ('steel_if_chosen', 'Steel (NOT applied)')):
            row = ke[key]
            lines.append('| %s | %.1f g | %.1f J |' % (label, row['mass_g'], row['KE_J']))
        lines.append('')
        lines.append('%s' % ke['caveat'])
        lines.append('')
        lines.append('**This is a screening figure, not a safety clearance.** A weapon at')
        lines.append('this energy needs a rated containment box and a rules check against')
        lines.append('the specific event before it is spun at any speed.')
        lines.append('')

    path = os.path.join(EXPORT_DIR, 'Mass_Report_Generated.md')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    return path


# ----------------------------------------------------------------- driver

def apply_all(app, design, rep):
    """The whole job. Separated from run() so it also works through the MCP bridge."""
    root = design.rootComponent
    if any(occ.component.name == 'FingerTech_Mini_Switch_Envelope'
           for occ in root.allOccurrences):
        raise RuntimeError(
            'BattleBotFixes.py is a legacy script and cannot run on the '
            'competition variant. See STATUS.md and the one-time revision '
            'scripts in fusion_scripts/.')
    rep.data['document'] = app.activeDocument.name
    rep.data['root_component'] = root.name
    rep.data['space_check_mm'] = {
        'Base_Plate_Al': space_delta_mm(root, 'Base_Plate_Al'),
        'Spinner_Motor': space_delta_mm(root, 'Spinner_Motor'),
        'note': 'component-space vs world-space offset; must be ~0 for the hole cuts',
    }

    for label, fn in (('1_cradle', step_cradle),
                      ('2_module_mounts', step_module_mounts),
                      ('3_bar_material', step_bar_material),
                      ('4_mass_and_checks', step_mass_and_checks),
                      ('5_exports', step_exports)):
        try:
            fn(app, design, root, rep)
        except Exception:
            rep.error(label, traceback.format_exc())
            rep.say('%s FAILED - see errors in the JSON report' % label)

    try:
        app.activeDocument.save('BattleBotFixes: cradle, module mounts, 7075 bar, re-exports')
        rep.data['saved'] = True
    except Exception:
        rep.error('save', traceback.format_exc())
        rep.data['saved'] = False

    try:
        rep.data['mass_markdown'] = write_mass_markdown(rep)
    except Exception:
        rep.error('mass_markdown', traceback.format_exc())

    os.makedirs(EXPORT_DIR, exist_ok=True)
    out_path = os.path.join(EXPORT_DIR, 'fusion_run_report.json')
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(rep.data, f, indent=2, default=str)
    return out_path


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    rep = Report()
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('BattleBotFixes: no active Fusion design.\n'
                          'Open the BattleBot_Master document (robot2) and run again.')
            return
        out_path = apply_all(app, design, rep)
        msg = ['BattleBotFixes finished.', '']
        msg.extend(rep.data['summary'])
        if rep.data['errors']:
            msg.append('')
            msg.append('%d step(s) reported errors.' % len(rep.data['errors']))
        msg.append('')
        msg.append('Full report: ' + out_path)
        ui.messageBox('\n'.join(msg))
    except Exception:
        if ui:
            ui.messageBox('BattleBotFixes crashed:\n' + traceback.format_exc())


# ----------------------------- STEP 2b: the PCB's own holes were never cut

PCB_HOLE_D = 2.9         # mm, matches the Ø2.9 bore already in each PCB_Standoff


def step_pcb_holes(app, design, root, rep):
    """`ESP32_Control_Board` reports 4 mounting holes in the notes, but the solid
    has none.

    The earlier pass sketched them on the component's XY plane at z=0 and cut
    +/-4 mm from there - but the board had already been moved up onto the 3 mm
    standoffs, so it sits at z 8.0-9.6 mm. The cut missed the body entirely and
    removed nothing, while still reporting success.

    This cuts them where the board actually is, at the real standoff centres.
    """
    out = {}
    board = find_body(root, 'ESP32_Control_Board')
    occ = find_occ(root, 'ESP32_Control_Board')
    if board is None or occ is None:
        out['status'] = 'skipped - ESP32_Control_Board not found'
        rep.step('2b_pcb_holes', out)
        return

    out['holes_before'] = len(find_vertical_holes(board, PCB_HOLE_D, tol_mm=1.0))

    centres = []
    for cn, bn, b in all_bodies(root):
        if bn.startswith('PCB_Standoff'):
            bb = bbox_mm(b)
            centres.append((round((bb['x'][0] + bb['x'][1]) / 2.0, 2),
                            round((bb['y'][0] + bb['y'][1]) / 2.0, 2)))
    centres = sorted(set(centres))
    out['standoff_centres_mm'] = [list(c) for c in centres]
    if not centres:
        out['status'] = 'skipped - no PCB_Standoff bodies found to line up with'
        rep.step('2b_pcb_holes', out)
        return

    d = space_delta_mm(root, 'ESP32_Control_Board')
    out['space_delta_mm'] = d
    if d is not None and d > SPACE_TOL_MM:
        out['status'] = 'ABORTED - board sits %.3f mm off its component origin' % d
        rep.step('2b_pcb_holes', out)
        return

    comp = occ.component
    if has_sketch(comp, 'PCB_Mount_Holes_Real'):
        out['status'] = 'already applied on a previous run'
        rep.step('2b_pcb_holes', out)
        return

    bb = bbox_mm(board)
    z_lo, z_hi = bb['z'][0], bb['z'][1]
    out['board_z_mm'] = [z_lo, z_hi]

    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'PCB_Mount_Holes_Real'
    for (hx, hy) in centres:
        sk.sketchCurves.sketchCircles.addByCenterRadius(
            sketch_pt(sk, hx, hy, 0.0), MM(PCB_HOLE_D / 2.0))
    col = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        col.add(p)
    cut = comp.features.extrudeFeatures.createInput(
        col, adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(z_lo - 2.0)))
    cut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM((z_hi - z_lo) + 4.0)))
    cut.participantBodies = [comp.bRepBodies.itemByName('ESP32_Control_Board')
                             or comp.bRepBodies.item(0)]
    comp.features.extrudeFeatures.add(cut)

    # the real test: are the holes detectable in the solid now?
    board2 = find_body(root, 'ESP32_Control_Board')
    found = find_vertical_holes(board2, PCB_HOLE_D, tol_mm=1.0)
    out['holes_after'] = len(found)
    out['hole_centres_mm'] = [list(h) for h in found]
    out['status'] = 'BUILT' if len(found) >= len(centres) else 'CUT MADE BUT HOLES NOT DETECTED'
    rep.step('2b_pcb_holes', out)
    rep.say('PCB holes: %d -> %d (were silently missing)'
            % (out['holes_before'], out['holes_after']))


def plate_supports_bolt(base, x_mm, y_mm, z_mm, hole_d_mm, ring=1.5, n=8):
    """Is there base-plate material to land a bolt head/thread on at (x, y)?

    Tests a ring of points around the position rather than the axis itself,
    because on a re-run the axis is already a drilled hole and a point test
    there would report 'no material' and wrongly abandon the build."""
    import math as _m
    r = (hole_d_mm / 2.0) + ring
    for i in range(n):
        a = 2.0 * _m.pi * i / n
        p = adsk.core.Point3D.create(MM(x_mm + r * _m.cos(a)),
                                     MM(y_mm + r * _m.sin(a)),
                                     MM(z_mm))
        if base.pointContainment(p) != adsk.fusion.PointContainment.PointInsidePointContainment:
            return False
    return True

# ------------------------------------- STEP 6: TT drive motor mounts

TT_EDGE_GAP = 6.0        # mm clearance to the wheel and to the motor can
TT_SEAT_CLEAR = 0.5      # mm per side around the gearbox in the pocket
TT_CHEEK_T = 5.0         # mm cheek wall thickness (enough to tap M3 into the top)
TT_CLAMP_W = 10.0        # mm width of the clamp bar across the gearbox
TT_CLAMP_T = 3.0         # mm clamp bar thickness
TT_BOLT_TAP_D = 2.5      # mm M3 tap drill
TT_BOLT_CLR_D = 3.4      # mm M3 clearance
TT_BOLT_DEPTH = 5.0      # mm blind tap depth into the floor
TT_CLAMP_TAP_DEPTH = 8.0 # mm blind tap depth down into the cheeks
TT_CHEEK_DROP = 1.5      # mm the cheeks stop BELOW the gearbox top on purpose.
                         # If the cheek tops were flush with the gearbox, the
                         # clamp bar would bottom out on the cheeks and apply no
                         # load at all to the gearbox. Dropping them makes the
                         # bar bear on the gearbox first, so tightening the M3s
                         # produces real preload and the bar acts as the spring.


def _tt_side(app, design, root, rep, side, out):
    """Build one TT drive motor mount. side is 'L' or 'R'."""
    gb = find_body(root, 'TT_Gearbox_' + side)
    can = find_body(root, 'TT_Motor_Can_' + side)
    wheel = find_body(root, ('Left' if side == 'L' else 'Right') + '_Wheel')
    base = find_body(root, 'Base_Plate_Al')
    base_occ = find_occ(root, 'Base_Plate_Al')
    if None in (gb, can, wheel, base, base_occ):
        out['status'] = 'skipped - a required body was not found'
        return

    g, c, w = bbox_mm(gb), bbox_mm(can), bbox_mm(wheel)
    base_top = bbox_mm(base)['z'][1]

    if side == 'L':
        xlo, xhi = w['x'][1] + TT_EDGE_GAP, c['x'][0] - TT_EDGE_GAP
    else:
        xlo, xhi = c['x'][1] + TT_EDGE_GAP, w['x'][0] - TT_EDGE_GAP
    xlo, xhi = round(xlo, 2), round(xhi, 2)

    seat = (g['y'][1] - g['y'][0]) / 2.0 + TT_SEAT_CLEAR     # pocket half width
    outer = seat + TT_CHEEK_T                                 # cheek outer half width
    floor_top = g['z'][0]                                     # gearbox sits on the floor
    gearbox_top = g['z'][1]
    cheek_top = gearbox_top - TT_CHEEK_DROP                   # see TT_CHEEK_DROP
    cheek_mid = seat + TT_CHEEK_T / 2.0
    xmid = round((xlo + xhi) / 2.0, 2)

    out['measured'] = {
        'gearbox_bbox_mm': g, 'can_bbox_mm': c, 'wheel_bbox_mm': w,
        'base_plate_top_mm': base_top,
    }
    out['planned'] = {
        'x_mm': [xlo, xhi], 'outer_half_width_mm': round(outer, 2),
        'seat_half_width_mm': round(seat, 2),
        'floor_z_mm': [base_top, floor_top],
        'cheek_z_mm': [floor_top, round(cheek_top, 2)],
        'gearbox_top_mm': gearbox_top,
        'clamp_z_mm': [gearbox_top, gearbox_top + TT_CLAMP_T],
        'preload_gap_mm': TT_CHEEK_DROP,
    }
    if xhi - xlo < 10.0 or floor_top - base_top < 3.0:
        out['status'] = 'skipped - not enough room for a sensible mount'
        return

    # ---- clearance probe. The gearbox itself is the thing being held, so it is
    # not a blocker; nor is the base plate the mount stands on.
    target = {'x': [xlo, xhi], 'y': [-outer, outer],
              'z': [base_top, gearbox_top + TT_CLAMP_T]}
    ignore = {'Base_Plate_Al', 'TT_Gearbox_' + side,
              'TT_Motor_Mount_' + side, 'TT_Motor_Clamp_' + side}
    blockers = [{'body': bn, 'bbox_mm': bbox_mm(b)}
                for cn, bn, b in all_bodies(root)
                if bn not in ignore and boxes_overlap(target, bbox_mm(b), 0.05)]
    out['probe_volume_mm'] = target
    out['blockers'] = blockers
    if blockers:
        out['status'] = 'ABANDONED - %s in the way' % ', '.join(b['body'] for b in blockers)
        return

    # ---- base plate must actually have material under all four bolts
    bolts = [(xlo + 4.0, -cheek_mid), (xlo + 4.0, cheek_mid),
             (xhi - 4.0, -cheek_mid), (xhi - 4.0, cheek_mid)]
    bad = []
    zmid = (bbox_mm(base)['z'][0] + base_top) / 2.0
    for (bx, by) in bolts:
        if not plate_supports_bolt(base, bx, by, zmid, TT_BOLT_CLR_D):
            bad.append([round(bx, 2), round(by, 2)])
    out['bolt_positions_mm'] = [[round(a, 2), round(b, 2)] for a, b in bolts]
    out['bolts_outside_base_plate'] = bad
    if bad:
        out['status'] = 'ABANDONED - %d bolt position(s) fall in the wheel-well cutout' % len(bad)
        return

    dt = find_occ(root, '02_Drivetrain')
    if dt is None:
        out['status'] = 'skipped - 02_Drivetrain not found'
        return
    dtc = dt.component

    for nm in ('TT_Motor_Mount_' + side, 'TT_Motor_Clamp_' + side):
        for o in list(dtc.occurrences):
            if o.component.name == nm:
                o.deleteMe()

    mat = get_lib_material(app, '', exact='Aluminum 6061')

    # ---- the U-cradle -------------------------------------------------
    occ = dtc.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = 'TT_Motor_Mount_' + side
    comp = occ.component

    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'TT_Mount_Block_Sketch'
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        sketch_pt(sk, xlo, -outer, 0.0), sketch_pt(sk, xhi, outer, 0.0))
    ext = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(base_top)))
    ext.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(cheek_top - base_top)))
    feat = comp.features.extrudeFeatures.add(ext)
    body = feat.bodies.item(0)
    zb = bbox_mm(body)['z']
    if abs(zb[0] - base_top) > 0.5 or abs(zb[1] - cheek_top) > 0.5:
        out['block_bbox_mm'] = bbox_mm(body)
        out['status'] = 'FAILED - mount block landed at the wrong height'
        occ.deleteMe()
        return
    body.name = 'TT_Motor_Mount_' + side

    # ---- pocket the gearbox seat (cuts through in X, open at the top) --
    sk2 = comp.sketches.add(comp.xYConstructionPlane)
    sk2.name = 'TT_Mount_Pocket_Sketch'
    sk2.sketchCurves.sketchLines.addTwoPointRectangle(
        sketch_pt(sk2, xlo - 5.0, -seat, 0.0), sketch_pt(sk2, xhi + 5.0, seat, 0.0))
    cut = comp.features.extrudeFeatures.createInput(
        sk2.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(floor_top)))
    cut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(cheek_top - floor_top + 10.0)))
    cut.participantBodies = [body]
    comp.features.extrudeFeatures.add(cut)

    # ---- blind M3 taps in the floor bottom, and clearance in the plate --
    sk3 = comp.sketches.add(comp.xYConstructionPlane)
    sk3.name = 'TT_Mount_Bolt_Sketch'
    for (bx, by) in bolts:
        sk3.sketchCurves.sketchCircles.addByCenterRadius(
            sketch_pt(sk3, bx, by, 0.0), MM(TT_BOLT_TAP_D / 2.0))
    col = adsk.core.ObjectCollection.create()
    for pr in sk3.profiles:
        col.add(pr)
    hc = comp.features.extrudeFeatures.createInput(
        col, adsk.fusion.FeatureOperations.CutFeatureOperation)
    hc.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(base_top)))
    hc.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(TT_BOLT_DEPTH)))
    hc.participantBodies = [body]
    comp.features.extrudeFeatures.add(hc)

    # ---- blind taps down into the cheek tops for the clamp -------------
    sk4 = comp.sketches.add(comp.xYConstructionPlane)
    sk4.name = 'TT_Mount_ClampTap_Sketch'
    for sgn in (-1.0, 1.0):
        sk4.sketchCurves.sketchCircles.addByCenterRadius(
            sketch_pt(sk4, xmid, sgn * cheek_mid, 0.0), MM(TT_BOLT_TAP_D / 2.0))
    col4 = adsk.core.ObjectCollection.create()
    for pr in sk4.profiles:
        col4.add(pr)
    tc = comp.features.extrudeFeatures.createInput(
        col4, adsk.fusion.FeatureOperations.CutFeatureOperation)
    tc.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(cheek_top)))
    tc.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(-TT_CLAMP_TAP_DEPTH)))
    tc.participantBodies = [body]
    comp.features.extrudeFeatures.add(tc)
    if mat:
        body.material = mat
    out['mount_bbox_mm'] = bbox_mm(body)
    out['mount_mass_g'] = round(body.physicalProperties.mass * 1000.0, 2)

    # ---- the clamp bar -------------------------------------------------
    occ2 = dtc.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ2.component.name = 'TT_Motor_Clamp_' + side
    comp2 = occ2.component
    sk5 = comp2.sketches.add(comp2.xYConstructionPlane)
    sk5.name = 'TT_Clamp_Sketch'
    sk5.sketchCurves.sketchLines.addTwoPointRectangle(
        sketch_pt(sk5, xmid - TT_CLAMP_W / 2.0, -outer, 0.0),
        sketch_pt(sk5, xmid + TT_CLAMP_W / 2.0, outer, 0.0))
    e2 = comp2.features.extrudeFeatures.createInput(
        sk5.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    e2.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(gearbox_top)))
    e2.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(TT_CLAMP_T)))
    f2 = comp2.features.extrudeFeatures.add(e2)
    cbody = f2.bodies.item(0)
    cbody.name = 'TT_Motor_Clamp_' + side

    sk6 = comp2.sketches.add(comp2.xYConstructionPlane)
    sk6.name = 'TT_Clamp_Holes_Sketch'
    for sgn in (-1.0, 1.0):
        sk6.sketchCurves.sketchCircles.addByCenterRadius(
            sketch_pt(sk6, xmid, sgn * cheek_mid, 0.0), MM(TT_BOLT_CLR_D / 2.0))
    col6 = adsk.core.ObjectCollection.create()
    for pr in sk6.profiles:
        col6.add(pr)
    c6 = comp2.features.extrudeFeatures.createInput(
        col6, adsk.fusion.FeatureOperations.CutFeatureOperation)
    c6.setSymmetricExtent(adsk.core.ValueInput.createByReal(MM(200.0)), True)
    c6.participantBodies = [cbody]
    comp2.features.extrudeFeatures.add(c6)
    if mat:
        cbody.material = mat
    out['clamp_bbox_mm'] = bbox_mm(cbody)
    out['clamp_mass_g'] = round(cbody.physicalProperties.mass * 1000.0, 2)

    # ---- matching clearance holes through the base plate ---------------
    bcomp = base_occ.component
    skname = 'TT_Mount_Clearance_Sketch_' + side
    if has_sketch(bcomp, skname):
        out['base_plate_clearance_holes'] = 'already present'
    else:
        bsk = bcomp.sketches.add(bcomp.xYConstructionPlane)
        bsk.name = skname
        for (bx, by) in bolts:
            bsk.sketchCurves.sketchCircles.addByCenterRadius(
                sketch_pt(bsk, bx, by, 0.0), MM(TT_BOLT_CLR_D / 2.0))
        bcol = adsk.core.ObjectCollection.create()
        for pr in bsk.profiles:
            bcol.add(pr)
        bcut = bcomp.features.extrudeFeatures.createInput(
            bcol, adsk.fusion.FeatureOperations.CutFeatureOperation)
        bcut.setSymmetricExtent(adsk.core.ValueInput.createByReal(MM(30.0)), True)
        bcut.participantBodies = [bcomp.bRepBodies.itemByName('Base_Plate_Al')
                                  or bcomp.bRepBodies.item(0)]
        bcomp.features.extrudeFeatures.add(bcut)
        out['base_plate_clearance_holes'] = 'cut'

    out['status'] = 'BUILT'


def step_tt_motor_mounts(app, design, root, rep):
    """The TT drive motors had no mount of any kind.

    Their gearboxes carry two Ø3.4 holes running lengthwise along X at
    y = +/-8.75, z = 21 - the manufacturer's own bracket interface. Those are
    NOT used here: both ends of that screw line are blocked, by the wheel
    outboard and by the motor can inboard. Anything bolted into them would only
    fit because this model draws the gearbox and can as two separate boxes with
    a gap between them, which the real motor does not have.

    So the gearbox body is clamped instead: a U-cradle rising from the base
    plate that it seats into, plus a bar clamping it down from above.
    """
    out = {}
    for side in ('L', 'R'):
        sub = {}
        try:
            _tt_side(app, design, root, rep, side, sub)
        except Exception:
            sub['status'] = 'ERROR'
            sub['traceback'] = traceback.format_exc()
        out[side] = sub
    rep.step('6_tt_motor_mounts', out)
    rep.say('TT motor mounts: L=%s R=%s'
            % (out['L'].get('status'), out['R'].get('status')))


# ----------------------- STEP 7: the fastener holes that were never cut

def find_owner(root, body_name):
    """(occurrence, component, NATIVE body) for a body, by name."""
    for occ in root.allOccurrences:
        b = occ.component.bRepBodies.itemByName(body_name)
        if b is not None:
            return occ, occ.component, b
    return None, None, None


def count_holes(root, body_name, dia_mm, tol=0.3):
    """Holes of a given diameter in the assembly-context body, any axis."""
    b = find_body(root, body_name)
    if b is None:
        return -1
    n = 0
    for f in b.faces:
        g = f.geometry
        if g.objectType != adsk.core.Cylinder.classType():
            continue
        if abs(CM(adsk.core.Cylinder.cast(g).radius) * 2 - dia_mm) <= tol:
            n += 1
    return n


def cut_holes_z(root, body_name, positions, dia_mm, z_start, z_len, sketch_name):
    """Vertical holes through a named body. z_len may be negative (cut downward)."""
    occ, comp, native = find_owner(root, body_name)
    if native is None:
        return 'body not found'
    if has_sketch(comp, sketch_name):
        return 'already present'
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = sketch_name
    for (x, y) in positions:
        sk.sketchCurves.sketchCircles.addByCenterRadius(
            sketch_pt(sk, x, y, 0.0), MM(dia_mm / 2.0))
    col = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        col.add(p)
    cut = comp.features.extrudeFeatures.createInput(
        col, adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(z_start)))
    cut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(z_len)))
    cut.participantBodies = [native]
    comp.features.extrudeFeatures.add(cut)
    return 'cut'


def cut_hole_x(root, body_name, y_mm, z_mm, dia_mm, sketch_name):
    """One hole running along X, right through the named body."""
    occ, comp, native = find_owner(root, body_name)
    if native is None:
        return 'body not found'
    if has_sketch(comp, sketch_name):
        return 'already present'
    sk = comp.sketches.add(comp.yZConstructionPlane)
    sk.name = sketch_name
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        sketch_pt(sk, 0.0, y_mm, z_mm), MM(dia_mm / 2.0))
    cut = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.setSymmetricExtent(adsk.core.ValueInput.createByReal(MM(400.0)), True)
    cut.participantBodies = [native]
    comp.features.extrudeFeatures.add(cut)
    return 'cut'


M3_CLR = 3.4
M3_TAP = 2.5
M2_5_PILOT = 2.1
SERVO_FLANGE = 49.5


def step_missing_fasteners(app, design, root, rep):
    """Cut the holes that earlier passes logged as done but never removed.

    Each one is verified by counting the result in the solid afterwards, because
    a feature reporting success is not evidence that it cut anything - that is
    exactly how these went missing.
    """
    out = {}

    # ---- where are the corner standoffs? ------------------------------
    corners = []
    for i in (1, 2, 3, 4):
        b = find_body(root, 'Standoff_%d' % i)
        if b is None:
            continue
        bb = bbox_mm(b)
        corners.append((round((bb['x'][0] + bb['x'][1]) / 2.0, 2),
                        round((bb['y'][0] + bb['y'][1]) / 2.0, 2),
                        bb['z'][0], bb['z'][1], 'Standoff_%d' % i))
    out['corner_standoffs'] = [[c[0], c[1], c[2], c[3]] for c in corners]
    corner_xy = [(c[0], c[1]) for c in corners]

    cover = find_body(root, 'Top_Cover')
    base = find_body(root, 'Base_Plate_Al')
    if cover is None or base is None or not corners:
        out['status'] = 'skipped - Top_Cover, Base_Plate_Al or standoffs missing'
        rep.step('7_missing_fasteners', out)
        return
    cz = bbox_mm(cover)['z']
    bz = bbox_mm(base)['z']

    # ---- a. top cover: 4 corner clearance + 2 servo flange -------------
    servo_xy = [(0.0, SERVO_FLANGE / 2.0), (0.0, -SERVO_FLANGE / 2.0)]
    before = count_holes(root, 'Top_Cover', M3_CLR)
    r1 = cut_holes_z(root, 'Top_Cover', corner_xy + servo_xy, M3_CLR,
                     cz[0] - 2.0, (cz[1] - cz[0]) + 4.0, 'Cover_Fastener_Holes')
    out['top_cover'] = {'action': r1, 'holes_before': before,
                        'holes_after': count_holes(root, 'Top_Cover', M3_CLR),
                        'positions': [list(p) for p in corner_xy + servo_xy]}

    # ---- b. base plate: 4 corner clearance -----------------------------
    before = count_holes(root, 'Base_Plate_Al', M3_CLR)
    r2 = cut_holes_z(root, 'Base_Plate_Al', corner_xy, M3_CLR,
                     bz[0] - 2.0, (bz[1] - bz[0]) + 4.0, 'Corner_Standoff_Clearance')
    out['base_plate'] = {'action': r2, 'holes_before': before,
                         'holes_after': count_holes(root, 'Base_Plate_Al', M3_CLR)}

    # ---- c. corner standoffs: tapped both ends + side hole -------------
    so = {}
    for (cx, cy, z0, z1, nm) in corners:
        a = cut_holes_z(root, nm, [(cx, cy)], M3_TAP, z0, 10.0, 'Tap_Bottom')
        b2 = cut_holes_z(root, nm, [(cx, cy)], M3_TAP, z1, -10.0, 'Tap_Top')
        c2 = cut_hole_x(root, nm, cy, 44.0, M3_TAP, 'Tap_Side')
        so[nm] = {'bottom': a, 'top': b2, 'side': c2,
                  'holes_now': count_holes(root, nm, M3_TAP)}
    out['corner_standoffs_tapped'] = so

    # ---- d. PCB standoffs: pilot bore ----------------------------------
    pcb = {}
    for i in (1, 2, 3, 4):
        nm = 'PCB_Standoff_%d' % i
        b = find_body(root, nm)
        if b is None:
            continue
        bb = bbox_mm(b)
        cx = round((bb['x'][0] + bb['x'][1]) / 2.0, 2)
        cy = round((bb['y'][0] + bb['y'][1]) / 2.0, 2)
        r = cut_holes_z(root, nm, [(cx, cy)], M2_5_PILOT,
                        bb['z'][0] - 1.0, (bb['z'][1] - bb['z'][0]) + 2.0, 'Pilot_Bore')
        pcb[nm] = {'action': r, 'centre': [cx, cy],
                   'holes_now': count_holes(root, nm, M2_5_PILOT)}
    out['pcb_standoffs'] = pcb

    # ---- e. hammer servo flange tabs -----------------------------------
    servo = find_body(root, 'Hammer_Servo')
    if servo is not None:
        sb = bbox_mm(servo)
        before = count_holes(root, 'Hammer_Servo', M3_CLR)
        r = cut_holes_z(root, 'Hammer_Servo', servo_xy, M3_CLR,
                        sb['z'][0] - 1.0, 8.0, 'Servo_Tab_Holes')
        out['hammer_servo'] = {'action': r, 'holes_before': before,
                               'holes_after': count_holes(root, 'Hammer_Servo', M3_CLR),
                               'tab_z_mm': sb['z'], 'positions': [list(p) for p in servo_xy]}

    out['status'] = 'DONE'
    rep.step('7_missing_fasteners', out)
    rep.say('Missing fasteners: cover %s->%s holes, servo %s->%s'
            % (out['top_cover']['holes_before'], out['top_cover']['holes_after'],
               out.get('hammer_servo', {}).get('holes_before'),
               out.get('hammer_servo', {}).get('holes_after')))


# ------------- STEP 8: finish the standoff taps, and mount the floaters

def make_post(parent_comp, name, x, y, z0, z1, od, mat):
    """A round post standing between z0 and z1. Returns (occurrence, body)."""
    for o in list(parent_comp.occurrences):
        if o.component.name == name:
            o.deleteMe()
    occ = parent_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = name + '_Sketch'
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        sketch_pt(sk, x, y, 0.0), MM(od / 2.0))
    e = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    e.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(z0)))
    e.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(z1 - z0)))
    f = comp.features.extrudeFeatures.add(e)
    body = f.bodies.item(0)
    body.name = name
    if mat:
        body.material = mat
    return occ, comp, body


def step_finish_mounts(app, design, root, rep):
    """Three leftovers:

    1. Corner standoffs 2-4 were skipped last pass - all four bodies live in one
       component, so the shared sketch name made the later ones look done.
    2. Spinner_ESC was floating with nothing under it.
    3. Main_Disconnect had holes but the bracket that supposedly raised it was
       never modelled. Its own footprint has the TT motor cans directly below,
       so it is hung from the top cover instead.
    """
    out = {}
    mat_nylon = get_lib_material(app, '', exact='Nylon 6')
    mat_al = get_lib_material(app, '', exact='Aluminum 6061')

    # ---- 1. standoffs 2-4 ----------------------------------------------
    so = {}
    for i in (1, 2, 3, 4):
        nm = 'Standoff_%d' % i
        b = find_body(root, nm)
        if b is None:
            continue
        if count_holes(root, nm, M3_TAP) >= 3:
            so[nm] = 'already tapped'
            continue
        bb = bbox_mm(b)
        cx = round((bb['x'][0] + bb['x'][1]) / 2.0, 2)
        cy = round((bb['y'][0] + bb['y'][1]) / 2.0, 2)
        cut_holes_z(root, nm, [(cx, cy)], M3_TAP, bb['z'][0], 10.0, 'Tap_Bottom_' + nm)
        cut_holes_z(root, nm, [(cx, cy)], M3_TAP, bb['z'][1], -10.0, 'Tap_Top_' + nm)
        cut_hole_x(root, nm, cy, 44.0, M3_TAP, 'Tap_Side_' + nm)
        so[nm] = 'tapped at (%.1f, %.1f)' % (cx, cy)
    out['corner_standoffs'] = so

    printed = find_occ(root, '06_3D_Printed')
    if printed is None:
        out['status'] = 'skipped - 06_3D_Printed not found'
        rep.step('8_finish_mounts', out)
        return
    pc = printed.component

    # ---- 2. Spinner_ESC on three posts ----------------------------------
    esc = find_body(root, 'Spinner_ESC')
    esc_out = {}
    if esc is not None:
        eb = bbox_mm(esc)
        base = find_body(root, 'Base_Plate_Al')
        base_top = bbox_mm(base)['z'][1]
        # y = -13 clears the TT motor cans (they stop at y = +/-10);
        # x = 0 threads the gap between the two cans
        pos = [(-18.0, -13.0), (18.0, -13.0), (0.0, 5.0)]
        clash = []
        for (px, py) in pos:
            col = {'x': [px - 3.5, px + 3.5], 'y': [py - 3.5, py + 3.5],
                   'z': [base_top, eb['z'][0]]}
            for cn, bn, bb2 in all_bodies(root):
                if bn in ('Base_Plate_Al', 'Spinner_ESC') or bn.startswith('ESC_Post'):
                    continue
                if boxes_overlap(col, bbox_mm(bb2), 0.05):
                    clash.append({'post': [px, py], 'hits': bn})
        esc_out['posts_planned'] = pos
        esc_out['clashes'] = clash
        if clash:
            esc_out['status'] = 'ABANDONED - post column is blocked'
        else:
            made = []
            for i, (px, py) in enumerate(pos, 1):
                nm = 'ESC_Post_%d' % i
                o2, c2, b2 = make_post(pc, nm, px, py, base_top, eb['z'][0], 6.0, mat_nylon)
                cut_holes_z(root, nm, [(px, py)], M2_5_PILOT,
                            eb['z'][0], -8.0, 'Pilot_' + nm)
                made.append(nm)
            cut_holes_z(root, 'Spinner_ESC', pos, 2.2,
                        eb['z'][0] - 1.0, (eb['z'][1] - eb['z'][0]) + 2.0, 'ESC_Mount_Holes')
            esc_out['posts'] = made
            esc_out['post_height_mm'] = round(eb['z'][0] - base_top, 2)
            esc_out['status'] = 'BUILT'
    out['spinner_esc'] = esc_out

    # ---- 3. Main_Disconnect hung from the top cover ----------------------
    md = find_body(root, 'Main_Disconnect')
    cover = find_body(root, 'Top_Cover')
    md_out = {}
    if md is not None and cover is not None:
        mb = bbox_mm(md)
        cb = bbox_mm(cover)
        pos = [(-9.5, 2.0), (9.5, 2.0)]      # the switch's own M2 hole positions
        clash = []
        for (px, py) in pos:
            col = {'x': [px - 3.5, px + 3.5], 'y': [py - 3.5, py + 3.5],
                   'z': [mb['z'][1], cb['z'][0]]}
            for cn, bn, bb2 in all_bodies(root):
                if bn in ('Top_Cover', 'Main_Disconnect') or bn.startswith('Disconnect_Post'):
                    continue
                if boxes_overlap(col, bbox_mm(bb2), 0.05):
                    clash.append({'post': [px, py], 'hits': bn})
        md_out['posts_planned'] = pos
        md_out['clashes'] = clash
        if clash:
            md_out['status'] = 'ABANDONED - post column is blocked'
        else:
            made = []
            for i, (px, py) in enumerate(pos, 1):
                nm = 'Disconnect_Post_%d' % i
                o2, c2, b2 = make_post(pc, nm, px, py, mb['z'][1], cb['z'][0], 6.0, mat_al)
                # M2 up from the switch below, M3 down from the cover above
                cut_holes_z(root, nm, [(px, py)], M2_5_PILOT, mb['z'][1], 8.0, 'PilotLo_' + nm)
                cut_holes_z(root, nm, [(px, py)], M3_TAP, cb['z'][0], -8.0, 'PilotHi_' + nm)
                made.append(nm)
            cut_holes_z(root, 'Top_Cover', pos, M3_CLR,
                        cb['z'][0] - 2.0, (cb['z'][1] - cb['z'][0]) + 4.0,
                        'Disconnect_Cover_Holes')
            md_out['posts'] = made
            md_out['post_height_mm'] = round(cb['z'][0] - mb['z'][1], 2)
            md_out['status'] = 'BUILT'
            md_out['note'] = ('the switch now hangs from the top cover, so it lifts out '
                              'with the cover. Its own footprint has the TT motor cans '
                              'directly underneath, so a post to the base plate is not '
                              'possible without moving it. An access hole through the '
                              'cover is NOT modelled - a main disconnect must be '
                              'reachable without opening the robot.')
    out['main_disconnect'] = md_out

    out['status'] = 'DONE'
    rep.step('8_finish_mounts', out)
    rep.say('Finish mounts: ESC=%s, disconnect=%s'
            % (esc_out.get('status'), md_out.get('status')))


# --------------------------------- STEP 9: Spinner_ESC, with a fallback

def step_esc_mount(app, design, root, rep):
    """Stand the Spinner_ESC on posts from the base plate.

    Its footprint is badly boxed in: the TT motor cans sit directly below it
    (y +/-10), and the ESP32 board occupies everything behind y = -13. The only
    clear column is the 7.4 mm gap between the two cans at x ~ 0, so the honest
    result is a two-point mount on the centreline. Candidate layouts are tried
    best-first and each is probed against the real solids.
    """
    out = {}
    esc = find_body(root, 'Spinner_ESC')
    base = find_body(root, 'Base_Plate_Al')
    printed = find_occ(root, '06_3D_Printed')
    if esc is None or base is None or printed is None:
        out['status'] = 'skipped - required bodies not found'
        rep.step('9_esc_mount', out)
        return

    eb = bbox_mm(esc)
    base_top = bbox_mm(base)['z'][1]
    out['esc_bbox_mm'] = eb

    candidates = [
        ('corners clear of the cans', 6.0, [(-18.0, -13.0), (18.0, -13.0), (0.0, 5.0)]),
        ('centreline pair, between the cans', 6.0, [(0.0, -9.0), (0.0, 7.0)]),
        ('centreline pair, slim posts', 5.0, [(0.0, -10.0), (0.0, 8.0)]),
    ]

    chosen = None
    tried = []
    for label, od, pos in candidates:
        clash = []
        for (px, py) in pos:
            half = od / 2.0 + 0.5
            col = {'x': [px - half, px + half], 'y': [py - half, py + half],
                   'z': [base_top, eb['z'][0]]}
            for cn, bn, b2 in all_bodies(root):
                if bn in ('Base_Plate_Al', 'Spinner_ESC') or bn.startswith('ESC_Post'):
                    continue
                if boxes_overlap(col, bbox_mm(b2), 0.05):
                    clash.append('%s hits %s' % (str([px, py]), bn))
        tried.append({'layout': label, 'od_mm': od, 'positions': pos,
                      'clashes': sorted(set(clash))})
        if not clash:
            chosen = (label, od, pos)
            break
    out['candidates_tried'] = tried

    if chosen is None:
        out['status'] = 'ABANDONED - no clear column anywhere under the ESC'
        rep.step('9_esc_mount', out)
        rep.say('Spinner_ESC: no clear post column, left unmounted')
        return

    label, od, pos = chosen
    mat = get_lib_material(app, '', exact='Nylon 6')
    made = []
    for i, (px, py) in enumerate(pos, 1):
        nm = 'ESC_Post_%d' % i
        make_post(printed.component, nm, px, py, base_top, eb['z'][0], od, mat)
        cut_holes_z(root, nm, [(px, py)], M2_5_PILOT, eb['z'][0], -8.0, 'Pilot_' + nm)
        made.append(nm)
    cut_holes_z(root, 'Spinner_ESC', pos, 2.2, eb['z'][0] - 1.0,
                (eb['z'][1] - eb['z'][0]) + 2.0, 'ESC_Mount_Holes')

    out['layout_used'] = label
    out['post_od_mm'] = od
    out['posts'] = made
    out['post_height_mm'] = round(eb['z'][0] - base_top, 2)
    out['status'] = 'BUILT'
    if len(pos) < 3:
        out['caveat'] = (
            'Two posts on one centreline. The board is cantilevered about +/-22 mm '
            'either side of that line, so this restrains it but does not stop it '
            'rocking. It is the best available without moving something: the TT '
            'motor cans are directly underneath and the ESP32 board is behind. '
            'A proper fix is to relocate the ESC, or hang it from the top cover '
            'the way Main_Disconnect now is.')
    rep.step('9_esc_mount', out)
    rep.say('Spinner_ESC: %d post(s), %s' % (len(pos), label))


# ------------------- STEP 10: lighten the cradle and the TT mounts

def cut_box_y(root, body_name, x0, x1, z0, z1, sketch_name, y_len=400.0):
    """A rectangular window cut straight through a body along Y."""
    occ, comp, native = find_owner(root, body_name)
    if native is None:
        return 'body not found'
    if has_sketch(comp, sketch_name):
        return 'already present'
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = sketch_name
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        sketch_pt(sk, x0, 0.0, z0), sketch_pt(sk, x1, 0.0, z1))
    cut = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.setSymmetricExtent(adsk.core.ValueInput.createByReal(MM(y_len)), True)
    cut.participantBodies = [native]
    comp.features.extrudeFeatures.add(cut)
    return 'cut'


def step_lighten(app, design, root, rep):
    """Take metal out of the two brackets, without changing material.

    Geometry only: a swap to printed nylon would save far more but is a
    structural call on parts that carry the weapon motor and the drivetrain,
    so it is quoted, not applied.
    """
    out = {}

    # ---- cradle: one window straight through under the saddle -----------
    cr = find_body(root, CRADLE_NAME)
    if cr is not None:
        before = round(cr.physicalProperties.mass * 1000.0, 2)
        cb = bbox_mm(cr)
        # roof left at 5.35 mm under the saddle bottom (z 19 -> 24.35)
        r = cut_box_y(root, CRADLE_NAME, -9.0, 9.0, 8.0, 19.0, 'Cradle_Lightening')
        cr2 = find_body(root, CRADLE_NAME)
        after = round(cr2.physicalProperties.mass * 1000.0, 2)
        out['cradle'] = {'action': r, 'bbox_mm': cb,
                         'mass_before_g': before, 'mass_after_g': after,
                         'saved_g': round(before - after, 2),
                         'window': 'x -9..9, z 8..19, through Y',
                         'roof_left_mm': 5.35}

    # ---- TT mounts: two windows per side, clear of every tapped hole ----
    tt = {}
    for side in ('L', 'R'):
        nm = 'TT_Motor_Mount_' + side
        b = find_body(root, nm)
        if b is None:
            continue
        before = round(b.physicalProperties.mass * 1000.0, 2)
        bb = bbox_mm(b)
        xlo, xhi = bb['x'][0], bb['x'][1]
        w1 = cut_box_y(root, nm, xlo + 1.5, xlo + 7.5, 15.5, 26.0, 'TT_Light_A_' + side)
        w2 = cut_box_y(root, nm, xhi - 7.5, xhi - 1.5, 15.5, 26.0, 'TT_Light_B_' + side)
        b2 = find_body(root, nm)
        after = round(b2.physicalProperties.mass * 1000.0, 2)
        tt[nm] = {'window_a': w1, 'window_b': w2,
                  'mass_before_g': before, 'mass_after_g': after,
                  'saved_g': round(before - after, 2)}
    out['tt_mounts'] = tt

    saved = out.get('cradle', {}).get('saved_g', 0) + sum(
        v['saved_g'] for v in tt.values())
    out['total_saved_g'] = round(saved, 2)

    # what a nylon swap would additionally buy, quoted not applied
    al, ny = 2.70, 1.14
    quote = 0.0
    for v in tt.values():
        quote += v['mass_after_g'] * (1 - ny / al)
    out['nylon_swap_would_save_g'] = round(quote, 2)
    out['nylon_swap_note'] = (
        'NOT applied. Swapping only the TT mounts to printed nylon would save the '
        'figure above, but they carry the drivetrain through chassis impacts - '
        'that is a structural decision for a human.')
    out['status'] = 'DONE'
    rep.step('10_lighten', out)
    rep.say('Lightening: %.1f g removed by geometry' % saved)


# ------------------------- STEP 11: fasten the battery holder

def step_battery_holder(app, design, root, rep):
    """The battery tray floats 27.6 mm above the floor with nothing holding it.

    The PCB keep-out (x +/-30) sits directly beneath the middle of it, so the
    posts go outboard of that, at x = +/-34, threading between the keep-out and
    the buck/BEC and driver boards.
    """
    out = {}
    hold = find_body(root, 'Battery_Holder')
    base = find_body(root, 'Base_Plate_Al')
    printed = find_occ(root, '06_3D_Printed')
    if hold is None or base is None or printed is None:
        out['status'] = 'skipped - required bodies not found'
        rep.step('11_battery_holder', out)
        return

    hb = bbox_mm(hold)
    base_top = bbox_mm(base)['z'][1]
    od = 5.0
    pos = [(-34.0, -30.0), (34.0, -30.0), (-34.0, -46.0), (34.0, -46.0)]
    out['holder_bbox_mm'] = hb
    out['post_height_mm'] = round(hb['z'][0] - base_top, 2)

    clash = []
    for (px, py) in pos:
        half = od / 2.0 + 0.5
        col = {'x': [px - half, px + half], 'y': [py - half, py + half],
               'z': [base_top, hb['z'][0]]}
        for cn, bn, b2 in all_bodies(root):
            if bn in ('Base_Plate_Al', 'Battery_Holder') or bn.startswith('Batt_Post'):
                continue
            if boxes_overlap(col, bbox_mm(b2), 0.05):
                clash.append('%s hits %s' % (str([px, py]), bn))
    out['positions'] = pos
    out['clashes'] = sorted(set(clash))
    if clash:
        out['status'] = 'ABANDONED - post column blocked'
        rep.step('11_battery_holder', out)
        rep.say('Battery holder: post column blocked, not fastened')
        return

    mat = get_lib_material(app, '', exact='Nylon 6')
    made = []
    for i, (px, py) in enumerate(pos, 1):
        nm = 'Batt_Post_%d' % i
        make_post(printed.component, nm, px, py, base_top, hb['z'][0], od, mat)
        cut_holes_z(root, nm, [(px, py)], M2_5_PILOT, hb['z'][0], -8.0, 'PilotTop_' + nm)
        cut_holes_z(root, nm, [(px, py)], M2_5_PILOT, base_top, 8.0, 'PilotBot_' + nm)
        made.append(nm)

    cut_holes_z(root, 'Battery_Holder', pos, 2.9,
                hb['z'][0] - 1.0, (hb['z'][1] - hb['z'][0]) + 2.0, 'Holder_Mount_Holes')
    cut_holes_z(root, 'Base_Plate_Al', pos, 2.9,
                bbox_mm(base)['z'][0] - 2.0, 6.0, 'Batt_Post_BaseHoles')

    out['posts'] = made
    out['status'] = 'BUILT'
    out['assembly_note'] = ('M2.5 down through the tray floor into each post, and M2.5 up '
                            'through the base plate into the post bottoms. Countersink the '
                            'tray screws - the battery sits directly on that floor.')
    rep.step('11_battery_holder', out)
    rep.say('Battery holder: %d posts' % len(made))


# --------------------- STEP 12: orthographic views + assembly sequence

VIEW_DIR = os.path.join(EXPORT_DIR, 'views')

ASSEMBLY_STAGES = [
    ('01_base_plate', ['Base_Plate_Al']),
    ('02_frame', ['Left_Side_Plate_Al', 'Right_Side_Plate_Al', 'Front_Wedge_L',
                  'Front_Wedge_R', 'Standoffs_Fasteners']),
    ('03_drivetrain', ['TT_Motor_Mount_L', 'TT_Motor_Mount_R', 'TT_Motor_Clamp_L',
                       'TT_Motor_Clamp_R', 'TT_Gearbox_L', 'TT_Gearbox_R',
                       'TT_Motor_Can_L', 'TT_Motor_Can_R', 'Left_Wheel', 'Right_Wheel']),
    ('04_weapon', ['Spinner_Motor_Cradle', 'Spinner_Motor', 'Spinner_Shaft',
                   'Left_Support', 'Right_Support', 'Spinner_Bar',
                   'Spinner_ESC', 'ESC_Post_1', 'ESC_Post_2']),
    ('05_electronics', ['PCB_Standoff_1', 'PCB_Standoff_2', 'PCB_Standoff_3',
                        'PCB_Standoff_4', 'ESP32_Control_Board', 'RC_Receiver',
                        'Wheel_Motor_Drivers', 'Power_Buck_BEC', 'Main_Disconnect',
                        'Batt_Post_1', 'Batt_Post_2', 'Batt_Post_3', 'Batt_Post_4',
                        'Battery_Holder', 'Battery_and_Tray']),
    ('06_cover_and_hammer', ['Disconnect_Post_1', 'Disconnect_Post_2', 'Top_Cover',
                             'Hammer_Servo', 'Hammer_Arm', 'Hammer_Head']),
]

HIDE_ALWAYS = ('Spinner_Sweep_Envelope', 'PCB_Keepout_Envelope',
               'Cable_Routing_Keepouts')


def _set_camera(app, eye, target, up, extents=None, fit=False):
    vp = app.activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    cam.target = adsk.core.Point3D.create(*target)
    cam.eye = adsk.core.Point3D.create(*eye)
    cam.upVector = adsk.core.Vector3D.create(*up)
    cam.isSmoothTransition = False
    if extents:
        cam.viewExtents = extents
    vp.camera = cam
    if fit:
        vp.fit()
    vp.refresh()
    adsk.doEvents()


def _shot(app, path, w=1600, h=1100):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    app.activeViewport.refresh()
    adsk.doEvents()
    return app.activeViewport.saveAsImageFile(path, w, h)


def _show(root, names, on):
    """Toggle only body-carrying occurrences.

    Parent occurrences must stay lit: switching off a parent hides everything
    beneath it no matter what the children say, which is how the first attempt
    produced six identical pictures of an empty viewport."""
    for occ in root.allOccurrences:
        if occ.component.name in names and occ.component.bRepBodies.count > 0:
            occ.isLightBulbOn = on


def _hide_all_bodies(root):
    for occ in root.allOccurrences:
        if occ.component.bRepBodies.count > 0:
            occ.isLightBulbOn = False
        else:
            occ.isLightBulbOn = True          # keep the parents lit


def step_views(app, design, root, rep):
    """Orthographic views and a visibility-stepped assembly sequence.

    These are not engineering drawings. Dimensioned 2D drawings are made in
    Fusion's Drawing workspace by hand; the API cannot place dimensions. The
    real manufacturing data is the DXF and STEP set.
    """
    out = {'files': []}
    os.makedirs(VIEW_DIR, exist_ok=True)

    for occ in root.allOccurrences:
        occ.isLightBulbOn = occ.component.name not in HIDE_ALWAYS

    views = [
        ('iso', (-30.0, -30.0, 24.0), (0.0, 0.0, 4.0), (0.0, 0.0, 1.0)),
        ('top', (0.0, 0.0, 60.0), (0.0, 0.0, 4.0), (0.0, 1.0, 0.0)),
        # +Y is the FRONT of this robot (the spinner sits at y = +75),
        # so the camera on -Y is the one that sees the back.
        ('rear', (0.0, -60.0, 4.0), (0.0, 0.0, 4.0), (0.0, 0.0, 1.0)),
        ('right', (60.0, 0.0, 4.0), (0.0, 0.0, 4.0), (0.0, 0.0, 1.0)),
        ('front', (0.0, 60.0, 4.0), (0.0, 0.0, 4.0), (0.0, 0.0, 1.0)),
    ]
    for nm, eye, tgt, up in views:
        _set_camera(app, eye, tgt, up, fit=True)
        p = os.path.join(VIEW_DIR, 'view_%s.png' % nm)
        out['files'].append({'file': 'view_%s.png' % nm, 'ok': _shot(app, p)})

    # ---- assembly sequence ------------------------------------------------
    _hide_all_bodies(root)
    shown = []
    for stage, names in ASSEMBLY_STAGES:
        shown.extend(names)
        _show(root, names, True)
        _show(root, HIDE_ALWAYS, False)
        _set_camera(app, (-30.0, -30.0, 24.0), (0.0, 0.0, 4.0), (0.0, 0.0, 1.0))
        app.activeViewport.fit()
        p = os.path.join(VIEW_DIR, 'assembly_%s.png' % stage)
        ok = _shot(app, p)
        visible = sorted(o.component.name for o in root.allOccurrences
                         if o.isLightBulbOn and o.component.bRepBodies.count > 0)
        out['files'].append({'file': 'assembly_%s.png' % stage, 'ok': ok,
                             'adds': names, 'visible_count': len(visible)})

    for occ in root.allOccurrences:
        occ.isLightBulbOn = occ.component.name not in HIDE_ALWAYS
    _set_camera(app, (-30.0, -30.0, 24.0), (0.0, 0.0, 4.0), (0.0, 0.0, 1.0), fit=True)

    out['dir'] = VIEW_DIR
    out['status'] = 'DONE'
    out['caveat'] = ('Images only. Dimensioned drawings need Fusion\'s Drawing '
                     'workspace; the API cannot place dimensions. Use the DXF set '
                     'for fabrication.')
    rep.step('12_views', out)
    rep.say('Views: %d images written' % len(out['files']))


# ------------------------- STEP 13: dimensions + fastener schedule

def step_dimension_doc(app, design, root, rep):
    """A manufacturing reference: envelope, parameters, part sizes, and every
    fastener hole position measured out of the solids."""
    out = {}
    lines = ['# Dimensions & Fastener Schedule',
             '',
             'Measured out of the live Fusion model on %s.'
             % datetime.datetime.now().isoformat(timespec='seconds'),
             '',
             '> Not a drawing. Dimensioned 2D drawings are a Fusion Drawing-workspace',
             '> job that the API cannot do. For fabrication use the DXF files in',
             '> `exports/DXF/` and the STEP master; this table is the reference that',
             '> goes with them.',
             '']

    # overall envelope from real bodies
    ENV = ('Spinner_Sweep_Envelope', 'PCB_Keepout_Envelope')
    xs, ys, zs = [], [], []
    rows = []
    for cn, bn, b in all_bodies(root):
        if bn in ENV:
            continue
        bb = bbox_mm(b)
        xs += bb['x']; ys += bb['y']; zs += bb['z']
        rows.append((bn, bb, round(b.physicalProperties.mass * 1000.0, 2),
                     b.material.name if b.material else '-'))
    lines += ['## Overall envelope', '',
              '| Axis | Min | Max | Size |', '|---|---:|---:|---:|',
              '| X (width) | %.1f | %.1f | **%.1f mm** |' % (min(xs), max(xs), max(xs) - min(xs)),
              '| Y (length) | %.1f | %.1f | **%.1f mm** |' % (min(ys), max(ys), max(ys) - min(ys)),
              '| Z (height) | %.1f | %.1f | **%.1f mm** |' % (min(zs), max(zs), max(zs) - min(zs)),
              '']

    props = root.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
    com = props.centerOfMass
    lines += ['Centre of mass: x %.1f, y %.1f, z %.1f mm (z measured from ground)'
              % (CM(com.x), CM(com.y), CM(com.z)), '']

    lines += ['## User parameters', '', '| Parameter | Value | Comment |', '|---|---:|---|']
    for i in range(design.userParameters.count):
        p = design.userParameters.item(i)
        lines.append('| `%s` | %s | %s |' % (p.name, p.expression, p.comment or ''))
    lines.append('')

    lines += ['## Parts', '',
              '| Part | X | Y | Z | Size (mm) | Material | Mass |',
              '|---|---|---|---|---|---|---:|']
    for bn, bb, mass, mat in sorted(rows):
        lines.append('| %s | %.1f…%.1f | %.1f…%.1f | %.1f…%.1f | %.1f × %.1f × %.1f | %s | %.2f g |'
                     % (bn, bb['x'][0], bb['x'][1], bb['y'][0], bb['y'][1],
                        bb['z'][0], bb['z'][1],
                        bb['x'][1] - bb['x'][0], bb['y'][1] - bb['y'][0],
                        bb['z'][1] - bb['z'][0], mat, mass))
    lines.append('')

    lines += ['## Fastener schedule', '',
              'Every cylindrical hole found in each solid, by diameter and position.',
              '', '| Part | Ø | Axis | Position (x, y, z) |', '|---|---:|---|---|']
    total = 0
    for cn, bn, b in sorted(all_bodies(root), key=lambda t: t[1]):
        if bn in ENV:
            continue
        seen = {}
        for f in b.faces:
            g = f.geometry
            if g.objectType != adsk.core.Cylinder.classType():
                continue
            c = adsk.core.Cylinder.cast(g)
            d = CM(c.radius) * 2
            if d > 8.0:
                continue
            ax = c.axis
            a = ('X' if abs(ax.x) > 0.9 else 'Y' if abs(ax.y) > 0.9
                 else 'Z' if abs(ax.z) > 0.9 else '?')
            seen[(round(d, 2), a, round(CM(c.origin.x), 1),
                  round(CM(c.origin.y), 1), round(CM(c.origin.z), 1))] = True
        for k in sorted(seen.keys()):
            lines.append('| %s | %.2f | %s | (%.1f, %.1f, %.1f) |'
                         % (bn, k[0], k[1], k[2], k[3], k[4]))
            total += 1
    lines += ['', 'Total holes listed: **%d**' % total, '']

    path = os.path.join(EXPORT_DIR, 'Dimensions_and_Fasteners.md')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    out['path'] = path
    out['parts'] = len(rows)
    out['holes'] = total
    out['status'] = 'DONE'
    rep.step('13_dimensions', out)
    rep.say('Dimensions doc: %d parts, %d holes' % (len(rows), total))


# ------------- STEP 14: relocate the disconnect, access hole, rear panel

def delete_sketch_and_feature(comp, sketch_name):
    """Remove a cut and the sketch that drove it, so no orphan holes are left."""
    removed = []
    for f in list(comp.features.extrudeFeatures):
        names = set()
        try:
            prof = f.profile
            try:
                items = [prof.item(i) for i in range(prof.count)]
            except Exception:
                items = [prof]
            for it in items:
                try:
                    names.add(it.parentSketch.name)
                except Exception:
                    pass
        except Exception:
            continue
        if sketch_name in names:
            try:
                f.deleteMe()
                removed.append('feature deleted')
            except Exception as e:
                removed.append('feature FAILED: %s' % e)
    for s in list(comp.sketches):
        if s.name == sketch_name:
            try:
                s.deleteMe()
                removed.append('sketch deleted')
            except Exception as e:
                removed.append('sketch FAILED: %s' % e)
    return removed or ['nothing found']


def move_body(comp, body, dx, dy, dz):
    col = adsk.core.ObjectCollection.create()
    col.add(body)
    t = adsk.core.Matrix3D.create()
    t.translation = adsk.core.Vector3D.create(MM(dx), MM(dy), MM(dz))
    try:
        mi = comp.features.moveFeatures.createInput2(col)
        mi.defineAsFreeMove(t)
        comp.features.moveFeatures.add(mi)
        return 'moved (createInput2)'
    except Exception:
        mi = comp.features.moveFeatures.createInput(col, t)
        comp.features.moveFeatures.add(mi)
        return 'moved (createInput)'


def cut_rect_z(root, body_name, x0, x1, y0, y1, z_start, z_len, sketch_name):
    occ, comp, native = find_owner(root, body_name)
    if native is None:
        return 'body not found'
    if has_sketch(comp, sketch_name):
        return 'already present'
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = sketch_name
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        sketch_pt(sk, x0, y0, 0.0), sketch_pt(sk, x1, y1, 0.0))
    cut = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(z_start)))
    cut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(z_len)))
    cut.participantBodies = [native]
    comp.features.extrudeFeatures.add(cut)
    return 'cut'


def cut_hole_y(root, body_name, x_mm, z_mm, dia_mm, sketch_name):
    """One hole running along Y, right through the named body."""
    occ, comp, native = find_owner(root, body_name)
    if native is None:
        return 'body not found'
    if has_sketch(comp, sketch_name):
        return 'already present'
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = sketch_name
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        sketch_pt(sk, x_mm, 0.0, z_mm), MM(dia_mm / 2.0))
    cut = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.setSymmetricExtent(adsk.core.ValueInput.createByReal(MM(400.0)), True)
    cut.participantBodies = [native]
    comp.features.extrudeFeatures.add(cut)
    return 'cut'


DISC_NEW_CENTRE = (42.0, -70.0)


def step_relocate_disconnect(app, design, root, rep):
    """Move the main disconnect somewhere it can actually be reached.

    At its old spot the hammer servo sat on the cover directly over it, so no
    usable access hole was possible. (42, -70) is clear of the servo footprint
    (y >= -28.75) and has an unobstructed column down to the base plate.
    """
    out = {}
    md = find_body(root, 'Main_Disconnect')
    cover = find_body(root, 'Top_Cover')
    base = find_body(root, 'Base_Plate_Al')
    printed = find_occ(root, '06_3D_Printed')
    if None in (md, cover, base, printed):
        out['status'] = 'skipped - required bodies not found'
        rep.step('14_relocate_disconnect', out)
        return

    mb = bbox_mm(md)
    cx = (mb['x'][0] + mb['x'][1]) / 2.0
    cy = (mb['y'][0] + mb['y'][1]) / 2.0
    out['old_centre_mm'] = [round(cx, 2), round(cy, 2)]

    if abs(cx - DISC_NEW_CENTRE[0]) < 0.5 and abs(cy - DISC_NEW_CENTRE[1]) < 0.5:
        out['status'] = 'already relocated'
        rep.step('14_relocate_disconnect', out)
        return

    # ---- clear away the old hanging arrangement -------------------------
    dt = find_occ(root, '06_3D_Printed').component
    killed = []
    for o in list(dt.occurrences):
        if o.component.name.startswith('Disconnect_Post'):
            o.deleteMe()
            killed.append(o.component.name if o.isValid else 'Disconnect_Post')
    out['old_posts_deleted'] = len(killed)
    occ_c, comp_c, native_c = find_owner(root, 'Top_Cover')
    out['old_cover_holes'] = delete_sketch_and_feature(comp_c, 'Disconnect_Cover_Holes')

    # ---- move it ---------------------------------------------------------
    occ_m, comp_m, native_m = find_owner(root, 'Main_Disconnect')
    dx = DISC_NEW_CENTRE[0] - cx
    dy = DISC_NEW_CENTRE[1] - cy
    out['translation_mm'] = [round(dx, 2), round(dy, 2)]
    out['move'] = move_body(comp_m, native_m, dx, dy, 0.0)

    md2 = find_body(root, 'Main_Disconnect')
    nb = bbox_mm(md2)
    out['new_bbox_mm'] = nb
    base_top = bbox_mm(base)['z'][1]
    cb = bbox_mm(cover)

    # ---- posts down to the base plate -----------------------------------
    holes = find_vertical_holes(md2, 2.2)
    out['switch_hole_centres_mm'] = [list(h) for h in holes]
    if len(holes) < 2:
        out['status'] = 'moved, but could not find its M2 holes to line posts up with'
        rep.step('14_relocate_disconnect', out)
        return

    clash = []
    for (px, py) in holes:
        col = {'x': [px - 3.5, px + 3.5], 'y': [py - 3.5, py + 3.5],
               'z': [base_top, nb['z'][0]]}
        for cn, bn, b2 in all_bodies(root):
            if bn in ('Base_Plate_Al', 'Main_Disconnect') or bn.startswith('Disconnect_Post'):
                continue
            if boxes_overlap(col, bbox_mm(b2), 0.05):
                clash.append('%s hits %s' % (str([px, py]), bn))
    out['post_clashes'] = sorted(set(clash))
    if clash:
        out['status'] = 'moved, but post column blocked - no posts built'
        rep.step('14_relocate_disconnect', out)
        return

    mat = get_lib_material(app, '', exact='Aluminum 6061')
    made = []
    for i, (px, py) in enumerate(holes, 1):
        nm = 'Disconnect_Post_%d' % i
        make_post(printed.component, nm, px, py, base_top, nb['z'][0], 6.0, mat)
        cut_holes_z(root, nm, [(px, py)], M2_5_PILOT, nb['z'][0], -8.0, 'PilotTop_' + nm)
        cut_holes_z(root, nm, [(px, py)], M2_5_PILOT, base_top, 8.0, 'PilotBot_' + nm)
        made.append(nm)
    cut_holes_z(root, 'Base_Plate_Al', holes, 2.9,
                bbox_mm(base)['z'][0] - 2.0, 6.0, 'Disconnect_Post_BaseHoles')
    out['posts'] = made
    out['post_height_mm'] = round(nb['z'][0] - base_top, 2)

    # ---- the access hole -------------------------------------------------
    ax0, ax1 = nb['x'][0] + 2.5, nb['x'][1] - 2.5
    ay0, ay1 = nb['y'][0] + 2.0, nb['y'][1] - 2.0
    r = cut_rect_z(root, 'Top_Cover', ax0, ax1, ay0, ay1,
                   cb['z'][0] - 2.0, (cb['z'][1] - cb['z'][0]) + 4.0,
                   'Disconnect_Access_Hole')
    out['access_hole'] = {'action': r,
                          'opening_mm': [round(ax1 - ax0, 1), round(ay1 - ay0, 1)],
                          'at': [round((ax0 + ax1) / 2, 1), round((ay0 + ay1) / 2, 1)],
                          'reach_depth_mm': round(cb['z'][0] - nb['z'][1], 1)}
    out['status'] = 'BUILT'
    rep.step('14_relocate_disconnect', out)
    rep.say('Disconnect moved to (%.0f, %.0f), %d posts, access hole %s'
            % (DISC_NEW_CENTRE[0], DISC_NEW_CENTRE[1], len(made),
               out['access_hole']['opening_mm']))


REAR_PANEL_T = 2.0
REAR_PANEL_TOP = None     # None = run right up to the underside of the top cover


def step_rear_panel(app, design, root, rep):
    """The back of the chassis is open. A printed panel closes it against
    debris. Non-structural: it is screwed to the two rear corner standoffs."""
    out = {}
    base = find_body(root, 'Base_Plate_Al')
    lsp = find_body(root, 'Left_Side_Plate_Al')
    rsp = find_body(root, 'Right_Side_Plate_Al')
    covers = find_occ(root, 'Non_Structural_Covers')
    if None in (base, lsp, rsp) or covers is None:
        out['status'] = 'skipped - required bodies not found'
        rep.step('15_rear_panel', out)
        return

    bb = bbox_mm(base)
    base_top = bb['z'][1]
    x0 = bbox_mm(lsp)['x'][1]      # inner face of the left wall
    x1 = bbox_mm(rsp)['x'][0]
    y1 = bb['y'][0] + REAR_PANEL_T
    y0 = bb['y'][0]
    cover = find_body(root, 'Top_Cover')
    top = REAR_PANEL_TOP if REAR_PANEL_TOP else bbox_mm(cover)['z'][0]
    out['panel_mm'] = {'x': [x0, x1], 'y': [y0, y1], 'z': [base_top, top]}

    clash = []
    tgt = {'x': [x0, x1], 'y': [y0, y1], 'z': [base_top, top]}
    for cn, bn, b2 in all_bodies(root):
        if bn in ('Base_Plate_Al', 'Rear_Panel'):
            continue
        if boxes_overlap(tgt, bbox_mm(b2), 0.05):
            clash.append(bn)
    out['clashes'] = sorted(set(clash))
    if clash:
        out['status'] = 'ABANDONED - %s in the way' % ', '.join(sorted(set(clash)))
        rep.step('15_rear_panel', out)
        return

    for o in list(covers.component.occurrences):
        if o.component.name == 'Rear_Panel':
            o.deleteMe()
    occ = covers.component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = 'Rear_Panel'
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Rear_Panel_Sketch'
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        sketch_pt(sk, x0, y0, 0.0), sketch_pt(sk, x1, y1, 0.0))
    e = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    e.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(base_top)))
    e.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(top - base_top)))
    body = comp.features.extrudeFeatures.add(e).bodies.item(0)
    body.name = 'Rear_Panel'
    mat = get_lib_material(app, '', exact='Nylon 6')
    if mat:
        body.material = mat

    # ---- screw it to the two rear corner standoffs ----------------------
    rear = []
    for i in (1, 2, 3, 4):
        b = find_body(root, 'Standoff_%d' % i)
        if b is None:
            continue
        sb = bbox_mm(b)
        cy = (sb['y'][0] + sb['y'][1]) / 2.0
        if cy < -50:                       # the two at the back
            rear.append(('Standoff_%d' % i, round((sb['x'][0] + sb['x'][1]) / 2.0, 2)))
    fixings = []
    for (nm, sx) in rear:
        for z in (20.0, 55.0):
            cut_hole_y(root, nm, sx, z, M3_TAP, 'PanelTap_%.0f' % z)
            fixings.append([sx, z])
    # clearance holes through the panel, running along Y
    for (sx, z) in fixings:
        cut_hole_y(root, 'Rear_Panel', sx, z, M3_CLR, 'PanelClr_%.0f_%.0f' % (sx, z))

    out['fixings'] = fixings
    out['mass_g'] = round(body.physicalProperties.mass * 1000.0, 2)
    out['bbox_mm'] = bbox_mm(body)
    out['status'] = 'BUILT'
    out['note'] = ('Nylon, non-structural. Screwed to the two rear corner standoffs '
                   'with 4x M3 and captured laterally by the side walls. It runs the '
                   'full height to the underside of the top cover (z = %.0f mm) so the '
                   'back is actually closed - an 18 mm slot above a shorter panel would '
                   'have let debris straight in.' % top)
    rep.step('15_rear_panel', out)
    rep.say('Rear panel: %.1f g, %d fixings' % (out['mass_g'], len(fixings)))


# ---------------------- STEP 16: shields over the electronics boards

SHIELD_GAP = 3.0        # mm standoff above the board, for connectors and air
SHIELD_T = 2.0          # mm plate thickness
SHIELD_MODULES = ('RC_Receiver', 'Wheel_Motor_Drivers', 'Power_Buck_BEC')


def step_board_shields(app, design, root, rep):
    """A shield plate over each floor-level board.

    It rides on the board's own two M2 screws through short spacers, so it needs
    no new holes in the base plate and no new clear column - only the space
    directly above the board. The overhang is trimmed per board: the bay is
    tight, and a generous margin would run into the battery posts or the rear
    panel.
    """
    out = {'boards': {}}
    printed = find_occ(root, '06_3D_Printed')
    if printed is None:
        out['status'] = 'skipped - 06_3D_Printed not found'
        rep.step('16_board_shields', out)
        return
    mat = get_lib_material(app, '', exact='Nylon 6')

    for name in SHIELD_MODULES:
        info = {}
        b = find_body(root, name)
        if b is None:
            info['status'] = 'body not found'
            out['boards'][name] = info
            continue
        mb = bbox_mm(b)
        holes = find_vertical_holes(b, 2.2)
        info['module_bbox_mm'] = mb
        info['screw_positions_mm'] = [list(h) for h in holes]
        if len(holes) < 2:
            info['status'] = 'no M2 screws to ride on'
            out['boards'][name] = info
            continue

        z0 = mb['z'][1] + SHIELD_GAP
        z1 = z0 + SHIELD_T
        chosen = None
        for e in (3.0, 2.0, 1.0, 0.0):
            tgt = {'x': [mb['x'][0] - e, mb['x'][1] + e],
                   'y': [mb['y'][0] - e, mb['y'][1] + e],
                   'z': [mb['z'][1], z1]}
            clash = [bn for cn, bn, b2 in all_bodies(root)
                     if bn != name and not bn.startswith('Shield_')
                     and boxes_overlap(tgt, bbox_mm(b2), 0.05)]
            if not clash:
                chosen = (e, tgt)
                break
            info.setdefault('rejected', []).append(
                {'overhang_mm': e, 'clash': sorted(set(clash))})
        if chosen is None:
            info['status'] = 'ABANDONED - no room above this board'
            out['boards'][name] = info
            continue

        e, tgt = chosen
        nm = 'Shield_' + name
        for o in list(printed.component.occurrences):
            if o.component.name == nm:
                o.deleteMe()
        occ = printed.component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = nm
        comp = occ.component

        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.name = nm + '_Sketch'
        sk.sketchCurves.sketchLines.addTwoPointRectangle(
            sketch_pt(sk, tgt['x'][0], tgt['y'][0], 0.0),
            sketch_pt(sk, tgt['x'][1], tgt['y'][1], 0.0))
        ex = comp.features.extrudeFeatures.createInput(
            sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ex.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(MM(z0)))
        ex.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(SHIELD_T)))
        plate = comp.features.extrudeFeatures.add(ex).bodies.item(0)
        plate.name = nm

        # spacers from the board's top face up to the plate
        sk2 = comp.sketches.add(comp.xYConstructionPlane)
        sk2.name = nm + '_Spacers'
        for (hx, hy) in holes:
            sk2.sketchCurves.sketchCircles.addByCenterRadius(
                sketch_pt(sk2, hx, hy, 0.0), MM(2.5))
        col = adsk.core.ObjectCollection.create()
        for pr in sk2.profiles:
            col.add(pr)
        ex2 = comp.features.extrudeFeatures.createInput(
            col, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        ex2.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(MM(mb['z'][1])))
        ex2.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(SHIELD_GAP)))
        ex2.participantBodies = [plate]
        comp.features.extrudeFeatures.add(ex2)

        # the screw passes through plate and spacers
        cut_holes_z(root, nm, holes, 2.2, mb['z'][1] - 1.0,
                    (z1 - mb['z'][1]) + 2.0, nm + '_ScrewHoles')

        if mat:
            plate.material = mat
        info['overhang_mm'] = e
        info['plate_z_mm'] = [z0, z1]
        info['bbox_mm'] = bbox_mm(find_body(root, nm))
        info['mass_g'] = round(find_body(root, nm).physicalProperties.mass * 1000.0, 2)
        info['status'] = 'BUILT'
        out['boards'][name] = info

    built = [k for k, v in out['boards'].items() if v.get('status') == 'BUILT']
    out['total_mass_g'] = round(sum(v.get('mass_g', 0) for v in out['boards'].values()), 2)
    out['status'] = 'DONE'
    out['note'] = ('Each shield rides on the board\'s own two M2 screws through '
                   'integral spacers - use screws %0.0f mm longer than before. '
                   'The ESP32 board is not shielded this way: its keep-out reaches '
                   'z = 31.6 mm and the battery tray starts at 32.6 mm, so there is '
                   'no room. It is already covered by the battery tray above it.'
                   % (SHIELD_GAP + SHIELD_T))
    rep.step('16_board_shields', out)
    rep.say('Board shields: %d built, %.1f g' % (len(built), out['total_mass_g']))


# ------------------------- STEP 17: hollow out the front wedges

WEDGE_WALL = 2.5


def face_bbox_mm(f):
    bb = f.boundingBox
    return {'x': [CM(bb.minPoint.x), CM(bb.maxPoint.x)],
            'y': [CM(bb.minPoint.y), CM(bb.maxPoint.y)],
            'z': [CM(bb.minPoint.z), CM(bb.maxPoint.z)]}


def planar_face_at(body, axis, at_max, tol=0.1):
    """The largest planar face lying flat against one extreme of the body."""
    bb = bbox_mm(body)
    target = bb[axis][1] if at_max else bb[axis][0]
    best, best_area = None, -1.0
    for f in body.faces:
        if f.geometry.objectType != adsk.core.Plane.classType():
            continue
        fb = face_bbox_mm(f)
        if abs(fb[axis][0] - target) < tol and abs(fb[axis][1] - target) < tol:
            if f.area > best_area:
                best_area, best = f.area, f
    return best


def step_hollow_wedges(app, design, root, rep):
    """The front wedges are solid 45 mm aluminium - 237 g each, a third of the
    whole robot. Shelling keeps every functional surface (ramp angle, ground
    gap, the spinner corridor) and just removes the interior, which is what a
    fabricated wedge would be anyway.

    Open faces: the inner one facing the robot centre, and the bottom.
    """
    out = {'wedges': {}}
    for side, inner_at_max in (('L', True), ('R', False)):
        nm = 'Front_Wedge_' + side
        info = {}
        occ, comp, native = find_owner(root, nm)
        if native is None:
            info['status'] = 'body not found'
            out['wedges'][nm] = info
            continue
        before = round(native.physicalProperties.mass * 1000.0, 2)
        holes_before = count_holes(root, nm, M3_CLR)
        info['mass_before_g'] = before
        info['holes_before'] = holes_before
        info['bbox_mm'] = bbox_mm(find_body(root, nm))

        if before < 150.0:
            info['status'] = 'already hollow - left alone'
            out['wedges'][nm] = info
            continue

        inner = planar_face_at(native, 'x', inner_at_max)
        bottom = planar_face_at(native, 'z', False)
        info['open_faces_found'] = {'inner': inner is not None,
                                    'bottom': bottom is not None}
        if inner is None or bottom is None:
            info['status'] = 'ABANDONED - could not identify the faces to open'
            out['wedges'][nm] = info
            continue

        try:
            faces = adsk.core.ObjectCollection.create()
            faces.add(inner)
            faces.add(bottom)
            si = comp.features.shellFeatures.createInput(faces, False)
            si.insideThickness = adsk.core.ValueInput.createByReal(MM(WEDGE_WALL))
            comp.features.shellFeatures.add(si)
        except Exception:
            info['status'] = 'ABANDONED - shell failed'
            info['traceback'] = traceback.format_exc().splitlines()[-1]
            out['wedges'][nm] = info
            continue

        after_body = find_body(root, nm)
        after = round(after_body.physicalProperties.mass * 1000.0, 2)
        info['mass_after_g'] = after
        info['saved_g'] = round(before - after, 2)
        info['wall_mm'] = WEDGE_WALL
        info['holes_after'] = count_holes(root, nm, M3_CLR)
        info['status'] = 'SHELLED'
        if info['holes_after'] < holes_before:
            info['warning'] = ('the shell consumed %d of the M3 hole walls - '
                               'they need re-cutting'
                               % (holes_before - info['holes_after']))
        out['wedges'][nm] = info

    saved = sum(v.get('saved_g', 0) for v in out['wedges'].values())
    out['total_saved_g'] = round(saved, 2)
    out['status'] = 'DONE'
    rep.step('17_hollow_wedges', out)
    rep.say('Wedges hollowed: %.1f g saved' % saved)


# ------------- STEP 17b: hollow the wedges with a predictable pocket

def step_hollow_wedges2(app, design, root, rep):
    """Shelling blew the wedges down to 0.5 g, so this cuts an inset triangular
    pocket instead - the same result, but the volume is arithmetic I can check
    before and after.

    Each wedge is a right triangular prism: vertical back at y_min, flat bottom
    at z_min, ramp from the front tip (y_max, z_min) up to (y_min, z_max).
    The pocket is that triangle offset inward by the wall thickness, cut in from
    the inner face and stopping one wall short of the outer face.

    It also re-cuts the hold-down holes: they exist, but only 1 mm deep, which
    fastens nothing.
    """
    out = {'wedges': {}}
    for side in ('L', 'R'):
        nm = 'Front_Wedge_' + side
        info = {}
        occ, comp, native = find_owner(root, nm)
        if native is None:
            info['status'] = 'body not found'
            out['wedges'][nm] = info
            continue

        before = round(native.physicalProperties.mass * 1000.0, 2)
        info['mass_before_g'] = before
        bb = bbox_mm(find_body(root, nm))
        y0, y1 = bb['y'][0], bb['y'][1]
        z0, z1 = bb['z'][0], bb['z'][1]
        x_in = bb['x'][1] if side == 'L' else bb['x'][0]
        x_out = bb['x'][0] if side == 'L' else bb['x'][1]
        sign = 1.0 if x_out > x_in else -1.0
        depth = abs(x_out - x_in) - WEDGE_WALL
        t = WEDGE_WALL

        # inset triangle
        a = z1 - z0
        b = y1 - y0
        c = a * y1 + b * z0 - t * math.sqrt(a * a + b * b)
        pA = (y0 + t, z0 + t)
        pB = ((c - b * (z0 + t)) / a, z0 + t)
        pC = (y0 + t, (c - a * (y0 + t)) / b)
        area = abs((pB[0] - pA[0]) * (pC[1] - pA[1])) / 2.0
        info['pocket'] = {'vertices_yz': [list(map(lambda v: round(v, 2), p))
                                          for p in (pA, pB, pC)],
                          'area_mm2': round(area, 1),
                          'depth_mm': round(depth, 2),
                          'predicted_removed_mm3': round(area * depth, 0)}

        if before < 150.0:
            info['status'] = 'already hollow - left alone'
            out['wedges'][nm] = info
            continue

        if not has_sketch(comp, 'Wedge_Pocket'):
            sk = comp.sketches.add(comp.yZConstructionPlane)
            sk.name = 'Wedge_Pocket'
            lines = sk.sketchCurves.sketchLines
            p1 = sketch_pt(sk, 0.0, pA[0], pA[1])
            p2 = sketch_pt(sk, 0.0, pB[0], pB[1])
            p3 = sketch_pt(sk, 0.0, pC[0], pC[1])
            lines.addByTwoPoints(p1, p2)
            lines.addByTwoPoints(p2, p3)
            lines.addByTwoPoints(p3, p1)

            def make_cut(dist):
                ci = comp.features.extrudeFeatures.createInput(
                    sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
                ci.startExtent = adsk.fusion.OffsetStartDefinition.create(
                    adsk.core.ValueInput.createByReal(MM(x_in)))
                ci.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(dist)))
                ci.participantBodies = [native]
                return comp.features.extrudeFeatures.add(ci)

            feat = make_cut(sign * depth)
            m = find_body(root, nm).physicalProperties.mass * 1000.0
            if not (40.0 < m < 130.0):
                feat.deleteMe()
                feat = make_cut(-sign * depth)
                m = find_body(root, nm).physicalProperties.mass * 1000.0
                info['direction_flipped'] = True
            if not (40.0 < m < 130.0):
                feat.deleteMe()
                info['status'] = 'ABANDONED - pocket gave an implausible mass (%.1f g)' % m
                out['wedges'][nm] = info
                continue

        after_body = find_body(root, nm)
        after = round(after_body.physicalProperties.mass * 1000.0, 2)
        info['mass_after_g'] = after
        info['saved_g'] = round(before - after, 2)
        info['wall_mm'] = t

        # ---- the hold-down holes are only 1 mm deep; make them real tapped holes
        holes = find_vertical_holes(after_body, M3_CLR, tol_mm=0.6)
        info['holddown_positions_mm'] = [list(h) for h in holes]
        if holes and not has_sketch(comp, 'Wedge_HoldDown_Tap'):
            cut_holes_z(root, nm, holes, M3_TAP, z0, 10.0, 'Wedge_HoldDown_Tap')
            info['holddown'] = 'existing 1 mm holes deepened to 10 mm tapped M3'
        info['status'] = 'POCKETED'
        out['wedges'][nm] = info

    saved = sum(v.get('saved_g', 0) for v in out['wedges'].values())
    out['total_saved_g'] = round(saved, 2)
    out['status'] = 'DONE'
    rep.step('17_hollow_wedges', out)
    rep.say('Wedges pocketed: %.1f g saved' % saved)


# --------------- STEP 18: weapon shaft 3.17 -> 4 mm (matches the 624)

def step_shaft_4mm(app, design, root, rep):
    """`spinner_bearing_OD` 13 mm is a 624, which has a 4 mm bore, but the shaft
    was 3.17 mm (1/8"). They do not fit each other.

    Going to a 4 mm shaft is the better of the three fixes: it keeps a commodity
    bearing and puts more metal in the part that takes the strike loads.
    """
    out = {}
    up = design.userParameters
    p = up.itemByName('spinner_shaft_D')
    if p is None:
        out['status'] = 'skipped - spinner_shaft_D parameter not found'
        rep.step('18_shaft_4mm', out)
        return
    out['before_mm'] = round(CM(p.value), 3)
    if abs(CM(p.value) - 4.0) < 0.01:
        out['status'] = 'already 4 mm'
    else:
        p.expression = '4 mm'
        out['status'] = 'PARAMETER SET'

    # the earlier build had a stale-solve bug, so confirm the solid moved too
    b = find_body(root, 'Spinner_Shaft')
    if b is not None:
        bb = bbox_mm(b)
        dia = round(min(bb['y'][1] - bb['y'][0], bb['z'][1] - bb['z'][0]), 3)
        out['solid_diameter_mm'] = dia
        out['solid_matches_parameter'] = abs(dia - 4.0) < 0.05
        if not out['solid_matches_parameter']:
            out['status'] = ('PARAMETER SET BUT SOLID DID NOT REGENERATE '
                             '(still %.2f mm) - stale solve, needs a rebuild' % dia)
        pin = up.itemByName('roll_pin_d')
        if pin is not None:
            pd = CM(pin.value)
            out['roll_pin'] = {
                'pin_d_mm': round(pd, 2),
                'shaft_d_mm': dia,
                'wall_each_side_mm': round((dia - pd) / 2.0, 2),
                'd_over_D': round(pd / dia, 2),
                'note': ('wall improves from %.2f mm (on the old 3.17 shaft) to '
                         '%.2f mm, but d/D = %.2f is still high - usual drive-pin '
                         'practice is 0.25-0.33. Left as-is deliberately: a smaller '
                         'pin trades shaft section for shear area on a weapon joint '
                         'that already needs a human review.'
                         % ((3.17 - pd) / 2.0, (dia - pd) / 2.0, pd / dia)),
            }
    rep.step('18_shaft_4mm', out)
    rep.say('Weapon shaft: %s -> %s mm' % (out.get('before_mm'), out.get('solid_diameter_mm')))


# --------------- STEP 19: 1:1 pulleys and belt-tension slots

PULLEY_OD = 20.0
PULLEY_W = 6.0
PULLEY_X0 = 16.0
TENSION_SLOT = 3.0      # mm half-length of the slot; with a 3 mm screw that
                        # is +/-1.5 mm of real travel (need 0.67 mm for a 160 mm belt)


def step_pulleys(app, design, root, rep):
    """Model the belt drive as equal pulleys - the neutral choice - and slot the
    cradle's base-plate holes so the belt can actually be tensioned.

    A 1:1 GT2 pair at this 49.25 mm centre distance wants a 161.3 mm belt; the
    nearest stock length is 160 mm, which is 0.65 mm short. Without slots there
    is no way to take that up.
    """
    out = {}
    shaft = find_body(root, 'Spinner_Shaft')
    motor = find_body(root, 'Spinner_Motor')
    spin = find_occ(root, '03_Front_Spinner')
    if None in (shaft, motor, spin):
        out['status'] = 'skipped - spinner bodies not found'
        rep.step('19_pulleys', out)
        return

    sb, mb = bbox_mm(shaft), bbox_mm(motor)
    shaft_y = (sb['y'][0] + sb['y'][1]) / 2.0
    shaft_z = (sb['z'][0] + sb['z'][1]) / 2.0
    motor_y = (mb['y'][0] + mb['y'][1]) / 2.0
    motor_z = (mb['z'][0] + mb['z'][1]) / 2.0
    centre_distance = abs(shaft_y - motor_y)
    out['centre_distance_mm'] = round(centre_distance, 2)
    out['ratio'] = '1:1 (equal pulleys)'
    belt = 2 * centre_distance + math.pi * PULLEY_OD
    out['belt'] = {'required_mm': round(belt, 1),
                   'nearest_gt2_stock_mm': 160,
                   'shortfall_mm': round(belt - 160, 2)}

    mat = get_lib_material(app, '', exact='Aluminum 6061')
    made = []
    for nm, py, pz in (('Weapon_Pulley', shaft_y, shaft_z),
                       ('Motor_Pulley', motor_y, motor_z)):
        tgt = {'x': [PULLEY_X0, PULLEY_X0 + PULLEY_W],
               'y': [py - PULLEY_OD / 2, py + PULLEY_OD / 2],
               'z': [pz - PULLEY_OD / 2, pz + PULLEY_OD / 2]}
        clash = [bn for cn, bn, b2 in all_bodies(root)
                 if bn not in ('Spinner_Shaft', 'Spinner_Motor', nm)
                 and boxes_overlap(tgt, bbox_mm(b2), 0.05)]
        if clash:
            out.setdefault('skipped', []).append({'pulley': nm, 'clash': sorted(set(clash))})
            continue
        for o in list(spin.component.occurrences):
            if o.component.name == nm:
                o.deleteMe()
        occ = spin.component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = nm
        comp = occ.component
        sk = comp.sketches.add(comp.yZConstructionPlane)
        sk.name = nm + '_Sketch'
        sk.sketchCurves.sketchCircles.addByCenterRadius(
            sketch_pt(sk, 0.0, py, pz), MM(PULLEY_OD / 2.0))
        e = comp.features.extrudeFeatures.createInput(
            sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        e.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(MM(PULLEY_X0)))
        e.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(PULLEY_W)))
        body = e and comp.features.extrudeFeatures.add(e).bodies.item(0)
        body.name = nm
        if abs(bbox_mm(body)['x'][0] - PULLEY_X0) > 0.5:
            pass
        if mat:
            body.material = mat
        made.append({'name': nm, 'bbox_mm': bbox_mm(body),
                     'mass_g': round(body.physicalProperties.mass * 1000.0, 2)})
    out['pulleys'] = made

    # ---- tension slots in the base plate under the cradle -----------------
    cr = find_body(root, CRADLE_NAME)
    slots = []
    if cr is not None:
        for (bx, by) in [(-CRADLE_BOLT_INSET, 16.0), (CRADLE_BOLT_INSET, 16.0),
                         (-CRADLE_BOLT_INSET, 37.0), (CRADLE_BOLT_INSET, 37.0)]:
            r = cut_rect_z(root, 'Base_Plate_Al',
                           bx - CRADLE_BOLT_CLR_D / 2.0, bx + CRADLE_BOLT_CLR_D / 2.0,
                           by - TENSION_SLOT, by + TENSION_SLOT,
                           bbox_mm(find_body(root, 'Base_Plate_Al'))['z'][0] - 2.0, 6.0,
                           'Tension_Slot_%.0f_%.0f' % (bx, by))
            slots.append({'at': [bx, by], 'action': r})
    out['tension_slots'] = slots
    out['slot_travel_mm'] = '+/- %.1f (screw travel in a %.1f mm slot)' % (TENSION_SLOT - 1.5, 2 * TENSION_SLOT)
    out['status'] = 'BUILT'
    out['caveat'] = ('1:1 is modelled as a PLACEHOLDER ratio, not a chosen design '
                     'point. Weapon energy scales with the square of the ratio, so '
                     'the ~10 J figure is only valid at 1:1. Confirm against your '
                     'event rulebook before spinning it.')
    rep.step('19_pulleys', out)
    rep.say('Pulleys: %d built, belt %.1f mm, tension slots +/-%.0f mm'
            % (len(made), belt, TENSION_SLOT))


# --------- STEP 20: bores, hammer overlaps, and the ESC T-posts

def overlap_mm3(root, a, b):
    tbm = adsk.fusion.TemporaryBRepManager.get()
    ba, bb = find_body(root, a), find_body(root, b)
    if ba is None or bb is None:
        return None
    ta, tb = tbm.copy(ba), tbm.copy(bb)
    try:
        tbm.booleanOperation(ta, tb, adsk.fusion.BooleanTypes.IntersectionBooleanType)
        return round(ta.volume * 1000.0, 3)
    except Exception:
        return 0.0


def step_bores_and_hammer(app, design, root, rep):
    """Parts that a shaft passes through were modelled as solids with no bore,
    and the hammer arm was sunk into both the servo and the head. None of that
    can be made. Each fix is checked by re-measuring the true overlap."""
    out = {}
    shaft = find_body(root, 'Spinner_Shaft')
    motor = find_body(root, 'Spinner_Motor')
    sb, mb = bbox_mm(shaft), bbox_mm(motor)
    sy, sz = (sb['y'][0] + sb['y'][1]) / 2.0, (sb['z'][0] + sb['z'][1]) / 2.0
    my, mz = (mb['y'][0] + mb['y'][1]) / 2.0, (mb['z'][0] + mb['z'][1]) / 2.0
    shaft_d = round(min(sb['y'][1] - sb['y'][0], sb['z'][1] - sb['z'][0]), 2)

    bores = {}
    for nm in ('Spinner_Bar', 'Weapon_Pulley', 'Left_Support', 'Right_Support'):
        before = overlap_mm3(root, 'Spinner_Shaft', nm)
        r = cut_hole_x(root, nm, sy, sz, shaft_d, 'Shaft_Bore')
        bores[nm] = {'bore_d_mm': shaft_d, 'action': r,
                     'overlap_before_mm3': before,
                     'overlap_after_mm3': overlap_mm3(root, 'Spinner_Shaft', nm)}
    # the A2212's own output shaft is 3.17 mm; it is not modelled, but the
    # pulley still needs the bore
    r = cut_hole_x(root, 'Motor_Pulley', my, mz, 3.17, 'Shaft_Bore')
    bores['Motor_Pulley'] = {'bore_d_mm': 3.17, 'action': r,
                             'note': 'A2212 output shaft is 3.17 mm'}
    out['bores'] = bores

    # ---- hammer: lift arm + head onto the servo top ---------------------
    ham = {}
    servo = find_body(root, 'Hammer_Servo')
    arm = find_body(root, 'Hammer_Arm')
    head = find_body(root, 'Hammer_Head')
    if None not in (servo, arm, head):
        ham['servo_arm_before_mm3'] = overlap_mm3(root, 'Hammer_Servo', 'Hammer_Arm')
        ham['arm_head_before_mm3'] = overlap_mm3(root, 'Hammer_Arm', 'Hammer_Head')
        lift = round(bbox_mm(servo)['z'][1] - bbox_mm(arm)['z'][0], 3)
        ham['lift_mm'] = lift
        if lift > 0.01:
            for nm in ('Hammer_Arm', 'Hammer_Head'):
                o, c, n = find_owner(root, nm)
                move_body(c, n, 0.0, 0.0, lift)
        ham['servo_arm_after_mm3'] = overlap_mm3(root, 'Hammer_Servo', 'Hammer_Arm')

        # socket in the head so the arm plugs in instead of passing through it
        ab, hb = bbox_mm(find_body(root, 'Hammer_Arm')), bbox_mm(find_body(root, 'Hammer_Head'))
        y_lo = max(ab['y'][0], hb['y'][0])
        y_hi = min(ab['y'][1], hb['y'][1])
        o, c, n = find_owner(root, 'Hammer_Head')
        if y_hi > y_lo and not has_sketch(c, 'Arm_Socket'):
            sk = c.sketches.add(c.xZConstructionPlane)
            sk.name = 'Arm_Socket'
            sk.sketchCurves.sketchLines.addTwoPointRectangle(
                sketch_pt(sk, ab['x'][0], 0.0, ab['z'][0]),
                sketch_pt(sk, ab['x'][1], 0.0, ab['z'][1]))

            def socket(y0, dist):
                ci = c.features.extrudeFeatures.createInput(
                    sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
                ci.startExtent = adsk.fusion.OffsetStartDefinition.create(
                    adsk.core.ValueInput.createByReal(MM(y0)))
                ci.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(dist)))
                ci.participantBodies = [n]
                return c.features.extrudeFeatures.add(ci)

            depth = y_hi - y_lo
            tries = [(y_hi, -depth), (y_lo, depth), (-y_hi, depth), (-y_lo, -depth)]
            done = False
            for y0, dist in tries:
                f = socket(y0, dist)
                if overlap_mm3(root, 'Hammer_Arm', 'Hammer_Head') < 1.0:
                    done = True
                    break
                f.deleteMe()
            ham['socket'] = 'cut, %.1f mm deep' % depth if done else 'FAILED to orient'
        ham['arm_head_after_mm3'] = overlap_mm3(root, 'Hammer_Arm', 'Hammer_Head')
        ham['head_mass_g'] = round(find_body(root, 'Hammer_Head').physicalProperties.mass * 1000.0, 2)
    out['hammer'] = ham

    # ---- ESC: replace the two plain posts with T-posts ------------------
    esc = find_body(root, 'Spinner_ESC')
    base = find_body(root, 'Base_Plate_Al')
    printed = find_occ(root, '06_3D_Printed')
    t = {}
    if None not in (esc, base, printed):
        eb = bbox_mm(esc)
        base_top = bbox_mm(base)['z'][1]
        bar_t, bar_half_x, bar_half_y = 2.0, 20.0, 3.0
        bar_z0 = eb['z'][0] - bar_t
        pos = [(0.0, -9.0), (0.0, 7.0)]
        clash = []
        for (px, py) in pos:
            bar = {'x': [px - bar_half_x, px + bar_half_x],
                   'y': [py - bar_half_y, py + bar_half_y], 'z': [bar_z0, eb['z'][0]]}
            col = {'x': [px - 3.5, px + 3.5], 'y': [py - 3.5, py + 3.5],
                   'z': [base_top, bar_z0]}
            for cn, bn, b2 in all_bodies(root):
                if bn in ('Base_Plate_Al', 'Spinner_ESC') or bn.startswith('ESC_Post'):
                    continue
                if boxes_overlap(bar, bbox_mm(b2), 0.05) or boxes_overlap(col, bbox_mm(b2), 0.05):
                    clash.append('%s hits %s' % (str([px, py]), bn))
        t['clashes'] = sorted(set(clash))
        if clash:
            t['status'] = 'ABANDONED - kept the plain posts'
        else:
            mat = get_lib_material(app, '', exact='Nylon 6')
            for i, (px, py) in enumerate(pos, 1):
                nm = 'ESC_Post_%d' % i
                o2, c2, body = make_post(printed.component, nm, px, py, base_top,
                                         bar_z0, 6.0, mat)
                sk = c2.sketches.add(c2.xYConstructionPlane)
                sk.name = nm + '_Crossbar'
                sk.sketchCurves.sketchLines.addTwoPointRectangle(
                    sketch_pt(sk, px - bar_half_x, py - bar_half_y, 0.0),
                    sketch_pt(sk, px + bar_half_x, py + bar_half_y, 0.0))
                e = c2.features.extrudeFeatures.createInput(
                    sk.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
                e.startExtent = adsk.fusion.OffsetStartDefinition.create(
                    adsk.core.ValueInput.createByReal(MM(bar_z0)))
                e.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(bar_t)))
                e.participantBodies = [body]
                c2.features.extrudeFeatures.add(e)
                cut_holes_z(root, nm, [(px, py)], M2_5_PILOT, eb['z'][0], -8.0, 'Pilot_' + nm)
            t['status'] = 'BUILT'
            t['crossbars'] = '40 x 6 x 2 mm at z %.1f-%.1f, under both ends of the board' % (bar_z0, eb['z'][0])
    out['esc_tposts'] = t
    out['status'] = 'DONE'
    rep.step('20_bores_hammer_esc', out)
    rep.say('Bores, hammer overlaps and ESC T-posts done')


# ------------- STEP 21: real electronics + printed holders for them

# (name, L, W, H in mm, preferred centre, parent component)
REAL_PARTS = [
    ('Rx_ER6',      43.0, 25.0, 15.0, (0.0, -78.0),  '05_Electronics'),
    ('UBEC_5A',     50.0, 17.0, 10.0, (50.0, -38.0), '05_Electronics'),
    ('DRV8874_A',   17.8, 15.2,  3.0, (-50.0, -35.0), '05_Electronics'),
    ('DRV8874_B',   17.8, 15.2,  3.0, (-50.0, -55.0), '05_Electronics'),
    ('ESC_Tekko32', 34.3, 17.3,  4.5, (25.0, 28.0),  '03_Front_Spinner'),
]
MOCKS_TO_REMOVE = ['RC_Receiver', 'Wheel_Motor_Drivers', 'Power_Buck_BEC', 'Spinner_ESC',
                   'ESC_Post_1', 'ESC_Post_2', 'Shield_RC_Receiver',
                   'Shield_Wheel_Motor_Drivers', 'Shield_Power_Buck_BEC']

H_CLR = 0.2        # mm per side between part and pocket wall
H_WALL = 1.6
H_FLOOR = 2.0
H_TAB_LEN = 7.0
H_TAB_T = 4.0      # thick enough for an M3 self-tapping screw from below
H_TIE_W = 4.0      # underside groove for the cable tie
H_TIE_D = 1.2
H_GAP = 1.0        # keep-away from anything else


def delete_components_named(root, names):
    deleted = []

    def walk(comp):
        for o in list(comp.occurrences):
            nm = o.component.name
            if nm in names:
                deleted.append(nm)
                o.deleteMe()
            else:
                walk(o.component)
    walk(root)
    return deleted


def make_box(parent_comp, name, x0, x1, y0, y1, z0, z1, mat):
    for o in list(parent_comp.occurrences):
        if o.component.name == name:
            o.deleteMe()
    occ = parent_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = name + '_Sketch'
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        sketch_pt(sk, x0, y0, 0.0), sketch_pt(sk, x1, y1, 0.0))
    e = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    e.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(z0)))
    e.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(z1 - z0)))
    body = comp.features.extrudeFeatures.add(e).bodies.item(0)
    body.name = name
    if mat:
        body.material = mat
    return occ, comp, body


def _holder_layout(L, W, H, cx, cy, rot):
    """Axis-aligned rectangles (world mm) for one holder. rot=0: L along X."""
    half_a = L / 2.0 + H_CLR + H_WALL
    half_b = W / 2.0 + H_CLR + H_WALL
    tab_b = min(half_b, 6.0)

    def rect(a0, a1, b0, b1):
        if rot == 0:
            return (cx + a0, cx + a1, cy + b0, cy + b1)
        return (cx + b0, cx + b1, cy + a0, cy + a1)

    def pt(a, b):
        return (cx + a, cy + b) if rot == 0 else (cx + b, cy + a)

    return {
        'block': rect(-half_a, half_a, -half_b, half_b),
        'tabs': [rect(-half_a - H_TAB_LEN, -half_a, -tab_b, tab_b),
                 rect(half_a, half_a + H_TAB_LEN, -tab_b, tab_b)],
        'pocket': rect(-L / 2.0 - H_CLR, L / 2.0 + H_CLR, -W / 2.0 - H_CLR, W / 2.0 + H_CLR),
        'part': rect(-L / 2.0, L / 2.0, -W / 2.0, W / 2.0),
        'groove': rect(-H_TIE_W / 2.0, H_TIE_W / 2.0, -half_b - 1.0, half_b + 1.0),
        'holes': [pt(-half_a - H_TAB_LEN / 2.0, 0.0), pt(half_a + H_TAB_LEN / 2.0, 0.0)],
        'extent': rect(-half_a - H_TAB_LEN, half_a + H_TAB_LEN, -half_b, half_b),
    }


def step_real_electronics(app, design, root, rep):
    """Swap the 25x20x10 placeholder boxes for the real parts chosen in
    exports/Electronics_Parts_Selection.md, and give each one a printed holder.

    None of these four parts has a mounting hole, so the holder is the mount:
    a pocket with walls 1 mm lower than the part (so a cable tie bears on the
    part), an underside groove the tie can pass through without being pinched
    by the base plate, and two 4 mm tabs taking M3 screws up from below.
    Positions are found by searching the base plate for free space, nearest to
    where each old placeholder sat.
    """
    out = {'parts': {}}
    base = find_body(root, 'Base_Plate_Al')
    base_occ = find_occ(root, 'Base_Plate_Al')
    printed = find_occ(root, '06_3D_Printed')
    base_top = bbox_mm(base)['z'][1]
    bz = bbox_mm(base)['z']

    # ---- clear out what the real parts make obsolete ---------------------
    out['removed_components'] = delete_components_named(root, MOCKS_TO_REMOVE)
    out['removed_m2_plate_holes'] = delete_sketch_and_feature(
        base_occ.component, 'Module_Mount_Holes_Sketch')

    ENV_OK = ('Base_Plate_Al',)
    obstacles = [(bn, bbox_mm(b)) for cn, bn, b in all_bodies(root) if bn not in ENV_OK]

    mat_part = get_lib_material(app, '', exact='ABS Plastic')
    mat_hold = get_lib_material(app, '', exact='Nylon 6')

    def clear_of_obstacles(ext, z1):
        e = {'x': [ext[0] - H_GAP, ext[1] + H_GAP],
             'y': [ext[2] - H_GAP, ext[3] + H_GAP], 'z': [base_top + 0.05, z1 + H_GAP]}
        for bn, bb in obstacles:
            if boxes_overlap(e, bb, 0.0):
                return False
        return True

    def on_plate(lay):
        zmid = (bz[0] + bz[1]) / 2.0
        pts = []
        for r in [lay['block']] + lay['tabs']:
            pts += [(r[0] + 0.5, r[2] + 0.5), (r[1] - 0.5, r[2] + 0.5),
                    (r[0] + 0.5, r[3] - 0.5), (r[1] - 0.5, r[3] - 0.5)]
        for (px, py) in pts:
            p = adsk.core.Point3D.create(MM(px), MM(py), MM(zmid))
            if base.pointContainment(p) != adsk.fusion.PointContainment.PointInsidePointContainment:
                return False
        for (hx, hy) in lay['holes']:
            if not plate_supports_bolt(base, hx, hy, zmid, M3_CLR):
                return False
        return True

    for (name, L, W, H, pref, parent) in REAL_PARTS:
        info = {'size_mm': [L, W, H], 'preferred': list(pref)}
        z_top = base_top + H_FLOOR + H + 1.0
        cands = []
        x = -76.0
        while x <= 76.0:
            y = -86.0
            while y <= 86.0:
                for rot in (0, 1):
                    cands.append(((x - pref[0]) ** 2 + (y - pref[1]) ** 2, x, y, rot))
                y += 2.0
            x += 2.0
        cands.sort()
        chosen = None
        tested = 0
        for d2, x, y, rot in cands:
            lay = _holder_layout(L, W, H, x, y, rot)
            if not clear_of_obstacles(lay['extent'], z_top):
                continue
            tested += 1
            if on_plate(lay):
                chosen = (x, y, rot, lay)
                break
            if tested > 400:
                break
        if chosen is None:
            info['status'] = 'NO FREE SPACE FOUND on the base plate'
            out['parts'][name] = info
            continue

        x, y, rot, lay = chosen
        info['centre_mm'] = [x, y]
        info['moved_from_preferred_mm'] = round(math.sqrt((x - pref[0]) ** 2 + (y - pref[1]) ** 2), 1)
        info['orientation'] = 'L along X' if rot == 0 else 'L along Y'

        # the part itself, at real size, sitting on the holder floor
        par = find_occ(root, parent)
        pr = lay['part']
        make_box(par.component, name, pr[0], pr[1], pr[2], pr[3],
                 base_top + H_FLOOR, base_top + H_FLOOR + H, mat_part)

        # the holder
        hname = 'Holder_' + name
        blk = lay['block']
        wall_top = base_top + H_FLOOR + max(H - 1.0, 1.5)
        occ, hcomp, hbody = make_box(printed.component, hname, blk[0], blk[1], blk[2], blk[3],
                                     base_top, wall_top, mat_hold)
        sk = hcomp.sketches.add(hcomp.xYConstructionPlane)
        sk.name = hname + '_Tabs'
        for t in lay['tabs']:
            sk.sketchCurves.sketchLines.addTwoPointRectangle(
                sketch_pt(sk, t[0], t[2], 0.0), sketch_pt(sk, t[1], t[3], 0.0))
        col = adsk.core.ObjectCollection.create()
        for i in range(sk.profiles.count):
            col.add(sk.profiles.item(i))
        e = hcomp.features.extrudeFeatures.createInput(
            col, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        e.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(MM(base_top)))
        e.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(H_TAB_T)))
        e.participantBodies = [hbody]
        hcomp.features.extrudeFeatures.add(e)

        pk = lay['pocket']
        cut_rect_z(root, hname, pk[0], pk[1], pk[2], pk[3],
                   base_top + H_FLOOR, 60.0, hname + '_Pocket')
        g = lay['groove']
        cut_rect_z(root, hname, g[0], g[1], g[2], g[3],
                   base_top - 1.0, 1.0 + H_TIE_D, hname + '_TieGroove')
        cut_holes_z(root, hname, lay['holes'], M3_TAP, base_top - 1.0, H_TAB_T + 2.0,
                    hname + '_TabPilots')
        cut_holes_z(root, 'Base_Plate_Al', lay['holes'], M3_CLR, bz[0] - 2.0, 6.0,
                    hname + '_PlateHoles')

        hb = find_body(root, hname)
        info['holder_mass_g'] = round(hb.physicalProperties.mass * 1000.0, 2)
        info['holder_bbox_mm'] = bbox_mm(hb)
        info['screw_holes_mm'] = [[round(a, 1), round(b, 1)] for a, b in lay['holes']]
        info['status'] = 'BUILT'
        out['parts'][name] = info

        # later parts must avoid this one
        obstacles.append((name, bbox_mm(find_body(root, name))))
        obstacles.append((hname, bbox_mm(hb)))

    out['status'] = 'DONE'
    rep.step('21_real_electronics', out)
    rep.say('Real electronics: %d/%d placed with holders'
            % (sum(1 for v in out['parts'].values() if v.get('status') == 'BUILT'),
               len(REAL_PARTS)))


# ------------- STEP 21b: compact holders, screwed down through the floor

REAL_PARTS_V2 = [
    # name, L, W, H, preferred centre (None = next to the previous part), parent
    ('ESC_Tekko32', 34.3, 17.3,  4.5, (25.0, 28.0),  '03_Front_Spinner'),
    ('Rx_ER6',      43.0, 25.0, 15.0, (0.0, -78.0),  '05_Electronics'),
    ('UBEC_5A',     50.0, 17.0, 10.0, (45.0, -40.0), '05_Electronics'),
    ('DRV8874_A',   17.8, 15.2,  3.0, (-45.0, -40.0), '05_Electronics'),
    ('DRV8874_B',   17.8, 15.2,  3.0, None,          '05_Electronics'),
]


def _holder_layout2(L, W, H, cx, cy, rot):
    """No tabs: two M3 screws go down through the holder floor, under where the
    part will sit, into tapped holes in the base plate. The holder goes in
    first, then the part on foam tape, then a cable tie through the groove."""
    half_a = L / 2.0 + H_CLR + H_WALL
    half_b = W / 2.0 + H_CLR + H_WALL
    hole_a = max(L / 2.0 - 4.0, H_TIE_W / 2.0 + 3.0)

    def rect(a0, a1, b0, b1):
        return (cx + a0, cx + a1, cy + b0, cy + b1) if rot == 0 else (cx + b0, cx + b1, cy + a0, cy + a1)

    def pt(a, b):
        return (cx + a, cy + b) if rot == 0 else (cx + b, cy + a)

    return {
        'block': rect(-half_a, half_a, -half_b, half_b),
        'pocket': rect(-L / 2.0 - H_CLR, L / 2.0 + H_CLR, -W / 2.0 - H_CLR, W / 2.0 + H_CLR),
        'part': rect(-L / 2.0, L / 2.0, -W / 2.0, W / 2.0),
        'groove': rect(-H_TIE_W / 2.0, H_TIE_W / 2.0, -half_b - 1.0, half_b + 1.0),
        'holes': [pt(-hole_a, 0.0), pt(hole_a, 0.0)],
        'extent': rect(-half_a, half_a, -half_b, half_b),
    }


def step_real_electronics2(app, design, root, rep):
    """Second layout pass. The tabbed holders from the first pass were 14 mm
    longer than they needed to be and the UBEC found no room at all, so the
    tabs go: screws run through the holder floor instead. Idempotent - it
    clears its own earlier output (parts, holders and their plate holes)
    before placing anything."""
    out = {'parts': {}}
    base = find_body(root, 'Base_Plate_Al')
    base_occ = find_occ(root, 'Base_Plate_Al')
    printed = find_occ(root, '06_3D_Printed')
    bz = bbox_mm(base)['z']
    base_top = bz[1]

    names = [p[0] for p in REAL_PARTS_V2]
    out['removed_components'] = delete_components_named(
        root, MOCKS_TO_REMOVE + names + ['Holder_' + n for n in names])
    cleaned = []
    for s in list(base_occ.component.sketches):
        if s.name.endswith('_PlateHoles') or s.name == 'Module_Mount_Holes_Sketch':
            cleaned.append((s.name, delete_sketch_and_feature(base_occ.component, s.name)))
    out['cleaned_plate_holes'] = cleaned

    obstacles = [(bn, bbox_mm(b)) for cn, bn, b in all_bodies(root) if bn != 'Base_Plate_Al']
    mat_part = get_lib_material(app, '', exact='ABS Plastic')
    mat_hold = get_lib_material(app, '', exact='Nylon 6')
    zmid = (bz[0] + bz[1]) / 2.0

    def clear(ext, z1):
        e = {'x': [ext[0] - H_GAP, ext[1] + H_GAP],
             'y': [ext[2] - H_GAP, ext[3] + H_GAP], 'z': [base_top + 0.05, z1 + H_GAP]}
        return not any(boxes_overlap(e, bb, 0.0) for bn, bb in obstacles)

    def on_plate(lay):
        r = lay['block']
        for (px, py) in ((r[0] + 0.5, r[2] + 0.5), (r[1] - 0.5, r[2] + 0.5),
                         (r[0] + 0.5, r[3] - 0.5), (r[1] - 0.5, r[3] - 0.5),
                         ((r[0] + r[1]) / 2, (r[2] + r[3]) / 2)):
            p = adsk.core.Point3D.create(MM(px), MM(py), MM(zmid))
            if base.pointContainment(p) != adsk.fusion.PointContainment.PointInsidePointContainment:
                return False
        return all(plate_supports_bolt(base, hx, hy, zmid, M3_CLR) for hx, hy in lay['holes'])

    prev = None
    for (name, L, W, H, pref, parent) in REAL_PARTS_V2:
        pref = pref or prev
        info = {'size_mm': [L, W, H], 'preferred': list(pref)}
        z_top = base_top + H_FLOOR + H + 1.0
        cands = []
        x = -76.0
        while x <= 76.0:
            y = -86.0
            while y <= 86.0:
                for rot in (0, 1):
                    cands.append(((x - pref[0]) ** 2 + (y - pref[1]) ** 2, x, y, rot))
                y += 1.0
            x += 1.0
        cands.sort()
        chosen = None
        tested = 0
        for d2, x, y, rot in cands:
            lay = _holder_layout2(L, W, H, x, y, rot)
            if not clear(lay['extent'], z_top):
                continue
            tested += 1
            if on_plate(lay):
                chosen = (x, y, rot, lay)
                break
            if tested > 3000:
                break
        if chosen is None:
            info['status'] = 'NO FREE SPACE FOUND on the base plate'
            out['parts'][name] = info
            continue

        x, y, rot, lay = chosen
        prev = (x, y)
        info['centre_mm'] = [x, y]
        info['moved_from_preferred_mm'] = round(math.sqrt((x - pref[0]) ** 2 + (y - pref[1]) ** 2), 1)
        info['orientation'] = 'L along X' if rot == 0 else 'L along Y'

        pr = lay['part']
        make_box(find_occ(root, parent).component, name, pr[0], pr[1], pr[2], pr[3],
                 base_top + H_FLOOR, base_top + H_FLOOR + H, mat_part)

        hname = 'Holder_' + name
        blk = lay['block']
        wall_top = base_top + H_FLOOR + max(H - 1.0, 1.5)
        make_box(printed.component, hname, blk[0], blk[1], blk[2], blk[3],
                 base_top, wall_top, mat_hold)
        pk = lay['pocket']
        cut_rect_z(root, hname, pk[0], pk[1], pk[2], pk[3],
                   base_top + H_FLOOR, 60.0, hname + '_Pocket')
        g = lay['groove']
        cut_rect_z(root, hname, g[0], g[1], g[2], g[3],
                   base_top - 1.0, 1.0 + H_TIE_D, hname + '_TieGroove')
        cut_holes_z(root, hname, lay['holes'], M3_CLR, base_top - 1.0, H_FLOOR + 2.0,
                    hname + '_FloorHoles')
        cut_holes_z(root, 'Base_Plate_Al', lay['holes'], M3_TAP, bz[0] - 2.0, 6.0,
                    hname + '_PlateHoles')

        hb = find_body(root, hname)
        info['holder_mass_g'] = round(hb.physicalProperties.mass * 1000.0, 2)
        info['holder_size_mm'] = [round(blk[1] - blk[0], 1), round(blk[3] - blk[2], 1),
                                  round(wall_top - base_top, 1)]
        info['screw_holes_mm'] = [[round(a, 1), round(b, 1)] for a, b in lay['holes']]
        info['status'] = 'BUILT'
        out['parts'][name] = info
        obstacles.append((name, bbox_mm(find_body(root, name))))
        obstacles.append((hname, bbox_mm(hb)))

    out['status'] = 'DONE'
    out['assembly'] = ('Holder first: 2x M3 countersunk down through the holder floor '
                       'into M3 tapped holes in the base plate. Then the part on foam '
                       'tape, then a cable tie around part + holder through the underside '
                       'groove. Note: M3 in 2 mm aluminium is only ~4 threads - fine for '
                       'these light parts, use rivnuts if the plate stays 2 mm and they loosen.')
    rep.step('21_real_electronics', out)
    rep.say('Real electronics: %d/%d placed'
            % (sum(1 for v in out['parts'].values() if v.get('status') == 'BUILT'),
               len(REAL_PARTS_V2)))


# ---------------- STEP 22: the real ESP32 DevKit on a printed carrier

# 30-pin ESP32-WROOM-32 DevKit, CH9102X, micro-USB, 4 corner holes.
# Board size agrees with the seller's 51.8 x 28.2. The hole pattern is NOT
# published - it was measured off the user's photo using the 2.54 mm header
# pitch as scale (checked against the 18 mm WROOM module width). Treat it as
# +/-0.5 mm, which is why it lives in a printed carrier, not the aluminium.
ESP_L, ESP_W, ESP_T = 51.5, 28.3, 1.6
ESP_HOLE_DX, ESP_HOLE_DY = 47.2, 23.4
ESP_MODULE = (25.5, 18.0, 3.1)       # WROOM-32 incl. antenna, sits at the -X end
ESP_HEADER_ROW = 25.4                 # row spacing
ESP_HEADER_SPAN = 35.6                # 15 pins x 2.54
ESP_HEADER_X0 = 6.3                   # first pin from the -X edge
ESP_UNDER = 18.0                      # room under the board for DuPont housings
ESP_CENTRE = (0.0, -38.0)
CARRIER_T = 2.0
ESP_POST_OD = 4.5
M2_PILOT = 1.6


def step_esp32_devkit(app, design, root, rep):
    """Replace the 60x50 placeholder PCB (and its keep-out, standoffs and
    plate holes) with the real DevKit on a printed carrier."""
    out = {}
    up = design.userParameters
    for nm in ('base_t', 'side_t'):
        p = up.itemByName(nm)
        if p is not None:
            p.comment = 'DECIDED 2026-09-23: 2 mm aluminium plate (user approved)'
    out['plate_thickness'] = '2 mm - confirmed'

    base = find_body(root, 'Base_Plate_Al')
    base_occ = find_occ(root, 'Base_Plate_Al')
    bz = bbox_mm(base)['z']
    base_top = bz[1]
    elec = find_occ(root, '05_Electronics')
    printed = find_occ(root, '06_3D_Printed')

    out['removed'] = delete_components_named(
        root, ['ESP32_Control_Board', 'PCB_Keepout_Envelope', 'PCB_Standoff_1',
               'PCB_Standoff_2', 'PCB_Standoff_3', 'PCB_Standoff_4',
               'ESP32_DevKit', 'Holder_ESP32'])
    out['removed_plate_holes'] = {
        n: delete_sketch_and_feature(base_occ.component, n)
        for n in ('PCB_Standoff_BaseHoles', 'Holder_ESP32_PlateHoles')}

    cx, cy = ESP_CENTRE
    x0, y0 = cx - ESP_L / 2.0, cy - ESP_W / 2.0          # board corner (-X,-Y)
    board_z0 = base_top + CARRIER_T + ESP_UNDER
    holes = [(cx + sx * ESP_HOLE_DX / 2.0, cy + sy * ESP_HOLE_DY / 2.0)
             for sx in (-1, 1) for sy in (-1, 1)]
    carrier_screws = [(cx - 15.0, cy), (cx + 15.0, cy)]
    margin = 2.0

    # ---- probe the whole envelope first ---------------------------------
    env = {'x': [x0 - margin, x0 + ESP_L + margin],
           'y': [y0 - margin, y0 + ESP_W + margin],
           'z': [base_top + 0.05, board_z0 + ESP_T + ESP_MODULE[2]]}
    clash = [bn for cn, bn, b in all_bodies(root)
             if bn != 'Base_Plate_Al' and boxes_overlap(env, bbox_mm(b), 0.05)]
    out['envelope_mm'] = env
    out['clashes'] = sorted(set(clash))
    if clash:
        out['status'] = 'ABANDONED - %s in the way' % ', '.join(sorted(set(clash)))
        rep.step('22_esp32', out)
        return
    zmid = (bz[0] + bz[1]) / 2.0
    bad = [list(h) for h in carrier_screws if not plate_supports_bolt(base, h[0], h[1], zmid, M3_CLR)]
    if bad:
        out['status'] = 'ABANDONED - carrier screws would land on existing holes %s' % bad
        rep.step('22_esp32', out)
        return

    mat_abs = get_lib_material(app, '', exact='ABS Plastic')
    mat_ny = get_lib_material(app, '', exact='Nylon 6')

    # ---- the board -------------------------------------------------------
    occ, ecomp, board = make_box(elec.component, 'ESP32_DevKit', x0, x0 + ESP_L, y0, y0 + ESP_W,
                                 board_z0, board_z0 + ESP_T, mat_abs)
    cut_holes_z(root, 'ESP32_DevKit', holes, 2.6, board_z0 - 1.0, ESP_T + 2.0, 'ESP32_Holes')

    def add_body(nm, a0, a1, b0, b1, z0, z1):
        sk = ecomp.sketches.add(ecomp.xYConstructionPlane)
        sk.name = nm + '_Sketch'
        sk.sketchCurves.sketchLines.addTwoPointRectangle(
            sketch_pt(sk, a0, b0, 0.0), sketch_pt(sk, a1, b1, 0.0))
        e = ecomp.features.extrudeFeatures.createInput(
            sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        e.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(MM(z0)))
        e.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(z1 - z0)))
        b = ecomp.features.extrudeFeatures.add(e).bodies.item(0)
        b.name = nm
        if mat_abs:
            b.material = mat_abs
        return b

    ml, mw, mh = ESP_MODULE
    add_body('ESP32_Module', x0, x0 + ml, cy - mw / 2.0, cy + mw / 2.0,
             board_z0 + ESP_T, board_z0 + ESP_T + mh)
    for tag, sy in (('A', -1), ('B', 1)):
        ry = cy + sy * ESP_HEADER_ROW / 2.0
        add_body('ESP32_Dupont_Keepout_' + tag,
                 x0 + ESP_HEADER_X0 - 1.27, x0 + ESP_HEADER_X0 + ESP_HEADER_SPAN - 1.27,
                 ry - 1.27, ry + 1.27, board_z0 - 16.5, board_z0)

    # ---- the printed carrier: plate + four posts --------------------------
    cocc, ccomp, carrier = make_box(printed.component, 'Holder_ESP32',
                                    x0 - margin, x0 + ESP_L + margin,
                                    y0 - margin, y0 + ESP_W + margin,
                                    base_top, base_top + CARRIER_T, mat_ny)
    sk = ccomp.sketches.add(ccomp.xYConstructionPlane)
    sk.name = 'Holder_ESP32_Posts'
    for (hx, hy) in holes:
        sk.sketchCurves.sketchCircles.addByCenterRadius(
            sketch_pt(sk, hx, hy, 0.0), MM(ESP_POST_OD / 2.0))
    col = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        col.add(sk.profiles.item(i))
    e = ccomp.features.extrudeFeatures.createInput(
        col, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    e.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(MM(base_top + CARRIER_T)))
    e.setDistanceExtent(False, adsk.core.ValueInput.createByReal(MM(ESP_UNDER)))
    e.participantBodies = [carrier]
    ccomp.features.extrudeFeatures.add(e)

    # lighten the plate between the header rows, clear of the two screws
    cut_rect_z(root, 'Holder_ESP32', cx - 9.0, cx + 9.0, y0 + 5.0, y0 + ESP_W - 5.0,
               base_top - 1.0, CARRIER_T + 2.0, 'Holder_ESP32_Window')
    cut_holes_z(root, 'Holder_ESP32', holes, M2_PILOT, board_z0, -8.0, 'Holder_ESP32_PostPilots')
    cut_holes_z(root, 'Holder_ESP32', carrier_screws, M3_CLR, base_top - 1.0,
                CARRIER_T + 2.0, 'Holder_ESP32_Screws')
    cut_holes_z(root, 'Base_Plate_Al', carrier_screws, M3_TAP, bz[0] - 2.0, 6.0,
                'Holder_ESP32_PlateHoles')

    out['board_z_mm'] = [round(board_z0, 2), round(board_z0 + ESP_T + mh, 2)]
    out['esp32_holes_mm'] = [[round(a, 2), round(b, 2)] for a, b in holes]
    out['carrier_screws_mm'] = [list(h) for h in carrier_screws]
    out['carrier_mass_g'] = round(find_body(root, 'Holder_ESP32').physicalProperties.mass * 1000.0, 2)
    out['status'] = 'BUILT'
    out['notes'] = [
        'ESP32 hole pattern 47.2 x 23.4 mm measured from a photo (+/-0.5 mm): '
        'measure the real board with calipers before printing the carrier.',
        'M2 screws down through the board into 1.6 mm pilots in the posts.',
        'Carrier: 2x M3 countersunk down into tapped holes in the base plate.',
        '18 mm under the board for straight DuPont leads on the down-facing headers.',
        'Power: UBEC 6 V into VIN - the board\'s own AMS1117 makes 3.3 V.',
    ]
    rep.step('22_esp32', out)
    rep.say('ESP32 DevKit placed on a printed carrier')
