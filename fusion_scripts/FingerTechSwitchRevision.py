import adsk.core, adsk.fusion, json


def cm(mm):
    return mm / 10.0


def rect_body(parent, name, x0, y0, x1, y1, z0, z1):
    occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = name + '_Outline'
    p0 = adsk.core.Point3D.create(cm(x0), cm(y0), 0)
    p1 = adsk.core.Point3D.create(cm(x1), cm(y1), 0)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(p0, p1)
    inp = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(cm(z0)))
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(z1-z0)))
    body = comp.features.extrudeFeatures.add(inp).bodies.item(0)
    body.name = name
    return occ, comp, body


def drill(comp, body, sketch_name, points, diameter, z0, length):
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = sketch_name
    for x, y in points:
        sk.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(cm(x), cm(y), 0), cm(diameter/2))
    profiles = adsk.core.ObjectCollection.create()
    for profile in sk.profiles:
        profiles.add(profile)
    cut = comp.features.extrudeFeatures.createInput(
        profiles, adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(cm(z0)))
    cut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(length)))
    cut.participantBodies = [body]
    comp.features.extrudeFeatures.add(cut)


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design or app.activeDocument.name != 'robot2':
        raise RuntimeError('Open the robot2 Fusion design first')
    root = design.rootComponent
    groups = {o.component.name: o.component for o in root.occurrences}
    if '05_Electronics' not in groups or '06_3D_Printed' not in groups:
        raise RuntimeError('Expected electronics and printed-parts groups were not found')
    electronics = groups['05_Electronics']
    printed = groups['06_3D_Printed']
    old = [o for o in electronics.occurrences if o.component.name == 'Main_Disconnect']
    if len(old) != 1:
        raise RuntimeError('Expected exactly one Main_Disconnect placeholder')
    if any(o.component.name == 'FingerTech_Mini_Switch_Envelope' for o in electronics.occurrences):
        raise RuntimeError('FingerTech switch envelope already exists')
    if any(o.component.name == 'FingerTech_Switch_Adapter' for o in printed.occurrences):
        raise RuntimeError('FingerTech adapter already exists')

    # Dimensions and 7.62 mm hole spacing from the FingerTech metric switch product page.
    # Preserve the old mount corridor around x=42, y=-70 and use the two existing posts.
    adapter, ac, ab = rect_body(printed, 'FingerTech_Switch_Adapter',
                                28.0, -79.0, 56.0, -61.0, 45.0, 49.0)
    drill(ac, ab, 'Adapter_Post_Clearance_M3',
          [(32.5, -70.0), (51.5, -70.0)], 3.4, 44.0, 6.0)
    drill(ac, ab, 'Adapter_Switch_Clearance_M2',
          [(38.19, -70.0), (45.81, -70.0)], 2.2, 44.0, 6.0)

    switch, sc, sb = rect_body(electronics, 'FingerTech_Mini_Switch_Envelope',
                               35.65, -76.35, 48.35, -63.65, 49.0, 55.35)
    drill(sc, sb, 'FingerTech_Mount_Holes_M2',
          [(38.19, -70.0), (45.81, -70.0)], 2.0, 48.0, 9.0)

    old[0].deleteMe()
    print(json.dumps({
        'document': app.activeDocument.name,
        'removed': 'Main_Disconnect placeholder',
        'added': ['FingerTech_Mini_Switch_Envelope', 'FingerTech_Switch_Adapter'],
        'switch_center_native_mm': [42, -70],
        'switch_body_mm': [12.7, 12.7, 6.35],
        'switch_mount_pitch_mm': 7.62,
        'adapter_size_mm': [28, 18, 4],
        'existing_post_centers_native_mm': [[32.5, -70], [51.5, -70]],
        'note': 'simplified housing; vendor terminal geometry and wiring clearance still need verification'
    }))
