import adsk.core, adsk.fusion, json


def cm(mm):
    return mm/10.0


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design or app.activeDocument.name != 'robot2':
        raise RuntimeError('robot2 must be active')
    root = design.rootComponent
    cover_occ = [o for o in root.allOccurrences if o.component.name == 'Top_Cover']
    if len(cover_occ) != 1:
        raise RuntimeError('Expected one Top_Cover')
    comp = cover_occ[0].component
    body = comp.bRepBodies.itemByName('Top_Cover')
    if body is None:
        raise RuntimeError('Top_Cover body missing')
    if any(sk.name == 'FingerTech_Access_Patch' for sk in comp.sketches):
        raise RuntimeError('Switch access patch already exists')

    # Fill the obsolete 20x16 mm rectangular cut-out with a flush join.
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'FingerTech_Access_Patch'
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(cm(31), cm(-79), 0),
        adsk.core.Point3D.create(cm(53), cm(-61), 0))
    fill = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
    fill.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(cm(83)))
    fill.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(2)))
    fill.participantBodies = [body]
    comp.features.extrudeFeatures.add(fill)

    # One 6 mm access hole centred on the 2.5 mm hex screw of the switch.
    sk2 = comp.sketches.add(comp.xYConstructionPlane)
    sk2.name = 'FingerTech_Hex_Access_6mm'
    sk2.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(cm(42), cm(-70), 0), cm(3))
    cut = comp.features.extrudeFeatures.createInput(
        sk2.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    cut.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(cm(82)))
    cut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(4)))
    cut.participantBodies = [body]
    comp.features.extrudeFeatures.add(cut)
    print(json.dumps({'cover': 'Top_Cover', 'removed_access_mm': [20, 16],
                      'new_access_diameter_mm': 6, 'center_native_mm': [42, -70],
                      'note': 'label ON/OFF direction on physical cover; verify hex driver reach'}))
