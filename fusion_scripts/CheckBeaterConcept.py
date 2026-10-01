import adsk.core, adsk.fusion, json


def bb(body):
    a = body.boundingBox
    return [[round(a.minPoint.x*10, 2), round(a.minPoint.y*10, 2), round(a.minPoint.z*10, 2)],
            [round(a.maxPoint.x*10, 2), round(a.maxPoint.y*10, 2), round(a.maxPoint.z*10, 2)]]


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    bodies = []
    parts = []
    for occ in root.allOccurrences:
        for body in occ.bRepBodies:
            bodies.append(body)
            if body.name in ('Beater_Rotor_Concept', 'Beater_Sweep_Envelope',
                             'Spinner_Shaft', 'Weapon_Pulley', 'Left_Support',
                             'Right_Support', 'Base_Plate_Al', 'Top_Cover'):
                parts.append({'name': body.name, 'bbox_world_mm': bb(body),
                              'mass_cad_g': round(body.physicalProperties.mass*1000, 3)})
    coll = adsk.core.ObjectCollection.create()
    for body in bodies:
        coll.add(body)
    inputs = design.createInterferenceInput(coll)
    inputs.areCoincidentFacesIncluded = False
    result = design.analyzeInterference(inputs)
    tbm = adsk.fusion.TemporaryBRepManager.get()
    pairs = []
    for i in range(result.count):
        item = result.item(i)
        volume_mm3 = None
        try:
            a = tbm.copy(item.entityOne)
            b = tbm.copy(item.entityTwo)
            tbm.booleanOperation(a, b, adsk.fusion.BooleanTypes.IntersectionBooleanType)
            volume_mm3 = round(a.volume*1000, 3)
        except Exception:
            pass
        pairs.append({'a': item.entityOne.name, 'b': item.entityTwo.name,
                      'volume_mm3': volume_mm3})
    analysis_names = {'Spinner_Sweep_Envelope', 'Beater_Sweep_Envelope',
                      'PCB_Keepout_Envelope', 'ESP32_Dupont_Keepout_A',
                      'ESP32_Dupont_Keepout_B'}
    physical_mass = sum(b.physicalProperties.mass*1000 for b in bodies
                        if b.name not in analysis_names)
    envelope_mass = sum(b.physicalProperties.mass*1000 for b in bodies
                        if b.name in analysis_names)
    print(json.dumps({'document': app.activeDocument.name,
                      'modified': app.activeDocument.isModified,
                      'body_count': len(bodies), 'parts': parts,
                      'physical_cad_mass_g': round(physical_mass, 3),
                      'analysis_envelope_mass_g': round(envelope_mass, 3),
                      'interference': pairs}))
