import adsk.core, adsk.fusion, json


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    out = {'document': app.activeDocument.name, 'modified': app.activeDocument.isModified,
           'appearance_candidates': [], 'parts': []}
    for lib in app.materialLibraries:
        for appearance in lib.appearances:
            name = appearance.name
            if any(token in name.lower() for token in ('red', 'orange', 'yellow', 'blue')):
                out['appearance_candidates'].append({'library': lib.name, 'name': name})
            if len(out['appearance_candidates']) >= 45:
                break
        if len(out['appearance_candidates']) >= 45:
            break
    for occ in root.allOccurrences:
        if occ.component.name in ('Beater_Rotor_Concept', 'Top_Cover',
                                  'Front_Wedge_L', 'Front_Wedge_R', 'Beater_Sweep_Envelope'):
            out['parts'].append({'name': occ.component.name,
                                 'light_on': occ.isLightBulbOn,
                                 'visible': occ.isVisible,
                                 'appearance': occ.appearance.name if occ.appearance else None})
    print(json.dumps(out))
