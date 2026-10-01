"""Give the revised beater a visible red appearance; leave its material intact."""

import adsk.core
import adsk.fusion
import json


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design or app.activeDocument.name != 'robot2':
        raise RuntimeError('Open robot2 first')
    matches = [o for o in design.rootComponent.allOccurrences
               if o.component.name == 'Beater_Rotor_Concept']
    if len(matches) != 1:
        raise RuntimeError('Expected one beater rotor in robot2')
    rotor = matches[0]
    body = rotor.bRepBodies.itemByName('Beater_Rotor_Concept')
    if not body:
        raise RuntimeError('Beater rotor body missing')
    material_before = body.material.name
    mass_before = body.physicalProperties.mass * 1000

    appearance = design.appearances.itemByName('Beater_Red_Visual_Only')
    if appearance is None:
        source = None
        for library in app.materialLibraries:
            if library.name == 'Fusion Appearance Library':
                source = library.appearances.itemByName('Aluminum - Anodized Glossy (Red)')
                break
        if source is None:
            raise RuntimeError('Red aluminum appearance not available')
        appearance = design.appearances.addByCopy(source, 'Beater_Red_Visual_Only')
    rotor.appearance = appearance
    if body.material.name != material_before:
        raise RuntimeError('Physical material unexpectedly changed')
    mass_after = body.physicalProperties.mass * 1000
    if abs(mass_after - mass_before) > 0.001:
        raise RuntimeError('Physical mass unexpectedly changed')
    app.activeDocument.save('Highlight beater rotor so design change is visible')
    print(json.dumps({'document': app.activeDocument.name,
                      'rotor': rotor.component.name,
                      'appearance': rotor.appearance.name,
                      'material': material_before,
                      'mass_cad_g': round(mass_after, 3),
                      'saved': not app.activeDocument.isModified}))
