"""Read CAD material properties and existing Simulation study inventory."""
import adsk.core
import adsk.fusion
import json


def property_value(prop):
    try:
        value = adsk.core.FloatProperty.cast(prop)
        if value:
            return value.value
    except Exception:
        pass
    return None


def run(_context):
    app = adsk.core.Application.get()
    if app.activeDocument.name != 'robot2':
        raise RuntimeError('robot2 must be active')
    design = adsk.fusion.Design.cast(app.activeProduct)
    wanted = {'Front_Wedge_L', 'Base_Plate_Al', 'Lifter_Pivot_Pin_6mm',
              'Lifter_Spatula_62mm', 'Top_Cover'}
    rows = []
    for occurrence in design.rootComponent.allOccurrences:
        for body in occurrence.bRepBodies:
            if body.name not in wanted:
                continue
            material = body.material
            properties = []
            for prop in material.materialProperties:
                properties.append({'name': prop.name, 'value': property_value(prop)})
            rows.append({'body': body.name, 'material': material.name,
                         'properties': properties})
    products = [p.productType for p in app.activeDocument.products]
    print(json.dumps({'parts': rows, 'products': products,
                      'document_saved': not app.activeDocument.isModified}))
