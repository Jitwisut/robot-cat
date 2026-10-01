"""Read only the engineering properties used for the impact screening."""
import adsk.core
import adsk.fusion
import json


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    wanted = {'Lifter_Pivot_Pin_6mm', 'Lifter_Spatula_62mm'}
    result = []
    for occurrence in design.rootComponent.allOccurrences:
        for body in occurrence.bRepBodies:
            if body.name not in wanted:
                continue
            props = []
            for prop in body.material.materialProperties:
                if prop.name not in {'Yield Strength', 'Young\'s Modulus',
                                     'Tensile Strength'}:
                    continue
                value = adsk.core.FloatProperty.cast(prop)
                if value:
                    props.append({'name': prop.name, 'value': value.value,
                                  'unit': value.units})
            result.append({'body': body.name, 'material': body.material.name,
                           'properties': props})
    print(json.dumps(result))
