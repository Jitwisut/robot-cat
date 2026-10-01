import adsk.core, adsk.fusion, json


def bb(body):
    a = body.boundingBox
    return [[round(a.minPoint.x*10, 2), round(a.minPoint.y*10, 2), round(a.minPoint.z*10, 2)],
            [round(a.maxPoint.x*10, 2), round(a.maxPoint.y*10, 2), round(a.maxPoint.z*10, 2)]]


def run(_context):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    group = [o for o in root.occurrences if o.component.name == '03_Front_Spinner'][0]
    out = []
    for occ in group.component.occurrences:
        out.append({'name': occ.component.name,
                    'bodies': [{'name': body.name, 'bbox': bb(body),
                                'mass_g': round(body.physicalProperties.mass*1000, 2)}
                               for body in occ.bRepBodies]})
    print(json.dumps({'doc_modified': app.activeDocument.isModified, 'parts': out}))
