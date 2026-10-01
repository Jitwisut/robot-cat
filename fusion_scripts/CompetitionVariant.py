import adsk.core, adsk.fusion, json


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design or app.activeDocument.name != 'robot2':
        raise RuntimeError('robot2 must be active')
    root = design.rootComponent
    hammer = [occ for occ in root.occurrences if occ.component.name == '04_Top_Hammer']
    if len(hammer) != 1:
        raise RuntimeError('Expected exactly one top hammer group')
    bodies = [(body.name, round(body.physicalProperties.mass*1000, 3))
              for occ in hammer[0].childOccurrences
              for body in occ.bRepBodies]
    hammer[0].deleteMe()
    print(json.dumps({'removed_group': '04_Top_Hammer', 'removed_bodies': bodies,
                      'reason': 'competition variant: single front vertical spinner, lower profile'}))
