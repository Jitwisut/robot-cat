import adsk.core, adsk.fusion, json

def run(_context):
    app=adsk.core.Application.get();d=adsk.fusion.Design.cast(app.activeProduct)
    r=d.rootComponent
    spin=next(o for o in r.occurrences if o.component.name=='03_Front_Spinner')
    lift=next(o for o in r.occurrences if o.component.name=='07_Wedge_Lifter_Option')
    spin.isLightBulbOn=True;lift.isLightBulbOn=False
    for o in r.allOccurrences:
        if o.component.name=='Holder_ESC_Tekko32':o.isLightBulbOn=True
        if o.component.name.endswith('Sweep_Envelope'):o.isLightBulbOn=False
    print(json.dumps({'spinner_visible':spin.isVisible,'lifter_visible':lift.isVisible,
                      'spinner_body_count':sum(1 for o in spin.childOccurrences if o.component.name=='Spinner_Bar')}))
