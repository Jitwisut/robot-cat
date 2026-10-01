import adsk.core, adsk.fusion, json

def run(_context):
    r=adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
    groups={o.component.name:o for o in r.occurrences}
    groups['03_Front_Spinner'].isLightBulbOn=False
    groups['07_Wedge_Lifter_Option'].isLightBulbOn=True
    groups['08_Grabber_Addon_Option'].isLightBulbOn=True
    for o in r.allOccurrences:
        if o.component.name=='Holder_ESC_Tekko32':o.isLightBulbOn=False
        if o.component.name.endswith('Sweep_Envelope'):o.isLightBulbOn=False
    print(json.dumps({k:groups[k].isVisible for k in ('03_Front_Spinner','07_Wedge_Lifter_Option','08_Grabber_Addon_Option')}))
