import adsk.core, adsk.fusion, json, os
def run(_):
    a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
    if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
    p='/Users/jitwisutthobut/Desktop/robot/exports/robot2_before_items_1_3.f3d'
    o=d.exportManager.createFusionArchiveExportOptions(p)
    if not d.exportManager.execute(o):raise RuntimeError('backup failed')
    print(json.dumps({'backup':p,'bytes':os.path.getsize(p)}))
