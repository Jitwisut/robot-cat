import adsk.core,adsk.fusion,json,os
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 v=a.activeViewport;c=v.camera;c.isFitView=True;v.camera=c;v.refresh()
 path='/Users/jitwisutthobut/Desktop/robot/exports/views/robot2_items_1_3_live.png'
 if not v.saveAsImageFile(path,1800,1100):raise RuntimeError('view save failed')
 print(json.dumps({'image':path,'bytes':os.path.getsize(path)}))
