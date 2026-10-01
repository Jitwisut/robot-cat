import adsk.core,adsk.fusion,json,os
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent;cover=next(o for o in r.allOccurrences if o.component.name=='Top_Cover')
 old_visible=cover.isLightBulbOn;v=a.activeViewport;old=v.camera
 path='/Users/jitwisutthobut/Desktop/robot/exports/views/robot2_items_1_3_inside.png'
 try:
  cover.isLightBulbOn=False
  c=v.camera;c.eye=adsk.core.Point3D.create(20,25,22);c.target=adsk.core.Point3D.create(0,0,4)
  c.upVector=adsk.core.Vector3D.create(0,0,1);c.isFitView=True;v.camera=c;v.refresh()
  if not v.saveAsImageFile(path,1800,1100):raise RuntimeError('save failed')
 finally:
  cover.isLightBulbOn=old_visible;v.camera=old;v.refresh()
 print(json.dumps({'image':path,'bytes':os.path.getsize(path),'cover_restored':cover.isVisible}))
