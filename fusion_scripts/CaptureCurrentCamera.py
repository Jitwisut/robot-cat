import adsk.core, json, os

PATH='/Users/jitwisutthobut/Desktop/robot/exports/views/robot2_lifter_camera_corrected.png'

def run(_context):
    app=adsk.core.Application.get()
    vp=app.activeViewport
    vp.refresh();adsk.doEvents()
    if not vp.saveAsImageFile(PATH,1600,1100):raise RuntimeError('Image capture failed')
    print(json.dumps({'path':PATH,'bytes':os.path.getsize(PATH)}))
