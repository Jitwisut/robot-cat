import adsk.core, json, os

PATH = '/Users/jitwisutthobut/Desktop/robot/robot_v4/V4_View.png'


def run(_context):
    app = adsk.core.Application.get()
    vp = app.activeViewport
    cam = vp.camera
    # Fusion is Y-up; the nose points to Fusion -Z. View from front-right-above.
    cam.target = adsk.core.Point3D.create(0, 3, -1)
    cam.eye = adsk.core.Point3D.create(30, 22, -40)
    cam.upVector = adsk.core.Vector3D.create(0, 1, 0)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh(); adsk.doEvents()
    if not vp.saveAsImageFile(PATH, 1600, 1100):
        raise RuntimeError('Image capture failed')
    print(json.dumps({'path': PATH, 'bytes': os.path.getsize(PATH)}))
