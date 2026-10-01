"""Aim Fusion's camera above and toward the front weapon; leave CAD transforms untouched."""
import adsk.core, adsk.fusion, json

def mmvec(p):
    return [round(p.x*10,1),round(p.y*10,1),round(p.z*10,1)]

def run(_context):
    app=adsk.core.Application.get()
    if app.activeDocument.name!='robot2':raise RuntimeError('robot2 must be active')
    vp=app.activeViewport
    cam=vp.camera
    cam.cameraType=adsk.core.CameraTypes.OrthographicCameraType
    # The assembly floor is XZ, +Y points up, and its weapon faces -Z.
    cam.eye=adsk.core.Point3D.create(20,50,-55)
    cam.target=adsk.core.Point3D.create(0,4,-2)
    cam.upVector=adsk.core.Vector3D.create(0,1,0)
    cam.isSmoothTransition=False
    vp.camera=cam
    vp.fit()
    vp.refresh()
    adsk.doEvents()
    c=vp.camera
    print(json.dumps({'eye_mm':mmvec(c.eye),'target_mm':mmvec(c.target),
                      'up':[round(c.upVector.x,3),round(c.upVector.y,3),round(c.upVector.z,3)],
                      'document_modified':app.activeDocument.isModified}))
