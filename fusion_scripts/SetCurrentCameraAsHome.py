"""Make the currently visible front/top camera the Fusion Home view."""
import adsk.core, adsk.fusion, json

def run(_context):
    app=adsk.core.Application.get()
    if app.activeDocument.name!='robot2':raise RuntimeError('robot2 must be active')
    vp=app.activeViewport
    cam=vp.camera
    if cam.eye.y<=cam.target.y or cam.eye.z>=cam.target.z:
        raise RuntimeError('Camera is not above and in front of the robot')
    ok=vp.setCurrentAsHome(True)
    if not ok:raise RuntimeError('Setting Home view failed')
    app.activeDocument.save('Set front/top isometric Home view for robot2')
    print(json.dumps({'home_view_set':ok,'saved':not app.activeDocument.isModified,
                      'eye_mm':[round(cam.eye.x*10,1),round(cam.eye.y*10,1),round(cam.eye.z*10,1)]}))
