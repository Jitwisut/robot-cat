import adsk.core, adsk.fusion, json

def coords(p):
    return [round(p.x*10,2),round(p.y*10,2),round(p.z*10,2)]

def run(_context):
    app=adsk.core.Application.get();d=adsk.fusion.Design.cast(app.activeProduct)
    r=d.rootComponent
    chassis=next(o for o in r.occurrences if o.component.name=='01_Chassis')
    body=next(b for o in chassis.childOccurrences for b in o.bRepBodies if b.name=='Base_Plate_Al')
    bb=body.boundingBox
    pts=[]
    for x in (bb.minPoint.x,bb.maxPoint.x):
        for y in (bb.minPoint.y,bb.maxPoint.y):
            for z in (bb.minPoint.z,bb.maxPoint.z):
                p=adsk.core.Point3D.create(x,y,z)
                p.transformBy(chassis.transform2)
                pts.append(p)
    world=[[round(min(getattr(p,a) for p in pts)*10,2) for a in 'xyz'],
           [round(max(getattr(p,a) for p in pts)*10,2) for a in 'xyz']]
    cam=app.activeViewport.camera
    print(json.dumps({'base_native_mm':[coords(bb.minPoint),coords(bb.maxPoint)],
                      'base_transformed_mm':world,
                      'camera_eye_mm':coords(cam.eye),
                      'camera_target_mm':coords(cam.target),
                      'camera_up':[cam.upVector.x,cam.upVector.y,cam.upVector.z]}))
