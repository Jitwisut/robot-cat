"""Save revised live robot2 and export a reviewable Fusion archive."""
import adsk.core,adsk.fusion,json,os
PATH='/Users/jitwisutthobut/Desktop/robot/exports/robot2_items_1_3_2026-09-24.f3d'
def run(_):
 a=adsk.core.Application.get();doc=a.activeDocument;d=adsk.fusion.Design.cast(a.activeProduct)
 if doc.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent
 if len(list(r.occurrences))!=9:raise RuntimeError('unexpected root group count')
 for o in r.occurrences:
  if any(abs(o.transform2.getCell(i,j)-(1 if i==j else 0))>.001 for i in range(3) for j in range(4)):
   raise RuntimeError('non-grid-aligned group '+o.component.name)
 lift=next(o for o in r.occurrences if o.component.name=='07_Wedge_Lifter_Option').component
 j=next(q for q in lift.asBuiltJoints if q.name=='Lifter_Main_Revolute')
 if abs(j.jointMotion.rotationValue)>.001:raise RuntimeError('lifter not at rest')
 if not next(o for o in r.allOccurrences if o.component.name=='Top_Cover').isVisible:raise RuntimeError('cover hidden')
 if not next(o for o in r.occurrences if o.component.name=='09_3S_Drive_Upgrade').isVisible:raise RuntimeError('drive hidden')
 doc.save('Revise front load path, lifter joints, and 3S drivetrain with battery restraints')
 options=d.exportManager.createFusionArchiveExportOptions(PATH)
 if not d.exportManager.execute(options):raise RuntimeError('Fusion archive failed')
 print(json.dumps({'document':doc.name,'saved':not doc.isModified,'archive':PATH,'archive_bytes':os.path.getsize(PATH),
                   'root_groups':len(list(r.occurrences)),'lifter_joint_deg':j.jointMotion.rotationValue}))
