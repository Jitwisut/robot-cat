import adsk.core,adsk.fusion,json
def run(_):
 a=adsk.core.Application.get();d=adsk.fusion.Design.cast(a.activeProduct)
 if a.activeDocument.name!='robot2':raise RuntimeError('robot2 required')
 r=d.rootComponent
 out={}
 for n in ['ESP32_DevKit','Holder_ESP32']:
  o=next(q for q in r.allOccurrences if q.component.name==n);o.isLightBulbOn=False;out[n]=o.isVisible
 print(json.dumps({'legacy_controller_suppressed':out,'active_control':'ER6 PWM direct to dual brushed ESC and lifter servo'}))
