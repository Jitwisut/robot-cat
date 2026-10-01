"""Save the verified Z-up, grid-aligned robot2 and export a local backup."""
import adsk.core
import adsk.fusion
import json
import os


ARCHIVE = '/Users/jitwisutthobut/Desktop/robot/exports/robot2_grid_aligned.f3d'


def run(_context):
    app = adsk.core.Application.get()
    doc = app.activeDocument
    if doc.name != 'robot2':
        raise RuntimeError('robot2 must be active')
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    occurrences = list(root.occurrences)
    if len(occurrences) != 7:
        raise RuntimeError('Unexpected root group count')
    for occurrence in occurrences:
        transform = occurrence.transform2
        for i in range(3):
            for j in range(4):
                expected = 1 if i == j else 0
                if abs(transform.getCell(i, j) - expected) > 0.001:
                    raise RuntimeError('Root group is not flat: ' + occurrence.component.name)
    spinner = next(o for o in occurrences if o.component.name == '03_Front_Spinner')
    lifter = next(o for o in occurrences if o.component.name == '07_Wedge_Lifter_Option')
    grabber = next(o for o in occurrences if o.component.name == '08_Grabber_Addon_Option')
    if spinner.isVisible or not lifter.isVisible or grabber.isVisible:
        raise RuntimeError('Unexpected weapon option visibility')

    viewport = app.activeViewport
    camera = viewport.camera
    if camera.eye.z <= camera.target.z:
        raise RuntimeError('Expected camera above the robot')
    if not viewport.setCurrentAsHome(True):
        raise RuntimeError('Could not set the flat view as Home')
    doc.save('Orient robot2 flat on XY grid and set Z-up Top/Home view')
    os.makedirs(os.path.dirname(ARCHIVE), exist_ok=True)
    options = design.exportManager.createFusionArchiveExportOptions(ARCHIVE)
    if not design.exportManager.execute(options):
        raise RuntimeError('F3D backup export failed')
    print(json.dumps({
        'saved': not doc.isModified,
        'archive': ARCHIVE,
        'archive_bytes': os.path.getsize(ARCHIVE),
        'root_pose': 'identity',
        'lifter_visible': lifter.isVisible,
        'spinner_visible': spinner.isVisible,
        'grabber_visible': grabber.isVisible,
    }))
