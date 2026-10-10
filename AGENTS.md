# Robot workspace instructions

## Approved Robot v4 orientation — user requirement, 2026-10-10

Keep Robot v4 in the approved flat orientation at all times:

- World +Z is up. Ground is the XY plane at Z=0, beneath the wheels.
- The robot nose points toward +Y; X is the lateral axis. CAD dimensions are millimetres.
- Do not rotate, translate, mirror, swap axes, or change the assembly coordinate convention to accommodate a viewer's defaults.
- Do not change this orientation or axis convention unless the user explicitly requests a change.
- In Fusion, set the document's Top to +Z and its Home to the approved oblique view above the robot. Correct the viewer when it imports with Y-up; preserve every component transform and the CAD geometry.
- Preserve existing print-file orientations. Do not silently reorient or regenerate manufacturing files as part of a viewer fix.
- Verify the floor grid is beneath the wheels, the robot lies flat, and component transforms and assembly bounds are unchanged. When exporting F3D, reopen it to verify the orientation persists.

Approved Fusion reference:
`robot_v4/print_revision/r3/output_R3/DRAFT/ROBOT_V4_PRINT_R3_FUSION_Z_UP.f3d`

Verified reopened image:
`robot_v4/print_revision/r3/output_R3/DRAFT/ROBOT_V4_PRINT_R3_FUSION_REOPENED.png`

The helper `fusion_scripts/OrientV4PrintR3.py` corrects Top/Home without moving the 70 R3 components. This viewing correction does not change R3's DRAFT manufacturing status. Preserve historical V2/V3 revisions in their original coordinate systems; this requirement governs the current V4 and subsequent V4 revisions.
