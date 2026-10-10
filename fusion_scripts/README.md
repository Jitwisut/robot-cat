# Fusion scripts for `robot2`

## Robot v4 R3 viewer orientation

`OrientV4PrintR3.py` sets the active `ROBOT_V4_PRINT_R3` document's Top to +Z and Home to an oblique view above the XY floor. It does not rotate or translate components and does not change global Fusion preferences. It exports a local F3D without saving to the cloud. Use `robot_v4/print_revision/r3/output_R3/DRAFT/ROBOT_V4_PRINT_R3_FUSION_Z_UP.f3d` to reopen the corrected view; a STEP import may inherit Fusion's Y-up document orientation.

Checked on 2026-10-10: all 70 occurrence transforms and assembly bounds were unchanged. Reimporting the F3D retained camera up `(0,0,1)` and showed the grid beneath the wheels. The images `ROBOT_V4_PRINT_R3_FUSION_Z_UP.png` and `ROBOT_V4_PRINT_R3_FUSION_REOPENED.png` record both checks. This fixes the viewing orientation only; R3 retains its DRAFT manufacturing status.

The current `robot2` document is saved in Fusion with the wedge lifter visible. Its seven root occurrences use identity transforms, and the document's ViewCube Top is +Z. This puts the base on Fusion's XY layout grid and makes the front/top Home view show the robot lying flat. The backup is `../exports/robot2_grid_aligned.f3d`; the checked image is `../exports/views/robot2_flat_preview.png`. The original vertical spinner and optional grabber are available as modular groups; see `../exports/Modular_Weapons_Design_2026-09-24.md`. The beater concept is archived at `../exports/robot2_beater_concept_v1.f3d`.

- RestoreOriginalVerticalSpinner.py — one-time replacement of the beater with the original 6 mm axial bar and rotation envelope.
- AddWedgeLifterOption.py — one-time creation of the 62 mm spatula, 6 mm pivot and actuator envelopes.
- AddGrabberOption.py, CutGrabberPivotBores.py — one-time optional upper jaw and pivot bore creation.
- ShowVerticalSpinnerOption.py, ShowWedgeLifterOption.py, ShowGrabberOption.py — set alternative group visibility; check the ESC holder when switching.
- `OrientRobotFlat.py`, `SaveRobotFlat.py` — one-time correction and save of the current XY-grid pose. Do not rerun them on the saved document.
- `OrientRobotToGrid.py`, `DiagnoseAlignment.py` — historical Y-up/XZ-grid trials. **Do not run on the current document**; they rotate the robot upright in the corrected Z-up view.
- CheckWedgeLifterOption.py, CheckGrabberOption.py — read-only stationary interference checks; envelopes and hidden spinner hardware are excluded as applicable.
- SaveOriginalSpinnerOption.py, SaveWedgeLifterOption.py, SaveGrabberOption.py — save robot2 and export the three F3D alternatives.

- `FingerTechSwitchRevision.py` — one-time replacement of the old switch mock; expects the old `Main_Disconnect` and posts.
- `CompetitionVariant.py` — one-time removal of the `04_Top_Hammer` group.
- `FingerTechAccessRevision.py` — one-time replacement of the 20×16 mm top-cover cutout with a Ø6 mm access hole.
- `AlignCompetitionVariant.py` — historical Y-up/XZ-grid alignment. **Do not run on the current Z-up document**.
- `generate_competition_reports.py` — reads the exported JSON snapshot and regenerates current dimensions and mass reports; it does not modify Fusion.
- `BeaterConceptRevision.py` — one-time 34 mm wide beater concept and 34 mm sweep envelope; replaces the old bar. It is guarded against rerunning.
- `CheckBeaterConcept.py` — read-only bounding boxes, mass and interference check for the beater concept.
- `SaveBeaterConcept.py` — saves `robot2` and exports the F3D archive.
- `HighlightBeaterVisual.py` — gives the beater a red appearance while retaining Aluminum 7075 mass.
- `CleanBeaterView.py`, `RestoreBeaterAlignment.py` — historical beater-view helpers; the alignment helper is for the obsolete Y-up pose.
- `ExportBeaterRotorSTEP.py` — exports only the beater rotor concept as STEP.
- `mcp_local.py`, `export_beater_check.py`, `capture_fusion_view.py` — local helpers for the Fusion MCP server on port 27182.
- `BattleBotFixes.py` — historical script for the old assembly. It refuses to run on the current competition variant.

The geometry creation and migration scripts are one-time operations. Do not rerun them on the current document. Verify geometry, interference, group transforms, visibility and saved state before replacing exports.
