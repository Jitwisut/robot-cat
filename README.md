# robot-cat — ESP32 Mini BattleBot (V2 robot2 / V3 R3)

CAD, Fusion 360 scripts and reduced-order simulations for a ~1.1–1.5 kg mini battlebot.
Project status (Thai): [`STATUS.md`](STATUS.md). Nothing here is cleared for manufacture yet; no 3D FEA has been solved.

| Folder | Contents |
|---|---|
| `exports/` | V2 (`robot2`) CAD exports, reports, impact screening, lifter linkage analysis (`v2_linkage_2026-09-30/`) |
| `robot_v3/` | V3 study: R1–R3 CAD (F3D/STEP), build scripts, reports, robot2 vs R3 duel model (`duel_sim/`) |
| `robot_v4/` | V4 vertical beater: CAD, sizing (`design_v4.py`), matchup screen, ESP32 firmware, status (`robot_v4/STATUS.md`), reduced-order three-way sim (`three_way_sim/`), MuJoCo 3D duel sim (`mujoco_sim/`, needs `python3 -m venv .venv && .venv/bin/pip install mujoco numpy "imageio[ffmpeg]"`) |
| `fusion_scripts/` | Fusion 360 Python scripts that built/edited `robot2`; `V2_Lifter_Stops/` adds the lifter hard stops |

## Latest CAD

| Robot | File |
|---|---|
| V2 with lifter stops (recommended) | `exports/robot2_lifter_stops_2026-10-01.f3d` / `.step` |
| V2 as of items 1–3 | `exports/robot2_items_1_3_2026-09-24.f3d`, `exports/robot2_items_1_3_active_lifter_2026-10-01.step` |
| V3 R3 lifting wedge | `robot_v3/ROBOT_V3_R3_Lifter.f3d` / `.step` |
| V4 vertical beater (current direction) | `robot_v4/ROBOT_V4_Beater.f3d` / `.step` |

Axes differ: **V2 is Z-up, nose +Y**; **V3 files are Y-up, nose −Z**. STEP units are mm.
Other `robot2_*.f3d` files in `exports/` are earlier checkpoints; `exports/STL`, `exports/DXF` and `ESP32_Mini_BattleBot_*.step` predate the lifter.

## Re-run the models

```bash
pip install numpy matplotlib
python3 robot_v3/duel_sim/duel_sim.py
python3 exports/v2_linkage_2026-09-30/v2_linkage.py
```

Fusion scripts run from Utilities → Scripts and Add-Ins. Several older scripts write to absolute macOS paths and check for a document named `robot2`.
