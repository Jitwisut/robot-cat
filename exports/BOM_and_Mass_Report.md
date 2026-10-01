# ESP32 Mini BattleBot — BOM & Mass Report

> **HISTORICAL — do not use for the competition variant.** Read `../STATUS.md`,
> `Mass_Report_Generated.md`, and `Design_Review_Competition_2026-09-23.md`.
> This table predates the switch replacement and removal of the top hammer.
> `BattleBotFixes.py` is a legacy script and must not be run on the current robot2 design.
>
> Two known errors in the figures below are described in `../STATUS.md`:
> the front wedges are modelled as **45 mm solid aluminium** (413 g for the pair,
> almost certainly unintended), and the two analysis envelopes carry ~91 g of
> phantom mass because they were assigned ABS.

Generated from Fusion 360 model `robot` (BattleBot_Master), 2026-09-22.

## Bill of Materials (real parts, Thai retail)

| Subsystem | Part | Spec | Real mass | Price (THB) | Source |
|---|---|---|---:|---:|---|
| Drivetrain | TT Motor 200RPM ×2 | yellow gearmotor | 20g ea. | ~60-90 | user-specified |
| Drivetrain | Wheel 42mm ×2 | TT wheel | 8g ea. | — | user-specified |
| Front Spinner | A2212 1400KV brushless motor | Ø27.5×30mm | 55g | ~155 | SriTu Hobby |
| Front Spinner | 30A BLHeli ESC | ~45×26×8mm | 25g | ~150-400 | generic |
| Top Hammer | DS3218MG 20kg digital servo | 40.5×20×40.5mm | 60g | ~440-690 | SriTu Hobby / ThaiEasyElec |
| Electronics | 3S 2200mAh LiPo | 104×35×27mm | 200g | ~450-700 | FPVONLY (GNB/Gens-Ace class) |
| Electronics | ESP32-C3 SuperMini + control PCB (assembled) | 60×50mm footprint | ~30g | — | plan.md baseline |
| Electronics | RC receiver | — | 8g | — | generic |
| Electronics | Motor driver board | — | 10g | — | generic |
| Electronics | Buck/BEC | — | 8g | — | generic |
| Electronics | Main disconnect switch | — | 10g | — | generic |
| Electronics | Spinner shaft bearings ×2 | 623/624-class | 5g | — | generic |

**All items well within the 1,500-3,000 THB/item budget** — actual unit costs are much lower, leaving headroom to upgrade (e.g. higher-KV motor, premium servo) if desired.

## Mass Report

### CAD-computed mass (fabricated/structural parts — trustworthy, these are being designed not bought)

| Part | Material | Mass |
|---|---|---:|
| Base_Plate_Al | Aluminum 6061 | 155.5 g |
| Left_Side_Plate_Al | Aluminum 6061 | 35.9 g |
| Right_Side_Plate_Al | Aluminum 6061 | 35.9 g |
| Front_Wedge_L | Aluminum 6061 | 206.6 g |
| Front_Wedge_R | Aluminum 6061 | 206.6 g |
| Top_Cover | Nylon 6 | 64.5 g |
| Left/Right_Support (spinner bearing blocks) | Aluminum 6061 | 3.6 g ea. |
| Spinner_Shaft | Steel | 4.0 g |
| Spinner_Bar (weapon) | ~~Aluminum 6061~~ → **Aluminum 7075-T6** (applied by the script, ≈22 g) | 21.1 g |
| Hammer_Arm | Aluminum 6061 | 32.4 g |
| Hammer_Head | Aluminum 6061 | 24.3 g |
| PCB_Standoff ×4 | Nylon 6 | 0.1 g |
| Battery_Holder | Nylon 6 | 8.2 g |
| **Structural subtotal** | | **≈ 802 g** |

*(Excludes `Spinner_Sweep_Envelope` and `PCB_Keepout_Envelope` — these are reference/analysis volumes, not real parts, and are not counted toward mass.)*

### Corrected total (structural CAD mass + real BOM masses, not the inaccurate solid-envelope mass of purchased parts)

| Category | Mass |
|---|---:|
| Structural/fabricated (CAD, aluminum+nylon+steel) | 802 g |
| Purchased components (real datasheet masses, see BOM) | 477 g |
| Fasteners & wiring allowance | 100 g |
| Design-change margin | 90 g |
| **Total estimated mass** | **≈ 1,469 g** |

**Target: 1,500 g — currently ≈31g under budget.** Center of mass: x=0 (centered), y=+11mm (slightly forward), z=33mm above ground (low and stable).

## Known simplifications / follow-ups
- Spinner drive is belt/pulley in concept — pulleys and belt not modeled, only
  clearance-checked. **The drive ratio is therefore still undefined, so the
  ~10 J weapon-energy figure (which assumes 1:1) is not yet a usable number.**
- Bearing/shaft mismatch: `spinner_bearing_OD` 13 mm is a 624 (4 mm bore) but
  `spinner_shaft_D` is 3.17 mm (1/8"). Pick 623 + 3 mm shaft, R2-5 + 1/8" shaft,
  or step the shaft up to 4 mm and keep the 624.
- PCB standoffs pass through the PCB body (no drilled through-hole modeled) — cosmetic only.
- Motor/servo mounting bolt patterns not modeled — confirm against actual supplier datasheet before drilling.
- Spinner motor now has a saddle cradle (`Spinner_Motor_Cradle`) bolting it to the
  base plate with 4× M3 — pending the script run.
- Wire_Clips, Non_Structural_Covers, TPU_Bumpers left as empty placeholder components — need real cable routing / impact-zone info.
