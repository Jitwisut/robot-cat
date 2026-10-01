# Fastener Design, FDM Print Guide & Spinner Structural Review Checklist

> **ประวัติการแก้ก่อนรุ่นแข่ง:** บางข้อด้านล่าง เช่น ปีกหน้า “ยังไม่ยึด”
> และชุดค้อนบน ไม่ตรงกับ `robot2` ล่าสุด. ใช้ `../STATUS.md` และ
> `Design_Review_Competition_2026-09-23.md` เป็นรายการงานปัจจุบัน;
> ยืนยันรูยึดจากไฟล์ CAD/DXF และอะไหล่จริงก่อนผลิต.

## 1. Chassis fastening system (base–wall–cover)

Added a real, buildable fastening scheme: **4× M3 corner standoffs** (`Standoffs_Fasteners`
component, Ø6mm OD, Ø3.2mm bore) running the full interior height, at the 4 corners where the
side walls meet the base/wedge boundary.

- Base plate: 4× M3.4 clearance holes (vertical) — screw from below into standoff bottom.
- Top cover: 4× M3.4 clearance holes (vertical) — screw from above into standoff top.
- Side walls: 2× M3.4 clearance holes each (horizontal, at standoff mid-height) — screw through
  the wall into the standoff's side (self-tapping into the aluminum standoff, or tap M3 by hand
  before assembly).

**Assembly order:** base plate → standoffs (bolt from below) → side walls (screw horizontally
into standoffs) → wedges (currently butt against the wall, not separately fastened — see
open item below) → top cover (bolt from above).

**Open item:** Front_Wedge_L/R are not yet mechanically fastened to the base plate or the
standoffs — they currently rely on the wheel-well/wedge-trim geometry fitting flush. Add 1-2
screws per wedge into the base plate before relying on this structurally.

## 2. Real bolt patterns added (from datasheets)

| Part | Pattern | Source |
|---|---|---|
| TT motor gearbox (×2) | 2× M3 holes, 17.5mm apart | Handson Technology FAM1062 datasheet — **but see the red flag below: these holes do not exist in the model, and could not reach the gearbox even if they did** |
| A2212 spinner motor | 4× M3 holes, 16×19mm rectangle | Widely documented A2212 standard mount |
| DS3218MG hammer servo | 2× M3 holes, 49.5mm apart, cut into Top_Cover | Standard servo flange spacing (servodatabase.com) |

**Closed:** flange tabs were subsequently added to `Hammer_Servo` matching the 49.5 mm spacing
already cut in the top cover.

### TT drive motor mounts (`TT_Motor_Mount_L/R` + `TT_Motor_Clamp_L/R`)

An earlier pass recorded "matching TT motor mount holes through the side walls". Counting the
cylindrical faces in the real solids showed each side wall had **exactly two holes, both Ø3.4
along X at z = 44 mm** — the corner-standoff screws. There were no TT motor holes, and the
drive motors were unconstrained.

Cutting them would not have helped: `TT_Gearbox` spans x = −61…−37 mm while the inner face of
the side wall is at x = −78 mm — a **17 mm gap** — and that region of the wall is already
removed by the wheel-well cutout (which eats the wall from z = 5 up to z = 47 mm).

**The gearbox's own holes were deliberately not used.** It does carry two Ø3.4 holes, but they
run lengthwise along X at y = ±8.75, z = 21 — the manufacturer's bracket interface — and both
ends of that screw line are blocked, by the wheel outboard and the motor can inboard. A bracket
in the gap between gearbox and can would only fit because this model draws them as two separate
boxes; the real motor has no such gap.

So the gearbox body is clamped instead:

| Item | Value |
|---|---|
| `TT_Motor_Mount_L/R` | U-cradle rising from the base plate, gearbox seats into it |
| Envelope | x ±34.7…±55, y ±16.5, z 5…30 mm |
| Seat | ±11.5 mm half width (0.5 mm clearance per side), 7 mm floor, 5 mm cheeks |
| To base plate | 4× M3 per side at (±51, ±14) and (±38.7, ±14), tapped blind in the floor |
| `TT_Motor_Clamp_L/R` | 10 × 33 × 3 mm bar, 2× M3 down into tapped holes in the cheek tops |
| Preload gap | cheeks stop at z = 28.5, gearbox top is z = 30 — a deliberate 1.5 mm |
| Mass | 21.23 g mount + 2.53 g clamp per side (47.5 g for both) |
| Material | Aluminium 6061 |

Every bolt position was checked with point containment against the base plate solid before
cutting — the plate only has material from x = ±56 inboard, the rest is wheel-well cutout.

**The 1.5 mm cheek drop is load-bearing, not cosmetic.** The first build made the cheek tops
flush with the gearbox top at z = 30. That looks right and is useless: tightening the M3s
bottoms the clamp bar on the cheeks, which become a hard stop, and the gearbox sees **zero
clamping force** — it is merely covered. Dropping the cheeks 1.5 mm makes the bar contact the
gearbox first, so the screws generate real preload and the 3 mm aluminium bar acts as the
spring. If you ever re-cut these parts, keep the cheeks below the gearbox top.

**Optional, recommended:** a thin rubber or TPU pad between the clamp bar and the gearbox, for
friction and to avoid marking the plastic housing. Not structurally required now that the bar
actually preloads.

### Spinner motor mount (`Spinner_Motor_Cradle`)

Three earlier attempts at an A2212 bracket were all placed *behind* the motor and all collided
with `Spinner_ESC` / `Main_Disconnect`, so the bracket was removed and left as an open item.
The corridor behind the motor (y < 12 mm) is genuinely full; the volume *underneath* the can is
not. `BattleBotFixes` builds a saddle cradle there instead:

| Item | Value |
|---|---|
| Envelope | x −15…15, y 12…41, z 5…30.5 mm (as built) |
| Saddle radius | 14.15 mm (can 13.75 + 0.4 mm fit) |
| Wrap | ±55.6° — stops 8 mm below the axis so the legs keep real thickness |
| Thinnest leg | 2.08 mm rear / 3.58 mm front |
| Mass | 51.3 g, Aluminium 6061 |
| Fastening | 4× M3 from below, through the base plate into tapped holes in the cradle |
| Material | Aluminium 6061 |

The script re-probes that volume against every body at run time and abandons the build (rather
than forcing it) if anything is in the way. That probe earned its keep: the first footprint put
the rear wall at y = 9 mm and the probe caught `TT_Motor_Can_L/R` reaching back to y = 10 mm.
The rear wall was moved flush to the can at y = 12 mm, leaving a 2 mm gap to the drive motors —
**re-check that gap if the drive motors or ESC ever move.** Belt clearance is fine: the drive line sits at
z ≈ 38.5 mm, 6 mm above the cradle's top face.

## 3. Battery_Holder / PCB_Standoff — now real parts

- **Battery_Holder**: floor + 5mm side lips along both long edges (prevents lateral sliding) +
  2 strap slots (10×4mm) near each end for a hook-and-loop strap. Not yet screwed down to
  anything below — add 2-4 mounting holes + matching posts if you want it permanently fixed
  rather than held by the strap alone.
- **PCB_Standoff ×4**: raised to a real 3mm standoff height (off the floor, standard practice),
  each with a Ø2.9mm through-bore sized for M2.5 self-tapping into FDM plastic. Matching
  Ø2.9mm clearance holes added through the base plate so a screw can run from underneath.

## 4. FDM print tolerances used

| Parameter | Value | Reason |
|---|---:|---|
| `m3_clear_d` | 3.4mm | M3 clearance in **CNC/laser-cut aluminum** (tight, no print shrinkage) |
| `m3_clear_d_fdm` | 3.6mm | M3 clearance for **FDM-printed** parts (+0.2mm for typical FDM oversizing/shrinkage) |
| `m2_5_clear_d_fdm` | 2.9mm | M2.5 self-tap pilot in FDM plastic |
| `print_wall_t` | 2mm | Minimum wall thickness for PETG/ABS at this scale — don't go thinner than ~1.2mm (3-4 perimeters at 0.4mm nozzle) |
| `assembly_clearance` | 1mm | General fit clearance already used throughout (wheel wells, wedge trims, etc.) |

**Print orientation recommendations** (not yet modeled, apply when slicing):
- `Battery_Holder`: print flat, floor-down — lips print vertically with no support needed.
- `PCB_Standoff`: print vertically (bore axis up) for the strongest through-bore; a raft/brim
  helps at this small a footprint.
- `Top_Cover` (if printed instead of CNC'd): print flat; the M3 holes should be printed
  slightly undersized and drilled to final size for a clean thread, since horizontal round
  holes print rough on FDM.

## 5. Spinner bar (`Spinner_Bar`) — structural review checklist

**I cannot replace a qualified human reviewer for this — a spinning weapon is a genuine safety
hazard.** Before cutting or printing this part, have someone experienced in combat robotics
check:

- [x] **Material**: moved off the 6061 mock to **Aluminum 7075-T6** (see
      `Sensor_and_Blade_Safety_Review.md` for the energy comparison against steel). A human
      still needs to confirm 7075 is right for your event rather than a hardened striking insert.
- [ ] **Kinetic energy vs containment**: a first-pass figure is now calculated (≈10 J at 1:1,
      A2212 1400KV on 3S) but **the belt ratio has not been chosen**, and energy scales with the
      square of it — so the real number is still unknown. Settle the pulleys, then check tip
      speed and stored energy against your event's weapon-energy limit and required containment
      rating before spinning it for the first time, at any speed.
- [ ] **Balance**: `Spinner_Bar` (65×20×6mm) must be dynamically balanced around the shaft axis —
      any asymmetry at high RPM causes severe vibration and can destroy the bearings/shaft.
- [x] **Fixed 2026-09-23: shaft is now 4 mm with a 624 bearing.** Was: these two parameters contradicted each other. A 13 mm OD bearing
      is a 624, which has a **4 mm** bore; a 3.17 mm (1/8") shaft will not fit it. Resolve to one
      of: 623 (3×10×4) on a 3 mm shaft, R2-5 (1/8"×3/8") on the existing 1/8" shaft, or step the
      shaft to 4 mm and keep the 624 — the last is preferred, since a thicker shaft survives
      strike loads better. Then confirm it against bending/shock, not just steady rotation.
- [ ] **Fastening the bar to the shaft**: not modeled at all yet (set-screw, keyway, or press
      fit) — this joint sees the highest shock load in the whole robot.
- [ ] **Competition rules**: confirm weapon energy, spin-up time, and containment requirements
      against the specific event's rulebook before attending.

None of the above were engineering-validated in this session — they are flagged, not solved.
