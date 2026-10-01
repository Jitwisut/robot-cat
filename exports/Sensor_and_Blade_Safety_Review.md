# Sensor/Electronics Mounting & Blade Fastening — Engineering Safety Review

> **รายงานประวัติ:** เนื้อหาด้านล่างเขียนก่อนการเปลี่ยนอิเล็กทรอนิกส์จริง,
> การใส่พูลเลย์ 1:1, การแก้สวิตช์ FingerTech และการถอดค้อนบน.
> บางข้อความ เช่น “ยังไม่มีพูลเลย์”, “Main_Disconnect ยังไม่มีแท่น”, และ
> งบน้ำหนัก 31 กรัม จึงไม่ตรงกับ `robot2` ปัจจุบัน.
> อ่าน `../STATUS.md`, `Design_Review_Competition_2026-09-23.md` และ
> `Mass_Report_Generated.md` สำหรับรุ่นแข่งล่าสุด. รายการตรวจเพลา/พิน/สมดุล
> ที่ยังไม่ได้ทำยังคงเป็นข้อค้างจริง.

## 1. Electronics/"sensor" mounting audit

No dedicated sensors (IMU, current sensor, encoder) are in this build's BOM — the closest
things to "sensor" hardware are the **RC receiver** (signal input) and the **ESP32 control
board**. Audited and fixed in this pass:

| Component | Before | After |
|---|---|---|
| `ESP32_Control_Board` | Sat on standoffs, no screw holes through the board itself | ~~4× M2 holes added~~ — **this was wrong, see below**. Now genuinely 4× Ø2.9 holes at the real standoff centres |
| `RC_Receiver` | Floating box, zero mounting features | 2× M2 holes added |
| `Wheel_Motor_Drivers` | Floating box, zero mounting features | 2× M2 holes added |
| `Power_Buck_BEC` | Floating box, zero mounting features | 2× M2 holes added |
| `Main_Disconnect` | Floating box, zero mounting features | 2× M2 holes added |

### Correction: the PCB's holes were reported but never cut

An audit that counted cylindrical faces in the actual solid — rather than trusting the
earlier log line — found **zero** holes in `ESP32_Control_Board`. The earlier pass sketched
them on the component XY plane at z = 0 and cut ±4 mm from there, but the board had already
been raised onto the 3 mm standoffs and sits at z = 8.0–9.6 mm. The cut missed the body
completely, removed nothing, and still reported "4 mount holes cut".

They are now cut at the real standoff centres — (±26, −59) and (±26, −17) mm, Ø2.9 to match
the bore already in each `PCB_Standoff` — and verified by re-counting: 0 → 4.

**Lesson worth keeping: a feature that reports success is not evidence that it removed
material.** Count the result in the solid.

**Resolved:** the base plate now has matching
Ø2.2 mm through-holes under every module that actually sits on the floor, found by reading
the module's own M2 hole positions out of the solid rather than by assuming a layout.

A *boss* was rejected deliberately, not overlooked: the base plate is 2 mm aluminium, so
there is no material to raise a boss from, and a printed boss would itself need fastening to
the plate — circular. The buildable answer at this scale is a through-hole: M2 countersunk
screw up from underneath, nut on top. Note the ground clearance is only 3 mm, so the underside
must be countersunk (90° × Ø3.8, ≤1.2 mm deep) — that is *not* modelled.

`Main_Disconnect` is skipped on purpose: it sits at z 45–55 mm on a bracket, not on the floor,
so a base-plate hole under it would mean a 45 mm long M2 post. It still needs its own bracket.

## 2. Blade (Spinner_Bar) → shaft fastening — engineering analysis

**This was the single biggest safety gap in the whole design**: until this pass, the weapon
bar had *no modeled fastening to the shaft at all* — it just sat there in the CAD.

### What was wrong
A plain friction fit or a single set-screw on a **3.17mm round shaft** is a well-known weak
point in small spinner weapons: the contact patch is tiny, and shock-loading during a strike
routinely spins the bar loose on the shaft, even when the set-screw is torqued correctly. This
is the most common structural failure mode reported in small combat-robot spinners.

### Fix applied
Added a **Ø2mm radial roll-pin hole** through both `Spinner_Shaft` and `Spinner_Bar` at their
crossing point (`roll_pin_d` parameter). A roll pin (or solid dowel pin) through both parts
transmits torque in shear across the pin's full cross-section, not by friction — it is
standard practice for small high-shock rotating assemblies and is significantly more reliable
than a set-screw alone. **This still needs someone experienced to size the pin against real
impact loads** — 2mm is a reasonable starting point for this weight class, not a calculated
final value.

### Kinetic energy estimate (why this matters)
Using the confirmed A2212 1400KV motor on 3S (11.1V):
- No-load speed ≈ 1400 KV × 11.1V ≈ **15,540 RPM** (ω ≈ 1,627 rad/s)
- `Spinner_Bar` mass (thin bar about centre, I = mL²/12, L = 65 mm)

| Bar material | Mass | I | KE at 1:1 |
|---|---:|---:|---:|
| Aluminium 6061 (old mock) | ≈ 21 g | 7.4×10⁻⁶ kg·m² | **≈ 9.8 J** |
| **Aluminium 7075-T6 (now applied)** | ≈ 22 g | 7.7×10⁻⁶ kg·m² | **≈ 10.2 J** |
| Steel (considered, not applied) | ≈ 61 g | 2.2×10⁻⁵ kg·m² | **≈ 28.6 J** |

7075-T6 was chosen over steel because steel nearly triples stored energy *and* adds ~40 g to a
build that is only ~31 g under its 1,500 g target. 7075 buys the hardness the striking edge
needs at essentially unchanged energy.

**The energy figures above are incomplete, and the gap matters.** The CAD places the motor
parallel to the weapon shaft with a 49.25 mm centre distance — i.e. a *belt drive* — but no
pulleys or belt are modelled, so **the drive ratio has not been chosen**. Every number above
assumes 1:1. Kinetic energy scales with the square of the ratio: a 2:1 step-up puts this at
roughly 40 J, not 10 J. **Do not quote ~10 J to an event's tech inspection until the pulleys
are actually specified.**

Loaded RPM (against air drag + cutting loads) is lower than no-load, but even the 1:1 figure is
already in the range where events require rated containment and a weapon-lock procedure — this
is not a toy-level force. **Do not spin this at any RPM without a proper containment box.**

### Checklist for a human reviewer (still required)
- [ ] Confirm roll pin diameter/material against actual impact loads (not just steady torque)
- [x] Bar material moved off the 6061 mock to 7075-T6 — *applied in CAD, still needs a human to
      confirm 7075 is the right call for your event rather than a hardened striking insert*
- [ ] **Confirm the belt ratio against your event's rules** — 1:1 GT2 pulleys are modelled as a placeholder (~10.1 J); energy scales with ratio²
- [x] **Fixed 2026-09-23: shaft is now 4 mm, bearing stays 624.** Was: the bearing/shaft mismatch: `spinner_bearing_OD` 13 mm is a 624 (4 mm bore) but
      `spinner_shaft_D` is 3.17 mm (1/8"). These do not fit each other. Either 623 + 3 mm shaft,
      R2-5 + 1/8" shaft, or step the shaft to 4 mm and keep the 624 (preferred — a thicker shaft
      survives shock better)
- [ ] Dynamic balance of the bar around the shaft axis
- [ ] Shaft diameter/bearing rating under shock, not just steady rotation
- [ ] Event-specific weapon energy and containment rules

**I am not a substitute for this review.** The CAD and the roll-pin fix reduce one specific,
well-known failure mode; they do not constitute a safety sign-off.
