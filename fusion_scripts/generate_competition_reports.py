"""Build current dimensions and mass summaries from a Fusion MCP JSON snapshot.

The input report is exported from the live ``robot2`` design after all geometry
edits. This script does not change the CAD model.
"""

from pathlib import Path
import json


ROOT = Path(__file__).resolve().parents[1]
EXPORTS = ROOT / "exports"
DATA = json.loads((EXPORTS / "competition_model_report.json").read_text())
PARTS = DATA["parts"]
PHYSICAL = [p for p in PARTS if not p["analysis_envelope"]]


def rng(lo, hi):
    return f"{lo:.2f}…{hi:.2f}"


def write_dimensions():
    overall_min = [min(p["min"][i] for p in PHYSICAL) for i in range(3)]
    overall_max = [max(p["max"][i] for p in PHYSICAL) for i in range(3)]
    lines = [
        "# ขนาด Assembly รุ่นแข่ง — `robot2`",
        "",
        "วัดจาก Fusion หลังถอดค้อนบนและใส่ตัวแทนสวิตช์ FingerTech แล้ว "
        "แกนโลก: X = ซ้าย/ขวา, Y = สูงขึ้น, Z = หน้า/หลัง; กริดพื้นอยู่ที่ Y=0.",
        "",
        "ข้อมูลนี้แทนรายงานเก่าที่วัดก่อนหมุน Assembly; สำเนาเดิมอยู่ที่ "
        "`Dimensions_and_Fasteners_legacy.md`. ตัวเลขรูยึดที่ไม่ได้ระบุใหม่ด้านล่าง"
        "ต้องอ่านจาก CAD/DXF ปัจจุบันและตรวจอะไหล่จริงก่อนส่งผลิต.",
        "",
        "## ขอบเขตชิ้นงาน (ไม่รวม analysis envelope)",
        "",
        "| แกน | ต่ำสุด | สูงสุด | ช่วง |",
        "|---|---:|---:|---:|",
    ]
    for i, axis in enumerate("XYZ"):
        lines.append(f"| {axis} | {overall_min[i]:.2f} | {overall_max[i]:.2f} | "
                     f"{overall_max[i]-overall_min[i]:.2f} mm |")
    lines += [
        "",
        "## ขนาดชิ้นส่วนในพิกัดโลก",
        "",
        "| กลุ่ม | ชิ้นส่วน | X (mm) | Y (mm) | Z (mm) | วัสดุใน CAD |",
        "|---|---|---:|---:|---:|---|",
    ]
    for p in PHYSICAL:
        lines.append(
            f"| {p['group']} | `{p['name']}` | {rng(p['min'][0],p['max'][0])} | "
            f"{rng(p['min'][1],p['max'][1])} | {rng(p['min'][2],p['max'][2])} | "
            f"{p['material'] or 'ไม่ระบุ'} |"
        )
    lines += [
        "",
        "## รูยึดที่แก้ล่าสุด",
        "",
        "- `FingerTech_Mini_Switch_Envelope`: ตัวเรือน 12.7 × 12.7 × 6.35 mm; "
        "รู M2 สองรูห่าง 7.62 mm ตามสเปกผู้ผลิต. เป็นตัวแทนรูปทรง ยังไม่รวมขั้วทองแดงและสายไฟ.",
        "- `FingerTech_Switch_Adapter`: 28 × 18 × 4 mm, Nylon 6; "
        "รูสวิตช์ Ø2.2 mm ศูนย์ที่พิกัดชิ้นส่วนเดิม X=38.19/45.81, Y=-70 mm; "
        "รูยึดเสา Ø3.4 mm X=32.5/51.5, Y=-70 mm. พิกัดนี้เป็นพิกัดภายในชิ้นส่วน "
        "ก่อนหมุน Assembly; ในแกนโลก X คงเดิมและ Z=70 mm.",
        "- ช่องเปิดฝาบนเดิม 20 × 16 mm ถูกปิดและแทนด้วยรูเข้าประแจ Ø6 mm "
        "ที่ศูนย์พิกัดชิ้นส่วน X=42, Y=-70 mm; ต้องตรวจระยะเอื้อมประแจและตำแหน่งขั้วไฟกับชิ้นจริง.",
        "- รูยึด ESP32 บน holder ยังอิงรูปถ่าย ±0.5 mm; วัดบอร์ดจริงก่อนพิมพ์.",
        "",
        "**ยังไม่ใช่แบบผลิตขั้นสุดท้าย:** ขนาดอะไหล่ที่ซื้อ, รูเพลาอาวุธ, "
        "การทำปีกหน้าเปลือกกลวง และทางเดินสายต้องยืนยันก่อนส่งร้าน.",
        "",
    ]
    (EXPORTS / "Dimensions_and_Fasteners.md").write_text("\n".join(lines), encoding="utf-8")


def write_mass():
    lines = [
        "# มวลรุ่นแข่ง — จาก Fusion ล่าสุด",
        "",
        "อ้างอิง `competition_model_report.json` ซึ่งอ่านจาก `robot2` หลังถอดค้อนบน "
        "และแก้สวิตช์; สำเนารายงานก่อนแก้อยู่ที่ `Mass_Report_legacy.md`.",
        "",
        "| รายการ | มวล CAD |",
        "|---|---:|",
        f"| ชิ้นงานทั้งหมด ยกเว้น analysis envelope | {DATA['mass_real_cad_g']:.1f} g |",
        f"| Analysis envelope (ไม่ใช่ชิ้นงาน) | {DATA['mass_envelopes_g']:.1f} g |",
        f"| รวมตาม solid ใน Fusion | {DATA['mass_real_cad_g']+DATA['mass_envelopes_g']:.1f} g |",
        "",
        "ตัวเลข CAD ของอะไหล่ที่ซื้อเป็นเพียงมวลของ solid ที่ใช้จัดวาง "
        "จึงยังใช้ยืนยันน้ำหนักลงแข่งไม่ได้. เช่นตัวแทนสวิตช์ใน CAD หนัก 1.10 g "
        "แต่ผู้ผลิตระบุของจริง 2.15 g. ต้องชั่งอะไหล่จริงพร้อมสาย, หัวต่อ, น็อต, "
        "สายพาน และตัวล็อกอาวุธก่อนล็อกน้ำหนัก.",
        "",
        "## ตามกลุ่ม (มวล CAD รวม envelope ในกลุ่ม)",
        "",
        "| กลุ่ม | มวล |",
        "|---|---:|",
    ]
    for name, mass in DATA["mass_by_group_cad_g"].items():
        lines.append(f"| {name} | {mass:.1f} g |")
    lines += ["", "## รายชิ้น", "", "| ชิ้นส่วน | มวล CAD | หมายเหตุ |",
              "|---|---:|---|"]
    for p in PARTS:
        note = "analysis envelope" if p["analysis_envelope"] else ""
        if p["name"] == "FingerTech_Mini_Switch_Envelope":
            note = "ตัวแทนรูปทรง; ของจริง 2.15 g ตามผู้ผลิต"
        lines.append(f"| `{p['name']}` | {p['mass_g']:.2f} g | {note} |")
    lines.append("")
    (EXPORTS / "Mass_Report_Generated.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    write_dimensions()
    write_mass()
    print("Updated dimensions and mass reports from competition_model_report.json")
