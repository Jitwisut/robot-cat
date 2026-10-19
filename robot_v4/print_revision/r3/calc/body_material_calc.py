"""Replace the printed TPU tub + lid with a sheet-metal (or other sheet) body: mass, balance and panel stiffness.

Body = the R3 Tub_TPU + Lid_TPU_2mm (CAD 388.9 + 51.3 g). A sheet body is modelled as flat panels on the same
outline (legacy_geometry.py), plus brackets that replace features the TPU tub has built in (8 motor walls,
2 upright side blocks, front floor wings, insert bosses). Every other part keeps its CAD mass and position.
Run: .venv/bin/python robot_v4/print_revision/r3/calc/body_material_calc.py   (writes body_material_calc.json)
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
R3 = os.path.dirname(HERE)
report = json.load(open(os.path.join(R3, 'output_R3', 'build_report.json')))
mass = report['mass']
parts = {p['part']: p for p in mass['parts']}
TOTAL = mass['total_g']
CG_Y = mass['centroid_mm'][1]
FRONT_Y, REAR_Y = 18.0, -70.0
LIMIT = 2000.0
body_now_g = parts['Tub_TPU']['mass_g'] + parts['Lid_TPU_2mm']['mass_g']
body_now_y = (parts['Tub_TPU']['mass_g'] * parts['Tub_TPU']['centroid_mm'][1] +
              parts['Lid_TPU_2mm']['mass_g'] * parts['Lid_TPU_2mm']['centroid_mm'][1]) / body_now_g

# panels on the R3 outline, mm (legacy_geometry.py): (name, area mm^2, centroid y mm)
PANELS = [
    ('floor y-110..64 x+-88', 176 * 174, -23.0),
    ('front floor wings y64..100 |x|34..88', 2 * 54 * 36, 82.0),
    ('side walls y-110..97 z7..58', 2 * 207 * 51, -6.5),
    ('rear wall', 160 * 51, -107.0),
    ('front bulkhead', 160 * 51, 63.0),
    ('lid y-110..64', 176 * 174, -23.0),
]
AREA = sum(a for _, a, _ in PANELS)
PANEL_Y = sum(a * y for _, a, y in PANELS) / AREA
# brackets replacing built-in TPU features (ASSUMED sizes, 2 mm thick, scaled by density):
#   8 motor wall brackets 26x51 mm, 2 upright brackets 32x43 mm folded L (x2 area), 12 PEM/rivet nuts ~0.5 g steel
BRACKET_AREA = 8 * 26 * 51 + 2 * 2 * 32 * 43
BRACKET_Y = (4 * 26 * 51 * 18 + 4 * 26 * 51 * -70 + 4 * 32 * 43 * 80) / BRACKET_AREA
HARDWARE_G = 12 * 0.5

# density g/cm^3, Young's modulus GPa, yield MPa (handbook, ASSUMED grade)
MATERIALS = {
    'steel_mild_SPCC': (7.85, 200, 250), 'stainless_304': (8.0, 193, 215), 'steel_spring_65Mn': (7.85, 200, 800),
    'aluminium_5052_H32': (2.68, 70, 193), 'aluminium_6061_T6': (2.70, 69, 276), 'titanium_gr2': (4.51, 105, 275),
    'polycarbonate': (1.20, 2.3, 62), 'TPU95A_printed_now': (1.21, 0.05, None),
}
THICK = {'steel_mild_SPCC': (0.5, 0.8, 1.0, 1.5), 'stainless_304': (0.5, 0.8, 1.0), 'steel_spring_65Mn': (0.5, 0.8),
         'aluminium_5052_H32': (1.0, 1.5, 2.0), 'aluminium_6061_T6': (1.5, 2.0, 3.0), 'titanium_gr2': (0.8, 1.0),
         'polycarbonate': (2.0, 3.0, 4.0)}


def rear_load(total_g, cg_y):
    return (FRONT_Y - cg_y) / (FRONT_Y - REAR_Y)


rest_g = TOTAL - body_now_g
rest_moment = TOTAL * CG_Y - body_now_g * body_now_y
rows = {}
for mat, ts in THICK.items():
    rho, E, sy = MATERIALS[mat]
    for t in ts:
        shell = AREA * t / 1000 * rho
        brackets = BRACKET_AREA * 2.0 / 1000 * rho if rho > 2 else BRACKET_AREA * 3.0 / 1000 * rho
        body = shell + brackets + HARDWARE_G
        total = rest_g + body
        cg = (rest_moment + shell * PANEL_Y + brackets * BRACKET_Y + HARDWARE_G * PANEL_Y) / total
        D_rel = (E * t ** 3) / (MATERIALS['TPU95A_printed_now'][1] * 4.0 ** 3)       # vs 4 mm TPU floor
        rows['%s|%.1fmm' % (mat, t)] = {
            'shell_g': round(shell, 1), 'brackets_g': round(brackets, 1), 'body_g': round(body, 1),
            'delta_vs_TPU_body_g': round(body - body_now_g, 1), 'robot_total_g': round(total, 1),
            'margin_to_2000_g': round(LIMIT - total, 1), 'fits_2kg': total <= LIMIT,
            'rear_load_pct': round(rear_load(total, cg) * 100, 2),
            'panel_bending_stiffness_vs_TPU4mm': round(D_rel, 0),
            'panel_yield_moment_Nmm_per_mm': None if sy is None else round(sy * t ** 2 / 6, 1)}

# armour-only option: keep the TPU tub, add steel/aluminium plates only on the front bulkhead and side walls
armour = {}
for mat, t in (('steel_mild_SPCC', 0.8), ('aluminium_6061_T6', 1.5), ('titanium_gr2', 0.8)):
    rho = MATERIALS[mat][0]
    a = 2 * 207 * 51 + 160 * 51
    g = a * t / 1000 * rho
    armour['%s|%.1fmm_sides+front' % (mat, t)] = {'added_g': round(g, 1), 'robot_total_g': round(TOTAL + g, 1),
                                                   'fits_2kg': TOTAL + g <= LIMIT}

# mass a metal body is allowed: current TPU body + the 24.5 g margin
allow = body_now_g + (LIMIT - TOTAL)
max_t = {mat: round(((allow - HARDWARE_G) * 1000 / rho - BRACKET_AREA * 2.0) / AREA, 2)
         for mat, (rho, _, _) in MATERIALS.items() if rho > 2}
out = {'body_now_g': round(body_now_g, 1), 'body_now_centroid_y_mm': round(body_now_y, 1),
       'allowed_body_g_for_2kg': round(allow, 1), 'panel_area_mm2': AREA, 'panel_centroid_y_mm': round(PANEL_Y, 1),
       'bracket_area_mm2': BRACKET_AREA, 'max_uniform_thickness_mm_within_2kg': max_t,
       'options': rows, 'armour_on_tpu': armour,
       'assumptions': ['flat panels on the R3 outline; bends, flanges and fastener tabs add ~5-10 %',
                       'brackets 2 mm metal (3 mm for polymer) replacing TPU motor walls / upright blocks',
                       'TPU flexural modulus 50 MPa ASSUMED; material grades not confirmed',
                       'no cost: needs a laser-cut + bending quote']}
json.dump(out, open(os.path.join(HERE, 'body_material_calc.json'), 'w'), indent=1)
print('TPU body now %.1f g at y %.1f; allowed metal body %.1f g; panel area %.0f mm2 + brackets %.0f mm2' % (
    body_now_g, body_now_y, allow, AREA, BRACKET_AREA))
print('max uniform thickness within 2 kg', max_t)
for k, v in rows.items():
    print('%-28s body %6.1f g  total %7.1f g  margin %7.1f  rear %5.2f%%  stiff x%-8.0f yieldM %s' % (
        k, v['body_g'], v['robot_total_g'], v['margin_to_2000_g'], v['rear_load_pct'],
        v['panel_bending_stiffness_vs_TPU4mm'], v['panel_yield_moment_Nmm_per_mm']))
print(json.dumps(armour, indent=1))
