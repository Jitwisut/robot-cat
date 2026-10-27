"""V4 R4-DIY: can R4-steel set A be built from common-shop stock and welded at home with a stick (ARC) welder?

User decisions 2026-10-27: ARC (stick) welder, ordinary steel shops are fine (12 x 12 tube), replace every custom
part that can be replaced. This file budgets mass part by part BEFORE any CAD, starting from the W1 CAD
(r4_steel/output/build_report.json, 1,918.4 g), and reports where the numbers do not close.

Every swap row: W1 part -> DIY part, stock + shop, old g, new g. Masses are W1 CAD masses or R3 CAD volumes x density,
scaled to the new thickness; tagged ASSUMED where nothing was measured.
Stock (web 2026-10-27, see sources): Thai Watsadu's smallest square tube is 19 x 19 x 1.2 (0.671 g/mm); ordinary
steel shops list 12 x 12 x 1.2 (1/2"); Thai Watsadu has no aluminium bar/plate and no solid PC sheet.
Coordinates: +Z up, nose +Y, mm (AGENTS.md). Nothing in r3/ or r4_steel/ is changed.
Run: python3 robot_v4/print_revision/r4_diy/calc/diy_substitution_calc.py   (writes diy_substitution_calc.json)
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PR = os.path.dirname(os.path.dirname(HERE))
w1 = json.load(open(os.path.join(PR, 'r4_steel', 'output', 'build_report.json')))
rows = {r['part']: r for r in w1['rows']}
RHO_ST, RHO_AL = 7.85e-3, 2.70e-3
LIMIT, TARGET = 2000.0, 1950.0
SRC = {
    'steellead_tube_table': 'https://www.steellead.com/steel-tube-price.html (12x12x1.2: 2.4 kg / 6 m, median 36 THB/kg)',
    'thaiwatsadu_tube_search': 'https://www.thaiwatsadu.com/th/search/เหล็กกล่อง (smallest 19x19x1.2, 119 THB / 6 m)',
    'thaiwatsadu_flat_bar': 'https://www.thaiwatsadu.com/th/search/เหล็กแบน (thinnest 4 mm)',
    'thaiwatsadu_pc': 'https://www.thaiwatsadu.com/th/search/โพลีคาร์บอเนต ตัน (6 mm twin-wall and corrugated only)',
}


def g(name):
    return rows[name]['mass_g']


def tube_g_per_mm(side, wall):
    return (side ** 2 - (side - 2 * wall) ** 2) * RHO_ST


# R3 CAD volumes (mm^3) of the parts that change material/thickness (revision.build(), dumped 2026-10-27)
VOL = {'Upright_L': 12135, 'Upright_R': 12270, 'Top_Brace': 2287, 'Bottom_Brace': 1871, 'M06': 4797}
TUBE, WALL = 12.0, 1.2                      # 1/2" black (not galvanised) square tube, ordinary steel shop
SHEET = 1.6                                 # black sheet offcut 1/16", tabs / braces / motor mount / wedgelets
FLAT = 3.2                                  # 2" x 1/8" (50.8 x 3.2) flat bar, uprights
WELD_ARC = 0.08                             # stick beads + spatter, fraction of steel: ASSUMED (W1 used 4 % for MIG)

swaps = []


def swap(group, old, new_desc, shop, old_g, new_g, note=''):
    swaps.append({'group': group, 'w1': old, 'diy': new_desc, 'shop': shop, 'old_g': round(old_g, 2), 'new_g': round(new_g, 2),
                  'delta_g': round(new_g - old_g, 2), 'note': note})


# ---------------------------------------------------------------- frame tubes: same W1 cut list, 12 x 12 x 1.2
tube_rows = [c for c in w1['tube_cut_list_mm']]
for c in tube_rows:
    old = g(c['part'])
    swap('tube', c['part'], 'tube 12x12x1.2 L%.0f' % c['length_mm'], 'ร้านเหล็ก', old, c['length_mm'] * tube_g_per_mm(TUBE, WALL))

# ---------------------------------------------------------------- tabs kept as rectangles from 1.6 sheet
KEEP_TABS = [n for n in rows if n.startswith(('Tab_Drive_Motor', 'Tab_Hinge', 'Tab_Battery_Strap', 'Clamp_Drive_Motor_Post', 'Tab_Weapon_Mount'))]
for n in KEEP_TABS:
    swap('tab', n, n + ' t1.6 hand-cut rectangle', 'ร้านเหล็ก (เศษแผ่น 1.6)', g(n), g(n) * SHEET / 1.5)

# ---------------------------------------------------------------- weapon module: welded steel replaces bolted 6061
web_tabs = [n for n in rows if n.startswith(('Tab_Upright_Web', 'Tab_Upright_Flange', 'Tab_Upright_Rung'))]
for side in ('L', 'R'):
    old = g('Upright_6061_6mm_' + side) + sum(g(n) for n in web_tabs if n.endswith('_' + side))
    new = VOL['Upright_' + side] / 6.0 * FLAT * RHO_ST          # same outline, 3.2 mm
    new += 2.0 * 12.0 * FLAT * RHO_ST                           # foot extended 2 mm back onto the bulkhead cross face (y 62)
    new += (46.0 - 30.2) * 26.0 * SHEET * RHO_ST                # floor flange upright -> rail inner face, y 62..88
    swap('weapon', 'Upright_6061_6mm_%s + web/flange/rungs' % side, 'upright 2"x1/8" flat bar welded + 1.6 flange',
         'ร้านเหล็ก', old, new, 'no M3 taps/bolts; axle holes stack-drilled before welding')
swap('weapon', 'Weapon_Top_Brace_3mm (6061)', 'top brace 1.6 sheet welded', 'ร้านเหล็ก', g('Weapon_Top_Brace_3mm'), VOL['Top_Brace'] / 3 * SHEET * RHO_ST)
swap('weapon', 'Weapon_Bottom_Brace_3mm (6061)', 'bottom brace 1.6 sheet welded', 'ร้านเหล็ก', g('Weapon_Bottom_Brace_3mm'), VOL['Bottom_Brace'] / 3 * SHEET * RHO_ST)
swap('weapon', 'Weapon_Motor_Mount_3mm (6061, notched)', 'motor mount 1.6 sheet welded to the two feet', 'ร้านเหล็ก', g('Weapon_Motor_Mount_3mm'),
     VOL['M06'] / 3 * SHEET * RHO_ST, 'stiffness E*t^3 = 46 % of 3 mm 6061: checked in the strength calc')
for side in ('L', 'R'):
    swap('weapon', 'Beater_Disc_%s (Hardox 450)' % side, 'disc 6 mm SS400 plate, ground to a 1:1 template', 'ร้านเหล็ก (เศษแผ่น 6)',
         g('Beater_Disc_' + side), g('Beater_Disc_' + side), 'same outline -> same inertia/energy; softer teeth (wear, not strength)')

# ---------------------------------------------------------------- wedgelet: SS400 with bent hinge ears, M3 bolt pins
EAR = (12.0, 12.0)          # ear tongue bent down from the wedgelet blank, mm
for side in ('L', 'R'):
    swap('wedgelet', 'Wedgelet_%s (65Mn)' % side, 'wedgelet 1.6 SS400 sheet', 'ร้านเหล็ก (เศษแผ่น 1.6)', g('Wedgelet_' + side), g('Wedgelet_' + side),
         'MACHINING.md: SS400 lifts 1.9x the R3 lifter load')
    old = g('Wedgelet_Carrier_PETG_%s1' % side) + g('Wedgelet_Carrier_PETG_%s2' % side)
    swap('wedgelet', 'Wedgelet carriers %s1/%s2 (6061 machined)' % (side, side), '2 ears bent from the wedgelet blank', '-', old,
         2 * EAR[0] * EAR[1] * SHEET * RHO_ST)
    swap('wedgelet', 'Hinge_Pin_D3_%s + E-clip (lathe)' % side, 'M3x60 bolt + nylock', 'ร้านน็อต', g('Hinge_Pin_D3_' + side) + g('Eclip_DIN6799_2.3_' + side),
         2.9 + 0.4, 'ASSUMED steel M3x60 2.9 g')
swap('misc', 'Hall_Post_PETG (6061 machined)', 'Hall post 1.6 strip L, welded on the cross', 'ร้านเหล็ก', g('Hall_Post_PETG'), 8 * 45 * SHEET * RHO_ST)
swap('misc', 'Dead_Shaft_M8_12.9 x80', 'M8x75 12.9 (uprights 3.2 not 6)', 'ร้านน็อต', g('Dead_Shaft_M8_12.9'), g('Dead_Shaft_M8_12.9') * 75 / 80)

# ---------------------------------------------------------------- weld beads: ARC instead of MIG
w1_steel = sum(r['mass_g'] for r in w1['rows'] if r.get('material', '') and r['material'].startswith('steel_'))
diy_steel = w1_steel + sum(s['delta_g'] for s in swaps if s['group'] in ('tube', 'tab')) \
    + sum(s['new_g'] for s in swaps if s['group'] == 'weapon' and 'Disc' not in s['w1'])   # welded weapon parts join the steel
swap('weld', 'weld beads + paint 4 % (MIG)', 'weld beads + spatter + paint %.0f %% (ARC)' % (WELD_ARC * 100), '-', g('weld beads + paint'), WELD_ARC * diy_steel,
     'ASSUMED until the frame is weighed')

unchanged = ['PC 2 mm skins (ร้านพลาสติก/อะคริลิค, ตัดเองได้)', 'skirts 0.8 Al + 11 brackets 1.5 Al (ร้านอลูมิเนียม/ร้านกระจก)',
             'weapon hub, motor pulley, 3 spacers, 2 wheel hubs (ร้านกลึง: need a lathe)', 'O-rings, bearings, motors, electronics, rivnuts']
base = w1['mass']['total_g']
delta = sum(s['delta_g'] for s in swaps)
total = base + delta
by_group = {}
for s in swaps:
    by_group[s['group']] = round(by_group.get(s['group'], 0) + s['delta_g'], 1)

# ---------------------------------------------------------------- ways to close the budget (each applied alone and with the strip)
tpm = tube_g_per_mm(TUBE, WALL)
top_cross = next(c for c in tube_rows if c['part'] == 'Frame_Top_Rear_Cross')['length_mm']
post = next(c for c in tube_rows if c['part'] == 'Frame_Post_Rear_L')['length_mm']
strip_top = top_cross * 12 * SHEET * RHO_ST
options = {
    'D0 keep everything': 0.0,
    'D1 top rear cross -> 1.6 x 12 strip on the posts (lid clamp only)': (strip_top - top_cross * tpm) * (1 + WELD_ARC),
    'D2 NoCage: no rear posts/top cross, lid on bought Al angles (+3 g)': -(top_cross + 2 * post) * tpm * (1 + WELD_ARC) + 3.0,
}
opt_out = {k: {'total_g': round(total + v, 1), 'margin_2000_g': round(LIMIT - total - v, 1), 'margin_1950_g': round(TARGET - total - v, 1)}
           for k, v in options.items()}

out = {'sources': SRC, 'stock': {'tube': '12x12x1.2', 'tube_g_per_mm': round(tpm, 4), 'tube_19x19x1.2_g_per_mm': round(tube_g_per_mm(19, 1.2), 4),
                                 'sheet_mm': SHEET, 'flat_bar': '50.8 x %.1f' % FLAT, 'weld_fraction': WELD_ARC},
       'w1_total_g': base, 'swaps': swaps, 'delta_by_group_g': by_group, 'delta_g': round(delta, 1), 'diy_total_g': round(total, 1),
       'diy_steel_g': round(diy_steel, 1), 'unchanged': unchanged, 'options': opt_out}
json.dump(out, open(os.path.join(HERE, 'diy_substitution_calc.json'), 'w'), ensure_ascii=False, indent=1)
for s in swaps:
    print('%-9s %-46s -> %-50s %7.1f -> %7.1f  (%+6.1f)' % (s['group'], s['w1'][:46], s['diy'][:50], s['old_g'], s['new_g'], s['delta_g']))
print('by group', by_group)
print('W1 %.1f g  delta %+.1f g  DIY %.1f g   steel %.1f g' % (base, delta, total, diy_steel))
for k, v in opt_out.items():
    print('%-70s %s' % (k, v))
