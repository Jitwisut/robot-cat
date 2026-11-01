"""Welded steel frame instead of the printed PETG parts of the PC body: does it fit the 2000 g limit?

Starting point is the PC-body build (pc_body_report.json, expected small parts solid). A frame is a set of members
on the PC-body box (x +-83, y -110..97, z 4..62) whose mass must fit inside what it replaces plus the current margin.
Scenarios:
  S1 frame replaces every printed PETG part (cleats, brackets, side blocks, wedgelet blocks, motor cradles); PC panels stay
  S2 frame also replaces the PC side walls, rear wall and bulkheads (PC floor + lid stay)
Each scenario needs welded/bolted tabs for the motors, uprights and wedgelet hinges (TABS below, ASSUMED sizes).
A bolted aluminium-angle frame (no print, no weld) is computed the same way for comparison.
Run: python3 robot_v4/print_revision/r3/calc/steel_frame_calc.py   (writes steel_frame_calc.json)
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
R3 = os.path.dirname(HERE)
pc = json.load(open(os.path.join(R3, 'pc_body', 'output', 'pc_body_report.json')))
LIMIT = 2000.0
TOTAL = pc['balance']['expected_small_parts_solid']['total_g']
MARGIN = LIMIT - TOTAL
panels = {p['file'][:-4]: p['mass_g'] for p in pc['panels']}
PETG = pc['body_breakdown_g']['new PETG parts (80 % rule)'] + sum(g for _, g in pc['mass_check']['r3_parts_not_in_rows'])
WALLS = sum(panels[k] for k in ('PC3_Side_Wall_L', 'PC3_Side_Wall_R', 'PC3_Rear_Wall', 'PC2_Bulkhead_L', 'PC2_Bulkhead_R'))
FREED = {'S1': PETG, 'S2': PETG + WALLS}
RHO = {'steel': 7.85e-3, 'aluminium': 2.70e-3}   # g/mm^3

# Member lengths on the PC-body box, mm
W, L, H = 166.0, 207.0, 58.0
RING = 2 * (W + L)                      # one perimeter ring
BULK = W                                # cross member at the weapon bulkhead (y 62)
FRAMES = {
    'F1 bottom ring + bulkhead cross': RING + BULK,
    'F2 cage: bottom + top ring, 4 posts, bulkhead cross bottom + top': 2 * RING + 4 * H + 2 * BULK,
    'J  corner joints only (angle along every panel edge, replaces cleats)': 4 * L + 2 * W + 2 * H + 2 * 52,
}
SECTIONS = {   # name: (material, cross-section area mm^2)
    'steel square tube 10x10x0.8': ('steel', 10 * 10 - 8.4 * 8.4),
    'steel square tube 10x10x1.0': ('steel', 10 * 10 - 8 * 8),
    'steel square tube 12x12x1.0': ('steel', 12 * 12 - 10 * 10),
    'steel angle 15x15x1.5': ('steel', 2 * 15 * 1.5 - 1.5 * 1.5),
    'steel flat bar 10x2': ('steel', 10 * 2),
    'steel round bar D6': ('steel', 3.14159 * 9),
    'aluminium angle 10x10x1': ('aluminium', 2 * 10 * 1 - 1),
    'aluminium angle 15x15x1.5': ('aluminium', 2 * 15 * 1.5 - 1.5 * 1.5),
}
# Tabs every frame needs (ASSUMED, flat plate): 4 drive-motor plates 26x33, 2 upright side plates 32x43, 2 wedgelet hinge tabs 20x30
TAB_AREA = 4 * 26 * 33 + 2 * 32 * 43 + 2 * 20 * 30
TAB_T = {'steel': 2.0, 'aluminium': 3.0}

rows = []
for sname, (mat, area) in SECTIONS.items():
    g_per_m = area * RHO[mat] * 1000
    tabs = TAB_AREA * TAB_T[mat] * RHO[mat]
    for fname, length in FRAMES.items():
        if (fname.startswith('J')) != ('angle' in sname):
            continue
        frame_g = area * length * RHO[mat] + tabs
        for sc in ('S1', 'S2'):
            if fname.startswith('J') and sc == 'S2':
                continue        # corner angles alone leave the walls open
            budget = FREED[sc] + MARGIN
            rows.append({'section': sname, 'g_per_m': round(g_per_m, 1), 'frame': fname, 'length_mm': length, 'scenario': sc, 'members_only_g': round(area * length * RHO[mat], 1),
                         'frame_plus_tabs_g': round(frame_g, 1), 'budget_g': round(budget, 1), 'margin_g': round(budget - frame_g, 1),
                         'robot_total_g': round(TOTAL - FREED[sc] + frame_g, 1), 'fits_2kg': frame_g <= budget})

# Thinnest steel wall skin S2 would need to close the open sides (side walls + rear wall, z 7..58)
skin_area = 2 * L * 51 + (2 * 80) * 51
out = {
    'start_total_g': TOTAL, 'margin_to_2000_g': round(MARGIN, 2), 'petg_replaced_g': round(PETG, 2), 'pc_walls_replaced_g': round(WALLS, 2),
    'budget_g': {k: round(v + MARGIN, 1) for k, v in FREED.items()},
    'tabs': {'area_mm2': TAB_AREA, 'steel_2mm_g': round(TAB_AREA * 2 * RHO['steel'], 1), 'aluminium_3mm_g': round(TAB_AREA * 3 * RHO['aluminium'], 1)},
    'steel_skin_0.5mm_for_open_walls_g': round(skin_area * 0.5 * RHO['steel'], 1),
    'rows': rows,
    'assumptions': ['frame members on the PC-body outline 166 x 207 x 58 mm; no mitre/weld-bead allowance (welds add ~2-5 %)',
                    'tab sizes ASSUMED from the PETG parts they replace; rear-load effect not computed (frame mass is near-symmetric in y)',
                    'steel cannot be welded to the 6061 uprights or 5052 skirts: those stay bolted'],
}
json.dump(out, open(os.path.join(HERE, 'steel_frame_calc.json'), 'w'), ensure_ascii=False, indent=1)
print('start %.1f g, margin %.1f g, PETG %.1f g, PC walls %.1f g, tabs steel %.1f g' % (TOTAL, MARGIN, PETG, WALLS, out['tabs']['steel_2mm_g']))
for r in rows:
    print('%-28s %-62s %s  members %6.1f  frame %6.1f  budget %6.1f  margin %7.1f  total %7.1f  %s' % (
        r['section'], r['frame'][:62], r['scenario'], r['members_only_g'], r['frame_plus_tabs_g'], r['budget_g'], r['margin_g'], r['robot_total_g'], 'OK' if r['fits_2kg'] else 'OVER'))
print('steel 0.5 mm skin to close S2 walls: %.1f g' % out['steel_skin_0.5mm_for_open_walls_g'])
