"""Reproducible first-yield screening for the current lifter geometry.

This is a beam idealization, not a Fusion FEA run or a whole-robot crash test.
Forces are N, dimensions mm, material strength MPa = N/mm^2.
"""
from __future__ import annotations

import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'exports' / 'impact_screening_2026-09-24.json'

# Read from the live robot2 document via Fusion MCP on 2026-09-24.
YIELD_MPA = 275.0  # Aluminum 6061 assigned to pin and spatula in CAD.
PIN_DIAMETER_MM = 6.0
PIN_SUPPORT_SPAN_MM = 67.0  # center-to-center: support x=-33.5 to +33.5.
SPATULA_WIDTH_MM = 62.0
TIP_Y_MM = 112.0
ROOT_Y_MM = 74.0  # aft end of the spatula is overlapped by the hinge boss.


def spatula_thickness(y_mm: float) -> float:
    # Side polygon in AddWedgeLifterOption.py, valid over y=74..112 mm.
    top_z = 30.0 + (8.5 - 30.0) * (y_mm - 72.0) / 40.0
    bottom_z = 27.0 + (5.8 - 27.0) * (y_mm - 70.0) / 42.0
    return top_z - bottom_z


def plate_yield_force(width_mm: float) -> tuple[float, float]:
    # Beam stress sigma = 6 F L/(b t^2) for a vertical tip load.
    samples = []
    for n in range(1000):
        y = ROOT_Y_MM + (TIP_Y_MM - ROOT_Y_MM) * n / 1000.0
        lever = TIP_Y_MM - y
        t = spatula_thickness(y)
        samples.append((YIELD_MPA * width_mm * t * t / (6.0 * lever), y))
    return min(samples)


def pin_yield_force(load_model: str) -> float:
    # sigma = 32 M/(pi d^3); simple supports at both ends.
    # Central point load: M = F L/4; distributed total load: M = F L/8.
    denominator = 8.0 if load_model == 'central_point' else 4.0
    return (YIELD_MPA * math.pi * PIN_DIAMETER_MM**3 /
            (denominator * PIN_SUPPORT_SPAN_MM))


def impact_average_force(mass_kg: float, speed_m_s: float,
                         stopping_distance_mm: float) -> float:
    # Work-energy average F = (1/2 m v^2)/delta. Peak force is unknown.
    return 0.5 * mass_kg * speed_m_s**2 / (stopping_distance_mm / 1000.0)


def main() -> None:
    plate_full, plate_y = plate_yield_force(SPATULA_WIDTH_MM)
    plate_strip, strip_y = plate_yield_force(10.0)
    point = pin_yield_force('central_point')
    distributed = pin_yield_force('distributed')
    scenarios = [
        {'speed_m_s': speed, 'stop_mm': stop,
         'average_force_N': round(impact_average_force(1.5, speed, stop), 1)}
        for speed in (1.0, 2.0, 3.0)
        for stop in (2.0, 5.0, 10.0)
    ]
    result = {
        'model': 'idealized first-yield screening; no whole-robot failure claim',
        'source': 'live Fusion robot2 body geometry and assigned material, 2026-09-24',
        'yield_MPa': YIELD_MPA,
        'pin': {'diameter_mm': PIN_DIAMETER_MM,
                'span_mm': PIN_SUPPORT_SPAN_MM,
                'central_point_yield_N': round(point, 1),
                'distributed_yield_N': round(distributed, 1)},
        'spatula': {'full_width_mm': SPATULA_WIDTH_MM,
                    'root_thickness_mm': round(spatula_thickness(ROOT_Y_MM), 3),
                    'full_width_ideal_yield_N': round(plate_full, 1),
                    'full_width_critical_y_mm': round(plate_y, 2),
                    'ten_mm_strip_ideal_yield_N': round(plate_strip, 1),
                    'ten_mm_strip_critical_y_mm': round(strip_y, 2)},
        'impact_scenarios_for_1p5kg': scenarios,
        'limitations': [
            'Pin load distribution, contact widths, clearances and joints are not defined.',
            'Wedges have no validated mechanical fastener load path in the current design.',
            'Hinge actuator, stop and battery/motor retention are not detailed.',
            'Yield is permanent deformation onset, not fracture or robot disablement.',
            'Impact average force is not peak force; actual effective mass and speed unknown.',
            'Material temper of purchased stock and printed part properties are unverified.',
        ],
    }
    assert 300 < point < 400
    assert 600 < distributed < 800
    assert 1000 < plate_full < 1300
    OUTPUT.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
