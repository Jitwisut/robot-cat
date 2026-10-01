"""Reproducible elastic screening of the V3 plow section (not robot FEA).

The drawing coordinates match build_robot_v3.py: length coordinate y=74..112 mm,
upper profile z=23..5 mm, and lower profile z=6..3..3 mm.  A vertical point
load acts at y=112.  Root at y=74 is idealized as perfectly clamped.
"""
import csv
import json
import math
from pathlib import Path

OUT = Path(__file__).resolve().parent
E_MPA = 68900.0
ASSUMED_YIELD_MPA = 275.0  # provisional 6061-T6 value; stock temper unverified
ROOT_Y = 74.0
TIP_Y = 112.0


def thickness(y):
    top = 23.0 - 18.0 * (y - ROOT_Y) / (TIP_Y - ROOT_Y)
    lower = 6.0 - 3.0 * (y - ROOT_Y) / 30.0 if y <= 104.0 else 3.0
    return top - lower


def compliance_mm_per_n(width_mm, n):
    """Simpson integration of tip deflection by Castigliano, n even."""
    if n % 2:
        raise ValueError('n must be even')
    h = (TIP_Y - ROOT_Y) / n
    s = 0.0
    for i in range(n + 1):
        y = ROOT_Y + i * h
        t = thickness(y)
        inertia = width_mm * t**3 / 12.0
        integrand = (TIP_Y - y)**2 / (E_MPA * inertia)
        s += (1 if i in (0, n) else 4 if i % 2 else 2) * integrand
    return s * h / 3.0


def peak_stress_coefficient(width_mm):
    """Peak elastic bending stress per N, evaluated along the plow span."""
    points = [(ROOT_Y + (TIP_Y - ROOT_Y) * i / 50000.0) for i in range(50001)]
    y = max(points, key=lambda p: 6.0 * (TIP_Y - p) / (width_mm * thickness(p)**2))
    return y, 6.0 * (TIP_Y - y) / (width_mm * thickness(y)**2)


def run():
    scenarios = []
    convergence = {}
    for width in (120.0, 10.0):
        compliances = {str(n): compliance_mm_per_n(width, n) for n in (40, 80, 160, 320, 640)}
        convergence[str(int(width))] = compliances
        comp = compliances['640']
        location, stress_per_n = peak_stress_coefficient(width)
        for force in (300.0, 583.0, 1163.0):
            stress = force * stress_per_n
            scenarios.append({
                'effective_contact_width_mm': width,
                'vertical_tip_force_N': force,
                'ideal_peak_elastic_bending_stress_MPa': round(stress, 4),
                'ideal_peak_location_y_mm': round(location, 4),
                'ideal_tip_displacement_mm': round(force * comp, 6),
                'ideal_elastic_yield_screen': 'exceeds assumed yield; elastic result invalid'
                if stress >= ASSUMED_YIELD_MPA else 'below assumed yield for plow member only',
            })
    result = {
        'model': '1D variable-section Euler-Bernoulli beam; perfect root; static vertical tip force',
        'source_geometry': 'Bolt_On_Full_Width_Plow_6061 in ROBOT_V3_Competition.f3d',
        'E_MPa': E_MPA,
        'assumed_yield_MPa': ASSUMED_YIELD_MPA,
        'convergence_mm_per_N': convergence,
        'scenario_results': scenarios,
        'excluded': ['plow bolts and tabs', 'front bulkhead', 'wheel/ground compliance',
                     '3D contact', 'plasticity', 'transient velocity and acceleration'],
    }
    (OUT / 'reduced_impact_results.json').write_text(json.dumps(result, indent=2) + '\n')
    with (OUT / 'reduced_impact_results.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=scenarios[0].keys())
        w.writeheader()
        w.writerows(scenarios)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    run()
