"""Reproducible reduced-order impact simulation for the current robot2 lifter.

This models the 6061 lifter blade as a variable-thickness Euler-Bernoulli beam,
the 6 mm steel pivot as a simply supported beam, and impact as a 1-DOF
elastoplastic collision. It is not a whole-robot Fusion Event Simulation.
SI units throughout unless an output key says otherwise.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'exports' / 'impact_sim_2026-09-24'
OUT.mkdir(exist_ok=True)

E_AL = 68.9e9
Y_AL = 275e6  # CAD-assigned 6061; heat treatment needs verification.
E_STEEL = 200e9  # Illustrative modulus; pivot alloy/grade is unspecified.
BLADE_ROOT_Y = 74e-3
BLADE_TIP_Y = 112e-3
BLADE_L = BLADE_TIP_Y - BLADE_ROOT_Y
PIN_L = 67e-3
PIN_D = 6e-3
PIVOT_TO_TIP = (112-68)*1e-3
SERVO_LABEL_TORQUE = 40*9.80665*0.01  # 40 kgf cm; nameplate, not impact rating.


def thickness(s: float) -> float:
    y_mm = (BLADE_ROOT_Y + s) * 1000
    top_mm = 30.0 + (8.5 - 30.0) * (y_mm - 72.0) / 40.0
    bottom_mm = 27.0 + (5.8 - 27.0) * (y_mm - 70.0) / 42.0
    return (top_mm - bottom_mm) * 1e-3


def blade_fe(width_m: float, n: int, force_n: float = 1.0) -> dict:
    """Linear beam FE with clamped root and downward tip point load."""
    h = BLADE_L / n
    ndof = 2 * (n + 1)
    k = np.zeros((ndof, ndof))
    for i in range(n):
        t = thickness((i + 0.5) * h)
        inertia = width_m * t**3 / 12
        a = E_AL * inertia / h**3
        ke = a * np.array([
            [12, 6*h, -12, 6*h],
            [6*h, 4*h*h, -6*h, 2*h*h],
            [-12, -6*h, 12, -6*h],
            [6*h, 2*h*h, -6*h, 4*h*h],
        ])
        ix = [2*i, 2*i+1, 2*i+2, 2*i+3]
        k[np.ix_(ix, ix)] += ke
    f = np.zeros(ndof)
    f[-2] = force_n
    u = np.zeros(ndof)
    u[2:] = np.linalg.solve(k[2:, 2:], f[2:])
    loc = np.linspace(0, BLADE_L, 1001)
    sigma = np.array([6 * force_n * (BLADE_L-s) /
                      (width_m * thickness(s)**2) for s in loc])
    j = int(np.argmax(sigma))
    peak_per_n = sigma[j] / force_n
    return {
        'mesh_elements': n,
        'width_mm': width_m*1000,
        'tip_displacement_mm_per_N': u[-2]*1000/force_n,
        'tip_stiffness_N_per_m': force_n/u[-2],
        'peak_von_mises_MPa_per_N': peak_per_n/1e6,
        'peak_y_mm': (BLADE_ROOT_Y+loc[j])*1000,
        'first_yield_force_N': Y_AL/peak_per_n,
        'stress_profile_y_mm': ((BLADE_ROOT_Y+loc)*1000).tolist()[::20],
        'stress_profile_MPa_per_N': (sigma/1e6).tolist()[::20],
    }


def impact(m: float, speed: float, ke: float, fy: float,
           hardening_fraction: float = 0.02) -> tuple[dict, list[dict]]:
    """Mass strikes bilinear elastoplastic spring, no contact damping.

    x is blade-tip displacement while contact holds. v/a are the equivalent
    impact mass values, not every component's velocity/acceleration.
    """
    hmod = hardening_fraction * ke
    dt = 2*math.pi*math.sqrt(m/ke)/8000
    x = 0.0
    v = speed
    xp = 0.0
    back = 0.0
    rows = []
    peak_f = peak_x = peak_a = peak_v = peak_xp = 0.0
    peak_t = 0.0
    for i in range(100000):
        t = i*dt
        trial = ke*(x-xp)
        gap = trial-back
        if gap > fy:
            dgamma = (gap-fy)/(ke+hmod)
            xp += dgamma
            back += hmod*dgamma
            trial -= ke*dgamma
        force = max(0.0, trial)
        a = -force/m
        if force > peak_f:
            peak_f, peak_t = force, t
        peak_x = max(peak_x, x)
        peak_a = max(peak_a, abs(a))
        peak_v = max(peak_v, abs(v))
        peak_xp = max(peak_xp, xp)
        if i % 10 == 0:
            rows.append({'time_ms': t*1000, 'displacement_mm': x*1000,
                         'velocity_m_s': v, 'acceleration_m_s2': a,
                         'contact_force_N': force,
                         'plastic_tip_offset_mm': xp*1000})
        if i > 1 and v < 0 and x <= xp and force < fy*1e-5:
            break
        v += a*dt
        x += v*dt
    else:
        raise RuntimeError('impact simulation did not separate')
    e_initial = 0.5*m*speed**2
    e_rebound = 0.5*m*v**2
    return ({
        'mass_kg': m, 'speed_m_s': speed, 'contact_duration_ms': t*1000,
        'peak_contact_force_N': peak_f, 'peak_time_ms': peak_t*1000,
        'maximum_displacement_mm': peak_x*1000,
        'permanent_tip_offset_mm': peak_xp*1000,
        'peak_equivalent_mass_velocity_m_s': peak_v,
        'peak_equivalent_mass_deceleration_m_s2': peak_a,
        'peak_equivalent_mass_deceleration_g': peak_a/9.80665,
        'rebound_speed_m_s': -v,
        'initial_kinetic_energy_J': e_initial,
        'rebound_kinetic_energy_J': e_rebound,
        'dissipated_energy_J': e_initial-e_rebound,
        'elastic_equivalent_root_bending_MPa': peak_f/fy*275,
        'root_stress_model_valid': bool(peak_xp <= 1e-8),
        'plasticity_triggered': bool(peak_xp > 1e-8),
        'small_deflection_valid': bool(peak_x < BLADE_L/10),
    }, rows)


def main() -> None:
    meshes = [blade_fe(0.062, n) for n in (5, 10, 20, 40, 80)]
    blade = meshes[-1]
    strip = blade_fe(0.010, 80)
    convergence = abs(meshes[-1]['tip_displacement_mm_per_N'] /
                      meshes[-2]['tip_displacement_mm_per_N'] - 1)
    assert convergence < 0.002, convergence
    s_quad = np.linspace(0, BLADE_L, 20001)
    i_quad = 0.062 * np.array([thickness(s)**3 for s in s_quad]) / 12
    compliance_analytic = np.trapezoid((BLADE_L-s_quad)**2/(E_AL*i_quad), s_quad)
    static_integral_error = abs(blade['tip_displacement_mm_per_N']/1000 /
                                compliance_analytic - 1)
    assert static_integral_error < 0.0001, static_integral_error
    assert abs(blade['first_yield_force_N'] - strip['first_yield_force_N']*6.2) < 1e-6
    scenarios = []
    for contact, section in [('full_62mm', blade), ('local_10mm', strip)]:
        for speed in (0.5, 1.0, 2.0, 3.0):
            result, rows = impact(1.5, speed,
                                  section['tip_stiffness_N_per_m'],
                                  section['first_yield_force_N'])
            result['contact_width'] = contact
            scenarios.append(result)
            with (OUT / f'time_{contact}_{speed:.1f}mps.csv').open('w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=rows[0].keys())
                w.writeheader(); w.writerows(rows)
    # Pivot screening uses a simply supported central transverse point load.
    inertia = math.pi*PIN_D**4/64
    pin = {
        'support_span_mm': PIN_L*1000, 'diameter_mm': PIN_D*1000,
        'assumed_steel_E_GPa': E_STEEL/1e9,
        'bending_von_mises_MPa_per_N': (8*PIN_L/(math.pi*PIN_D**3))/1e6,
        'midspan_displacement_mm_per_N':
            (PIN_L**3/(48*E_STEEL*inertia))*1000,
        'location_of_peak_stress': 'midspan outer surface',
        'yield_force_for_250MPa_steel_N': 250e6*math.pi*PIN_D**3/(8*PIN_L),
        'steel_yield_is_assumption': True,
    }
    for s in scenarios:
        s['pin_peak_von_mises_MPa_if_entire_force_routes_through_pin'] = (
            s['peak_contact_force_N']*pin['bending_von_mises_MPa_per_N'])
        s['pin_midspan_displacement_mm_if_entire_force_routes_through_pin'] = (
            s['peak_contact_force_N']*pin['midspan_displacement_mm_per_N'])
        s['pivot_torque_Nm_if_tip_force_not_stopped'] = (
            s['peak_contact_force_N']*PIVOT_TO_TIP)
        s['pivot_torque_to_40kgfcm_servo_nameplate_ratio'] = (
            s['pivot_torque_Nm_if_tip_force_not_stopped']/SERVO_LABEL_TORQUE)
    # A separate work-energy envelope for frontal/side collision planning.
    # It does not resolve where the force goes in the chassis.
    crash_envelope = []
    for speed in (0.5, 1.0, 2.0, 3.0):
        for stopping_mm in (2.0, 5.0, 10.0):
            avg_force = 0.5*1.5*speed**2/(stopping_mm/1000)
            crash_envelope.append({
                'equivalent_mass_kg': 1.5,
                'speed_m_s': speed,
                'assumed_stopping_distance_mm': stopping_mm,
                'kinetic_energy_J': 0.5*1.5*speed**2,
                'average_force_N': avg_force,
                'average_deceleration_g': avg_force/(1.5*9.80665),
                'peak_force': 'unknown',
                'load_path': 'unknown: front wedge / side panel / fasteners',
            })
    # Two freely moving bodies of 1.05 and 1.5 kg have a reduced mass near
    # 0.62 kg. This is illustrative because the CAD mass is provisional.
    illustrative_reduced_mass = 1.05*1.5/(1.05+1.5)
    mass_sensitivity = []
    for mass in (0.25, illustrative_reduced_mass, 1.5):
        for contact, section in [('full_62mm', blade), ('local_10mm', strip)]:
            for speed in (0.5, 1.0, 2.0):
                r, _ = impact(mass, speed,
                              section['tip_stiffness_N_per_m'],
                              section['first_yield_force_N'])
                mass_sensitivity.append({
                    'effective_mass_kg': mass,
                    'contact_width': contact,
                    'relative_speed_m_s': speed,
                    'peak_contact_force_N': r['peak_contact_force_N'],
                    'maximum_displacement_mm': r['maximum_displacement_mm'],
                    'permanent_tip_offset_mm': r['permanent_tip_offset_mm'],
                    'small_deflection_valid': r['small_deflection_valid'],
                })
    elastic = next(s for s in scenarios if s['contact_width']=='full_62mm' and s['speed_m_s']==0.5)
    if not elastic['plasticity_triggered']:
        predicted_f = 0.5*math.sqrt(1.5*blade['tip_stiffness_N_per_m'])
        assert abs(elastic['peak_contact_force_N']/predicted_f-1) < 0.01
        assert elastic['dissipated_energy_J'] >= -0.005
    summary = {
        'model_type': 'reduced-order beam finite element plus 1-DOF bilinear impact',
        'not': 'Fusion whole-robot FEA or physical impact certification',
        'cad_source': 'robot2_items_1_3_2026-09-24.f3d; dimensions in AddWedgeLifterOption.py and 2026-09-24 revisions',
        'material_6061': {'E_GPa':E_AL/1e9,'assumed_yield_MPa':Y_AL/1e6},
        'boundary': 'blade rear y=74 mm fully clamped; tip y=112 mm struck vertically; pivot treated separately as simply supported',
        'impact_assumptions': '1.5 kg equivalent mass, no contact damping, rigid opponent, no chassis/fastener/servo compliance; 2% spring hardening',
        'blade_mesh_convergence_tip_deflection_40_to_80_fraction': convergence,
        'blade_fe_to_independent_integral_compliance_error_fraction': static_integral_error,
        'full_width_elastic_first_yield_speed_m_s':
            blade['first_yield_force_N']/math.sqrt(1.5*blade['tip_stiffness_N_per_m']),
        'local_10mm_elastic_first_yield_speed_m_s':
            strip['first_yield_force_N']/math.sqrt(1.5*strip['tip_stiffness_N_per_m']),
        'blade_meshes': [{k:v for k,v in m.items() if not k.startswith('stress_profile')} for m in meshes],
        'blade_full_width': blade,
        'blade_local_10mm': strip,
        'pin': pin,
        'servo_nominal_40kgfcm_torque_Nm': SERVO_LABEL_TORQUE,
        'pivot_to_tip_lever_mm': PIVOT_TO_TIP*1000,
        'scenarios': scenarios,
        'front_side_work_energy_envelope': crash_envelope,
        'illustrative_two_free_bodies_reduced_mass_kg': illustrative_reduced_mass,
        'mass_sensitivity': mass_sensitivity,
        'limits': [
            'Bilinear spring plastic offset is only a proxy for permanent blade deformation; no 3D nonlinear material solve.',
            'Large deflection results marked invalid; buckling, fracture and contact change are omitted.',
            'Peak pin load transfer through pivot is an intentionally conservative hypothetical load path.',
            'Steel grade, fasteners, joints, printed material and actual contact patch are unverified.',
            'The model cannot establish force/speed that destroys the entire robot.',
        ],
    }
    (OUT / 'results.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False)+'\n')
    with (OUT / 'scenario_summary.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=scenarios[0].keys())
        w.writeheader(); w.writerows(scenarios)
    with (OUT / 'front_side_load_envelope.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=crash_envelope[0].keys())
        w.writeheader(); w.writerows(crash_envelope)
    with (OUT / 'mass_sensitivity.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=mass_sensitivity[0].keys())
        w.writeheader(); w.writerows(mass_sensitivity)
    print(json.dumps({
        'convergence':convergence,
        'blade_k_N_per_m':blade['tip_stiffness_N_per_m'],
        'blade_yield_N':blade['first_yield_force_N'],
        'peak_location_y_mm':blade['peak_y_mm'],
        'pin_yield_250MPa_N':pin['yield_force_for_250MPa_steel_N'],
        'scenarios':scenarios}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
