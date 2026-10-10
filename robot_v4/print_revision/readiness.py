"""Evidence gates for printing and subsequent physical assembly, without CAD imports."""
import argparse
import hashlib
import json
import math
from pathlib import Path

FIT_RESULTS = {'shaft_slides', 'hub_locks_rotation', 'hub_locks_axial', 'tyre_no_slip',
               'hinge_free', 'wedgelet_secure', 'insert_no_rotation', 'lid_secure'}
PREFLIGHT_RESULTS = {'fastener_lengths_checked', 'tool_access_reviewed',
                     'metal_interfaces_checked', 'battery_straps_checked',
                     'wire_routes_reviewed', 'disconnect_access_reviewed',
                     'assembly_instructions_complete'}
ASSEMBLY_RESULTS = {'fasteners_reachable', 'battery_retained', 'boards_insulated',
                    'wires_clear', 'disconnect_accessible', 'lid_accessible',
                    'no_undocumented_drilling', 'machined_hub_bearings_retained',
                    'shaft_clamp_free_rotation'}
ELECTRICAL_RESULTS = {'drive_current_and_limit_checked', 'weapon_esc_checked',
                      'battery_voltage_capacity_checked', 'connectors_fuse_wire_checked'}
SLICER_RESULTS = {'bed_with_brim', 'thin_features_present', 'support_removable',
                 'critical_holes_clear', 'materials_correct'}
COST_IDS = {'drive_motors', 'weapon_motor', 'esc', 'battery', 'drive_drivers', 'sensors',
            'controller', 'gamepad', 'charger', 'disconnect_fuse', 'wiring_connectors',
            'bearings', 'belt', 'metal_cutting', 'machining', 'fasteners_inserts',
            'hinge_pins_clips', 'skid', 'measurement_fit_service', 'trial_prints',
            'full_prints', 'shipping', 'contingency'}


def proof(path, base):
    if not isinstance(path, str) or not path.strip():
        return False
    candidate = Path(path)
    candidate = candidate if candidate.is_absolute() else Path(base) / candidate
    return candidate.is_file() and candidate.stat().st_size > 0


def positive(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value) and value > 0


def profile_signature(shop, base):
    """Bind physical fit and slicing to the actual saved settings, not a profile name."""
    if not proof(shop.get('profile_evidence'), base):
        return None
    path = Path(shop['profile_evidence'])
    path = path if path.is_absolute() else Path(base) / path
    fields = {k: shop.get(k) for k in ('name', 'printer', 'usable_bed_mm', 'nozzle_mm',
                                     'slicer', 'materials', 'brim_mm')}
    fields['profile_file_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return hashlib.sha256(json.dumps(fields, sort_keys=True).encode()).hexdigest()


def complete(item, required, signature, base):
    results = item.get('results', {})
    return (item.get('verified') is True and item.get('sha256_build') == signature
            and proof(item.get('evidence'), base)
            and required <= set(results) and all(results[key] is True for key in required))


def additional_print_blockers(config, report, base):
    blockers = []
    signature = report['build_sha256']
    profile = profile_signature(config['shop'], base)
    for key, required in [('fit', FIT_RESULTS), ('preflight', PREFLIGHT_RESULTS),
                          ('electrical', ELECTRICAL_RESULTS)]:
        if not complete(config.get(key, {}), required, signature, base):
            blockers.append('Incomplete current-build evidence: ' + key)
    if not profile or any(config[key].get('profile_sha256') != profile for key in ('fit', 'slicer')):
        blockers.append('Physical fit/slicer printer-material-profile evidence missing or changed')
    fit = config['fit']
    shafts = {'drive_Front_L','drive_Front_R','drive_Rear_L','drive_Rear_R'}
    if not (proof(fit.get('round1_evidence'),base) and proof(fit.get('round2_evidence'),base)
            and fit.get('round2_sha256_build') == signature and shafts <= set(fit.get('tested_drive_shafts',[]))
            and isinstance(fit.get('lid_cycles'),int) and not isinstance(fit.get('lid_cycles'),bool) and fit['lid_cycles'] >= 5):
        blockers.append('Two-round fit evidence, all four shafts and five lid cycles required')
    review = config['slicer'].get('review', {})
    if not SLICER_RESULTS <= set(review) or not all(review[k] is True for k in SLICER_RESULTS):
        blockers.append('Required slicer review checks missing/failed')
    specs = config['electrical']['specs']
    if specs.get('battery_cells') != 3 or not positive(specs.get('battery_capacity_mah')) or specs['battery_capacity_mah'] < 850:
        blockers.append('Battery must remain 3S and at least 850 mAh')
    required_specs = ('drive_pair_start_current_a', 'drive_pair_stall_current_a',
                      'driver_continuous_current_a', 'driver_current_limit_a',
                      'weapon_kv', 'weapon_spinup_current_a', 'esc_continuous_current_a',
                      'power_path_continuous_current_a', 'test_voltage_v', 'weapon_resistance_ohm',
                      'weapon_no_load_current_a', 'drive_rated_rpm', 'drive_rated_voltage_v',
                      'drive_stall_torque_nm')
    if any(not positive(specs.get(k)) for k in required_specs):
        blockers.append('Electrical ratings/current requirements incomplete')
    else:
        if specs['test_voltage_v'] < 12.6:
            blockers.append('Electrical validation must include fully charged 3S voltage 12.6 V')
        if specs['driver_current_limit_a'] > specs['driver_continuous_current_a']:
            blockers.append('Drive current limit exceeds confirmed continuous driver rating')
        if specs['drive_pair_start_current_a'] > specs['driver_current_limit_a']:
            blockers.append('Drive starting current exceeds validated driver current limit')
        if specs['weapon_spinup_current_a'] > specs['esc_continuous_current_a']:
            blockers.append('Weapon spin-up current exceeds confirmed ESC rating')
        if specs['weapon_spinup_current_a'] + 2 * specs['driver_current_limit_a'] > specs['power_path_continuous_current_a']:
            blockers.append('Combined current exceeds confirmed battery/connector/wire/fuse path')
    items = config['cost']['items']
    ids = [item['id'] for item in items]
    if not COST_IDS <= set(ids) or len(ids) != len(set(ids)):
        blockers.append('Project cost categories missing or duplicated')
    reserve = next((item['total_thb'] for item in items if item['id'] == 'contingency'), None)
    if not positive(reserve) or reserve < 300:
        blockers.append('Budget must include at least 300 THB contingency')
    manufactured=config.get('manufacturing',{})
    unmeasured=[row['part'] for row in report['mass'].get('parts',[]) if row['method']=='estimated CAD volume/density' and row.get('material') in ('steel','6061','foam')]
    if unmeasured or manufactured.get('verified') is not True or not proof(manufactured.get('evidence'),base):
        blockers.append('Manufactured metal/foam part masses not physically confirmed: '+', '.join(unmeasured))
    mass = report['mass']
    target_missed = (mass['total_g'] > config['design']['mass_target_g'] or
                     mass['rear_static_load_fraction'] < config['design']['rear_load_target'])
    decision = config.get('target_decision', {})
    if target_missed and not (decision.get('accepted_by_user') is True
                             and decision.get('sha256_build') == signature
                             and proof(decision.get('evidence'), base)):
        blockers.append('Mass/rear-load targets missed; current-build user decision required')
    return blockers


def assembly_blockers(config, report, base):
    """Full assembly acceptance requires measured robot mass/load after print release."""
    signature = report['build_sha256']
    blockers = []
    item = config['assembly']
    if not complete(item, ASSEMBLY_RESULTS, signature, base):
        blockers.append('Physical full-robot assembly checks missing/current build not verified')
    weight, rear = item.get('measured_total_mass_g'), item.get('measured_rear_load_fraction')
    if not positive(weight) or weight > config['design']['mass_limit_g']:
        blockers.append('Measured assembled robot mass missing or exceeds 2000 g')
    if not positive(rear) or not 0.10 <= rear <= 1:
        blockers.append('Measured rear wheel load missing or below 10%')
    if report.get('status') != 'PRINT_READY':
        blockers.append('Verified print release required before assembly acceptance')
    return blockers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--release-report', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.inputs.read_text())
    report = json.loads(args.release_report.read_text())
    from checks import build_signature
    blockers = assembly_blockers(config, report, args.inputs.parent)
    if build_signature(config) != report['build_sha256']:
        blockers.append('CAD inputs/source changed since released build')
    print(json.dumps({'status': 'ASSEMBLED_CHECKED' if not blockers else 'ASSEMBLY_PENDING',
                      'blockers': blockers, 'build_sha256': report['build_sha256']}, indent=2))
    return 2 if blockers else 0


if __name__ == '__main__':
    raise SystemExit(main())
