"""R3 release separation and evidence invalidation; fixtures never ship as evidence."""
import copy
import json
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import test_release
HERE=test_release.HERE
from checks import release_blockers,input_signature
from readiness import profile_signature,assembly_blockers,ELECTRICAL_RESULTS,PREFLIGHT_RESULTS
from inputs import validate_inputs
from coupons import coupons
from procurement import evaluate

class R3ReadinessTests(test_release.ReleaseTests):
    def setUp(self):
        super().setUp()
        self.config['readiness_version']=2
        self.config['manufacturing']={'verified':True,'evidence':'proof.txt','actual_mass_g_by_body':{}}
        for name,keys in [('electrical',ELECTRICAL_RESULTS),('preflight',PREFLIGHT_RESULTS)]:
            self.config[name]={'verified':True,'evidence':'proof.txt','sha256_build':'current','results':dict.fromkeys(keys,True)}
        self.config['electrical']['specs']={'battery_cells':3,'battery_capacity_mah':850,'test_voltage_v':12.6,'weapon_kv':1250,'weapon_resistance_ohm':.05,'weapon_no_load_current_a':1.5,'drive_rated_rpm':400,'drive_rated_voltage_v':12,'drive_stall_torque_nm':.12,'drive_pair_start_current_a':2,'drive_pair_stall_current_a':4,'driver_continuous_current_a':3,'driver_current_limit_a':2.5,'weapon_spinup_current_a':20,'esc_continuous_current_a':40,'power_path_continuous_current_a':30}
        self.config['cost']['items'] += [{'id':k,'total_thb':100,'quantity':1} for k in ('gamepad','measurement_fit_service')]
        next(x for x in self.config['cost']['items'] if x['id']=='contingency')['total_thb']=300
        self.report['mass'].update(total_g=1850,rear_static_load_fraction=.16)
        profile=profile_signature(self.config['shop'],self.base)
        self.config['fit'].update(profile_sha256=profile,round1_evidence='proof.txt',round2_evidence='proof.txt',round2_sha256_build='current',lid_cycles=5,tested_drive_shafts=['drive_Front_L','drive_Front_R','drive_Rear_L','drive_Rear_R'])
        self.config['slicer']['profile_sha256']=profile

    def test_false_physical_check_blocks(self):
        # Print approval needs preflight; full robot does not exist yet.
        self.config['preflight']['results']['tool_access_reviewed']=False
        self.assertTrue(any('preflight' in x for x in self.blockers()))

    def test_print_ready_without_full_robot_but_not_assembly_ready(self):
        self.config['assembly'].update(verified=False,evidence=None)
        self.config['assembly']['results']=dict.fromkeys(self.config['assembly']['results'])
        self.assertEqual(self.blockers(),[])
        self.report['status']='PRINT_READY'
        self.assertTrue(assembly_blockers(self.config,self.report,self.base))

    def test_assembly_measured_mass_load_and_print_release_required(self):
        self.report['status']='PRINT_READY'
        self.config['assembly'].update(measured_total_mass_g=1900,measured_rear_load_fraction=.11)
        self.assertEqual(assembly_blockers(self.config,self.report,self.base),[])
        self.config['assembly']['measured_rear_load_fraction']=.09
        self.assertTrue(assembly_blockers(self.config,self.report,self.base))
        self.config['assembly']['measured_rear_load_fraction']=.11
        self.report['status']='DRAFT_NOT_RELEASED'
        self.assertTrue(assembly_blockers(self.config,self.report,self.base))

    def test_profile_file_or_material_change_invalidates_trial(self):
        (self.base/'proof.txt').write_text('changed print settings')
        self.assertTrue(any('profile' in x for x in self.blockers()))
        self.config['shop']['materials']['PETG']='other brand'
        self.assertTrue(any('profile' in x for x in self.blockers()))

    def test_two_rounds_all_shafts_and_five_cycles(self):
        for field,bad in [('round1_evidence',None),('round2_sha256_build','old'),('lid_cycles',4),('tested_drive_shafts',['drive_Front_L'])]:
            with self.subTest(field=field):
                config=copy.deepcopy(self.config);config['fit'][field]=bad
                self.assertTrue(any('Two-round' in x for x in release_blockers(config,self.report,self.base)))

    def test_missed_target_requires_decision_and_never_waives_hard_limit(self):
        self.report['mass']['total_g']=1970
        self.assertTrue(any('user decision' in x for x in self.blockers()))
        self.config['target_decision']={'accepted_by_user':True,'sha256_build':'current','evidence':'proof.txt'}
        self.assertEqual(self.blockers(),[])
        self.report['mass']['mass_limit_passed']=False
        self.assertTrue(any('2000' in x for x in self.blockers()))

    def test_current_battery_missing_categories_and_reserve(self):
        specs=self.config['electrical']['specs'];specs['battery_capacity_mah']=650;specs['driver_current_limit_a']=4
        self.assertTrue(any('Battery' in x for x in self.blockers()))
        self.assertTrue(any('continuous driver' in x for x in self.blockers()))
        self.config['cost']['items']=[x for x in self.config['cost']['items'] if x['id']!='gamepad']
        next(x for x in self.config['cost']['items'] if x['id']=='contingency')['total_thb']=299
        self.assertTrue(any('categories' in x for x in self.blockers()))
        self.assertTrue(any('contingency' in x for x in self.blockers()))

    def test_evidence_flags_do_not_change_geometry_but_electrical_model_does(self):
        before=input_signature(self.config)
        self.config['fit'].update(profile_sha256='updated',round2_sha256_build='new-proof',round1_evidence='new.png',lid_cycles=6);self.config['preflight']['verified']=False
        self.assertEqual(input_signature(self.config),before)
        self.config['electrical']['specs']['weapon_kv']=1300
        self.assertNotEqual(input_signature(self.config),before)

    def test_r3_schema_rejects_missing_checks(self):
        del self.config['preflight']['results']['tool_access_reviewed']
        with self.assertRaises(ValueError):validate_inputs(self.config)

    def test_new_tyre_coupon_step_and_unequal_shafts(self):
        self.config['hardware']['drive_Rear_R']['actual']['shaft_diameter_mm']=4.1
        names=[r[0] for r in coupons(self.config)]
        self.assertEqual(sum(n.startswith('Tyre_Ring') for n in names),7)
        self.assertEqual(sum(n.startswith('D_Shaft') for n in names),8)

    def test_unmeasured_metal_mass_blocks(self):
        self.report['mass']['parts']=[{'part':'Beater_Disc_L','material':'steel','method':'estimated CAD volume/density'}]
        self.assertTrue(any('metal/foam' in x for x in self.blockers()))

    def test_budget_unknown_is_missing_and_reserve_reduces_ceiling(self):
        data={'date':'test','budget_thb':6000,'reserve_thb':300,'items':[{'id':'known','required_quantity':14,'units_per_pack':10,'source_id':'s'},{'id':'unknown','required_quantity':1,'units_per_pack':1,'source_id':None}]}
        report=evaluate(data,{'records':[{'id':'s','unit_price_thb':20}]})
        self.assertEqual(report['known_catalog_subtotal_thb'],40)
        self.assertEqual(report['remaining_budget_ceiling_thb'],5660)
        self.assertEqual(report['unpriced_items'],['unknown'])
        self.assertFalse(report['project_cost_complete'])

if __name__=='__main__':unittest.main()
