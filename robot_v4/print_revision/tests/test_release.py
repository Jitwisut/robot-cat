"""Release gates must fail closed for real production mistakes."""
import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
from checks import release_blockers
from inputs import validate_inputs
from revision import value


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.base=Path(self.temp.name)
        (self.base/'proof.txt').write_text('Synthetic unit-test evidence only')
        self.config=json.loads((HERE/'inputs.json').read_text())
        for item in self.config['hardware'].values():
            item['actual']=copy.deepcopy(item['nominal']);item['verified']=True;item['evidence']='proof.txt'
        fit=self.config['fit'];fit.update({'d_bore_diametral_clearance_mm':0.1,'d_bore_flat_clearance_mm':0.05,'hinge_diametral_clearance_mm':0.2,'insert_hole_diameter_mm':4,'tyre_inner_diameter_mm':43.8,'nut_pocket_clearance_mm':0.2})
        for name in ('fit','assembly'):
            item=self.config[name];item.update(verified=True,sha256_build='current',evidence='proof.txt')
            item['results']={k:True for k in item['results']}
        self.config['shop'].update(name='test',printer='test',usable_bed_mm=[250,250,250],nozzle_mm=0.4,slicer='test',brim_mm=5,profile_evidence='proof.txt',materials={'PETG':'test','TPU95A':'test'})
        self.config['slicer'].update(verified=True,sha256_build='current',project_file='proof.txt',report_file='proof.txt',printed_mass_g_by_file={'tub.stl':400})
        self.config['slicer']['review']={k:True for k in self.config['slicer']['review']}
        cost=self.config['cost'];cost.update(verified=True,evidence='proof.txt')
        for row in cost['items']:row['total_thb']=100
        self.report={'build_sha256':'current','manifest':[{'file':'tub.stl','dimensions_mm':[190.5,217.3,54]}],'cad':{'passed':True},'mesh':[{'passed':True}],'trial_mesh':[{'passed':True}],'mass':{'rear_load_passed':True,'mass_limit_passed':True},'simulation':{'passed':True,'build_sha256':'current'}}

    def tearDown(self):self.temp.cleanup()
    def blockers(self):return release_blockers(self.config,self.report,self.base)

    def test_complete_evidence_passes(self):
        validate_inputs(self.config);self.assertEqual(self.blockers(),[])

    def test_missing_actual_dimension_blocks(self):
        self.config['hardware']['drive_Front_L']['actual']['shaft_projection_mm']=None
        self.assertTrue(any('drive_Front_L' in x for x in self.blockers()))

    def test_nonexistent_evidence_blocks(self):
        self.config['fit']['evidence']='missing.png'
        self.assertTrue(any('fit' in x for x in self.blockers()))

    def test_stale_trial_or_simulation_blocks(self):
        self.config['fit']['sha256_build']='previous'
        self.report['simulation']['build_sha256']='previous'
        self.assertTrue(any('fit' in x for x in self.blockers()))
        self.assertTrue(any('dynamics' in x for x in self.blockers()))

    def test_trial_flags_cannot_replace_calibration(self):
        self.config['fit']['tyre_inner_diameter_mm']=None
        self.assertIn('Calibrated fit dimensions missing',self.blockers())

    def test_brim_outside_220_bed_blocks(self):
        self.config['shop']['usable_bed_mm']=[220,220,250]
        self.assertTrue(any('exceeds printer' in x for x in self.blockers()))

    def test_cost_limit_and_missing_cost(self):
        self.config['cost']['items'][0]['total_thb']=6001
        self.assertIn('Project cost exceeds 6000 THB',self.blockers())
        self.config['cost']['items'][0]['total_thb']=None
        self.assertIn('Complete confirmed project cost missing',self.blockers())

    def test_false_physical_check_blocks(self):
        self.config['assembly']['results']['fasteners_reachable']=False
        self.assertTrue(any('assembly' in x for x in self.blockers()))

    def test_failed_geometry_or_weight_blocks(self):
        self.report['cad']['passed']=False;self.report['mass']['mass_limit_passed']=False
        self.assertTrue(any('CAD' in x for x in self.blockers()))
        self.assertTrue(any('2000' in x for x in self.blockers()))

    def test_invalid_dimensions_rejected(self):
        self.config['hardware']['battery']['actual']['dimensions_mm']=[75,35,-25]
        with self.assertRaises(ValueError):validate_inputs(self.config)

    def test_nan_or_relaxed_acceptance_rejected(self):
        self.config['hardware']['drive_Front_L']['actual']['mass_g']=float('nan')
        with self.assertRaises(ValueError):validate_inputs(self.config)
        self.config['hardware']['drive_Front_L']['actual']['mass_g']=110
        self.config['design']['hinge_min_wall_mm']=0.4
        with self.assertRaises(ValueError):validate_inputs(self.config)

    def test_missing_slicer_part_mass_blocks(self):
        self.config['slicer']['printed_mass_g_by_file']={}
        self.assertTrue(any('Slicer' in x for x in self.blockers()))

    def test_catalog_size_is_used_but_cannot_release_unmeasured_part(self):
        item=self.config['hardware']['esc']
        item['actual']={key:None for key in item['actual']}
        item['catalog']={'source_id':'seller','url':'https://example.com/esc','values':{'dimensions_mm':[68,25,8]}}
        validate_inputs(self.config)
        self.assertEqual(value(self.config,'esc','dimensions_mm'),[68,25,8])
        self.assertIn('Unconfirmed hardware: esc',self.blockers())

    def test_real_measurement_overrides_catalog(self):
        item=self.config['hardware']['esc']
        item['catalog']={'source_id':'seller','url':'https://example.com/esc','values':{'dimensions_mm':[68,25,8]}}
        self.assertEqual(value(self.config,'esc','dimensions_mm'),item['actual']['dimensions_mm'])

    def test_catalog_dimension_is_validated(self):
        item=self.config['hardware']['esc'];item['actual']['dimensions_mm']=None
        item['catalog']={'source_id':'seller','url':'https://example.com/esc','values':{'dimensions_mm':[68,25,-8]}}
        with self.assertRaises(ValueError):validate_inputs(self.config)


if __name__=='__main__':unittest.main()
