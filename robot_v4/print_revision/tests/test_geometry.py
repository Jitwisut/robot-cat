"""Regression checks for unit/axis conversion and defective mesh rejection."""
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
import trimesh

HERE=Path(__file__).resolve().parents[1];sys.path.insert(0,str(HERE))
from legacy_geometry import build_baseline
from cad_backend import find_body
from checks import mesh_audit


class MeshTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.path=Path(self.temp.name)/'test.stl'
        self.mesh=trimesh.creation.box(extents=[10,10,10]);self.mesh.apply_translation([5,5,5])
    def tearDown(self):self.temp.cleanup()
    def audit(self):self.mesh.export(self.path);return mesh_audit(self.path)
    def test_good_mesh_passes(self):self.assertTrue(self.audit()['passed'])
    def test_open_mesh_rejected(self):
        self.mesh.update_faces(np.arange(len(self.mesh.faces)-1))
        result=self.audit();self.assertFalse(result['passed']);self.assertGreater(result['boundary_edges'],0)
    def test_inverted_mesh_rejected(self):
        self.mesh.invert();self.assertFalse(self.audit()['passed'])
    def test_duplicate_mesh_face_rejected(self):
        self.mesh.faces=np.vstack((self.mesh.faces,self.mesh.faces[:1]))
        self.assertFalse(self.audit()['passed'])


class BaselinePortTests(unittest.TestCase):
    def test_archived_stl_dimensions_and_volume_preserved(self):
        root=build_baseline()
        pairs={'V4_Tub_TPU95A_x1.stl':'Tub_TPU','V4_Lid_TPU95A_x1.stl':'Lid_TPU_2mm','V4_Hall_Post_PETG_x1.stl':'Hall_Post_PETG','V4_Wheel_Hub_PETG_x4.stl':'Wheel_Hub_PETG_Front_L','V4_Wheel_Tyre_TPU95A_x4.stl':'Wheel_Tyre_TPU_Front_L','V4_Wedgelet_Hinge_Block_PETG_L_x1.stl':'Wedgelet_Block_PETG_L','V4_Wedgelet_Hinge_Block_PETG_R_x1.stl':'Wedgelet_Block_PETG_R','V4_Wedgelet_Carrier_Inner_PETG_x2_mirror.stl':'Wedgelet_Carrier_PETG_R1','V4_Wedgelet_Carrier_Outer_PETG_x2_mirror.stl':'Wedgelet_Carrier_PETG_R2'}
        self.assertEqual(len(root.bodies()),58)
        for filename,name in pairs.items():
            with self.subTest(part=name):
                old=trimesh.load_mesh(HERE.parent/'manufacturing'/filename,process=False)
                shape=find_body(root,name).shape;bb=shape.BoundingBox()
                old_dim=old.extents[[0,2,1]]
                np.testing.assert_allclose([bb.xlen,bb.ylen,bb.zlen],old_dim,atol=0.025)
                self.assertLess(abs(shape.Volume()-old.volume)/shape.Volume(),0.001)
                self.assertTrue(shape.isValid())


if __name__=='__main__':unittest.main()
