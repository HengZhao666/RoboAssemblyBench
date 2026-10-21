import copy
import os
import unittest
from unittest.mock import patch
from roboassemblybench.core.beam_physical_compiler import _apply_column3_transport_raise


class Column3RaisedTransport(unittest.TestCase):
    def targets(self):
        return [{'name': name, 'position': [0.4, -0.2, .14], 'orientation': [1,0,0,0]}
                for name in ['part_3_pickup_clearance','part_3_assembly_clearance',
                             'part_3_preinsert_hover','part_3_insert_00',
                             'part_6_pickup_clearance','part_2_assembly_clearance']]

    def test_unset_and_zero_preserve_entire_target_contract(self):
        for env in [{}, {'BEAM_COLUMN3_TRANSPORT_RAISE_M':'0'}]:
            with patch.dict(os.environ,env,clear=True):
                t=self.targets();before=copy.deepcopy(t);_apply_column3_transport_raise(t)
                self.assertEqual(t,before)

    def test_only_two_column_clearances_rise_and_hover_stays_original(self):
        with patch.dict(os.environ,{'BEAM_COLUMN3_TRANSPORT_RAISE_M':'.16'}):
            t=self.targets();before=copy.deepcopy(t);_apply_column3_transport_raise(t)
            for i in range(2):
                self.assertEqual(t[i]['position'][:2],before[i]['position'][:2])
                self.assertAlmostEqual(t[i]['position'][2]-before[i]['position'][2],.16)
                self.assertEqual(t[i]['orientation'],before[i]['orientation'])
            self.assertEqual(t[2:],before[2:])

    def test_invalid_values_do_not_modify_any_target(self):
        for value in ['nan','inf','-.01','.24001','bad']:
            with self.subTest(value=value),patch.dict(os.environ,{'BEAM_COLUMN3_TRANSPORT_RAISE_M':value}):
                t=self.targets();before=copy.deepcopy(t)
                with self.assertRaises(ValueError):_apply_column3_transport_raise(t)
                self.assertEqual(t,before)

    def test_missing_duplicate_or_nonfinite_targets_fail_before_mutation(self):
        cases=[self.targets()[1:], self.targets()+[self.targets()[0]]]
        bad=self.targets();bad[1]['position'][0]=float('nan');cases.append(bad)
        with patch.dict(os.environ,{'BEAM_COLUMN3_TRANSPORT_RAISE_M':'.16'}):
            for targets in cases:
                original=copy.deepcopy(targets)
                with self.assertRaises(ValueError):_apply_column3_transport_raise(targets)
                self.assertEqual([t['position'][2] for t in targets],[t['position'][2] for t in original])


if __name__=='__main__':unittest.main()
