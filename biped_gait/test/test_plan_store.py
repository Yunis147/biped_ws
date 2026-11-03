"""Cache integrity and model/settings invalidation; never opens a robot bus."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from test_feedback import plans
from biped_gait.feedback_walk import Plans
from biped_gait import plan_store as store


class PlanStoreTests(unittest.TestCase):
    def test_node_paths_load_without_invoking_the_numerical_solver(self):
        with patch('biped_gait.quasi_static_walk.build_tables',side_effect=AssertionError('solver ran')):
            loaded=Plans(precomputed=True)
        self.assertEqual(set(loaded.cache),{'forward','backward'})

    def test_saved_path_round_trip_and_corruption_detection(self):
        settings=plans().settings;plan=plans().cache['forward']
        with tempfile.TemporaryDirectory() as directory:
            file=store.save(plan,settings,'forward',directory)
            loaded=store.load(settings,'forward',plan.initial,directory)
            for seg in plan.tables: np.testing.assert_array_equal(plan.tables[seg],loaded.tables[seg])
            with np.load(file,allow_pickle=False) as data: values={k:data[k] for k in data.files}
            values['tables']=values['tables'].copy();values['tables'][0,1,0] += .01
            np.savez_compressed(file,**values)
            with self.assertRaisesRegex(ValueError,'checksum'):
                store.load(settings,'forward',plan.initial,directory)

    def test_changed_model_or_profile_does_not_reuse_the_old_table(self):
        s=plans().settings;q=np.zeros(12)
        with tempfile.TemporaryDirectory() as directory:
            store.save(plans().cache['forward'],s,'forward',directory)
            with self.assertRaisesRegex(ValueError,'No precomputed path'):
                store.load(replace(s,body_roll=s.body_roll+.001),'forward',q,directory)
            old=store.m.BASE['mass']
            with patch.dict(store.m.BASE,mass=old+.001):
                with self.assertRaisesRegex(ValueError,'No precomputed path'):
                    store.load(s,'forward',q,directory)
