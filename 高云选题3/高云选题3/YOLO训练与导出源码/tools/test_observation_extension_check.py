"""V8 must freeze the complete V7 baseline and reject reused training identity."""
import copy,json,unittest
from test_people_extension_check import PeopleExtensionChecks
from validate_merged_dataset import sha

class ObservationExtensionChecks(unittest.TestCase):
    def setUp(self):
        self.fixture=PeopleExtensionChecks(methodName='test_valid_extension')
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        f=self.fixture
        baseline=json.loads((f.base/'manifest.json').read_text())
        baseline['sources']['v7']=copy.deepcopy(f.manifest['sources']['v7'])
        (f.base/'manifest.json').write_text(json.dumps(baseline))
        f.manifest['sources']['v8']={'dataset':'../source_data/v8/packaged'}
        f.manifest['baseline_manifest_sha256']=sha(f.base/'manifest.json')
        f.sources['v8']={'v8_new':dict(sample_id='v8_new',pair_id='v8_pair',pose_group='v8_pose',group_id='cell0',split='train',capture_plan={})}

    def test_new_v8_keeps_old_baseline(self):
        errors,report=self.fixture.check()
        self.assertEqual(errors,[]);self.assertEqual(report['extension_version'],'v8')

    def test_v7_capture_identity_cannot_be_reused(self):
        f=self.fixture;f.sources['v8']['v8_new']['pose_group']='v7_pose0'
        self.assertTrue(any('old pose group' in e for e in f.check()[0]))

    def test_v8_cannot_enter_validation_tile(self):
        f=self.fixture;f.sources['v8']['v8_new'].update(group_id='cell1',split='val')
        errors=f.check()[0]
        self.assertTrue(any('not train-only' in e for e in errors))
        self.assertTrue(any('not an original train group' in e for e in errors))

    def test_v7_provenance_frozen(self):
        self.fixture.manifest['sources']['v7']['manifest_sha256']='tampered'
        self.assertTrue(any('Original source provenance changed' in e for e in self.fixture.check()[0]))

if __name__=='__main__':unittest.main()
