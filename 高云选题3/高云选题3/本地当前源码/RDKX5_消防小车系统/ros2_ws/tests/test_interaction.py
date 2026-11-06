import pathlib,sys,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.interaction import InteractionRouter,scene_sentence

class TestInteraction(unittest.TestCase):
    def make(self):return InteractionRouter({'buildings':[{'building_id':'C'}]})
    def command(self,action='navigate',**kw):return dict(command_id='test_1',source='test',action=action,issued_at=10.,ttl_s=5.,building_id='C',**kw)
    def test_navigate_is_preview_until_enabled_and_deduplicated(self):
        router=self.make();r=router.route(self.command(),10.1)
        self.assertEqual(r['navigation']['mode'],'preview');self.assertTrue(r['navigation']['confirmed'])
        self.assertFalse(router.route(self.command(),10.2)['accepted'])
    def test_pause_resume_preserves_nav_ownership(self):
        router=self.make();router.route(self.command(),10.1)
        stop=self.command('stop');stop['command_id']='stop';s=router.route(stop,10.2)
        self.assertEqual(s['mission']['mode'],'idle');self.assertEqual(s['navigation']['action'],'pause')
        resume=self.command('resume');resume['command_id']='resume';r=router.route(resume,10.3)
        self.assertEqual(r['mission']['mode'],'navigate')
    def test_reject_expired_or_unknown(self):
        with self.assertRaises(ValueError):self.make().route(self.command(),20)
        c=self.command();c['building_id']='ZZ'
        with self.assertRaises(ValueError):self.make().route(c,10.1)
    def test_report_does_not_claim_casualties_or_stale_fire(self):
        r=dict(valid=True,stamp=10.,building_number='03',building_name='春晖楼',fire_status='observed',visible_person_count=2,visible_firefighter_count=1)
        self.assertIn('是否受困尚不能确定',scene_sentence(r,10.1))
        self.assertIn('过期',scene_sentence(r,14.))

if __name__=='__main__':unittest.main(verbosity=2)
