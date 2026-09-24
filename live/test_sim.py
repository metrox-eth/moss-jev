"""Offline integration tests; policy doubles are never used by the application."""
import json, threading, unittest
from unittest.mock import patch
import numpy as np
from physics import Physics
from server import Session
from policy import Jev,PolicyError

class PhysicsTests(unittest.TestCase):
    def test_contact_pick_and_drop(self):
        for scenario in ('can','offset'):
            with self.subTest(scenario=scenario):
                p=Physics();p.reset(scenario)
                for action in ('approach','align','close','lift'):p.finish(action)
                self.assertTrue(p.held());self.assertGreater(p.obj()[2],.09)
                for action in ('clear','carry','release'):p.finish(action)
                self.assertTrue(p.observe()['settled_in_bin']);self.assertEqual(p.contacts(),0)
                self.assertEqual(p.m.neq,0, 'No weld constraint may hold the object')
                self.assertTrue(np.all(p.d.qpos[p.qa] >= p.m.jnt_range[p.jids,0]-.02))
                self.assertTrue(np.all(p.d.qpos[p.qa] <= p.m.jnt_range[p.jids,1]+.02))

    def test_missed_grasp_then_recovery(self):
        p=Physics();p.reset('miss')
        for a in ('approach','align','close','lift'):p.finish(a)
        self.assertFalse(p.held());self.assertEqual(p.failures,1);self.assertLess(p.obj()[2],.065)
        self.assertTrue(p.needs_regrasp);p.finish('retry');self.assertFalse(p.needs_regrasp)
        for a in ('approach','align','close','lift','clear','carry','release'):p.finish(a)
        self.assertTrue(p.observe()['settled_in_bin']);self.assertEqual(p.attempts,2)

    def test_no_attachment_and_gravity(self):
        p=Physics()
        for a in ('approach','align','close','lift'):p.finish(a)
        self.assertTrue(p.held());z=p.obj()[2]
        p.finish('release')
        self.assertLess(p.obj()[2],z-.04);self.assertFalse(p.in_bin())

    def test_rover_repositions_and_collects(self):
        p=Physics();p.reset('far');before=p.obj().copy()
        self.assertFalse(p.observe()['within_work_area'])
        p.command('reposition');self.assertTrue(p.observe()['reposition_notice'])
        while p.plan:p.step()
        self.assertGreater(p.base(),.3);self.assertTrue(p.observe()['within_work_area'])
        np.testing.assert_allclose(p.obj(),before,atol=.001)
        for a in ('approach','align','close','lift','clear','carry','release'):p.finish(a)
        for _ in range(5000):p.step()
        self.assertTrue(p.observe()['settled_in_bin'])

    def test_full_size_can_and_bin_containment(self):
        p=Physics();np.testing.assert_allclose(p.m.geom('can').size[:2],[.033,.0575])
        for a in ('approach','align','close','lift','clear','carry','release'):p.finish(a)
        for _ in range(5000):p.step()
        self.assertTrue(p.observe()['settled_in_bin'])
        # Full cylinder footprint, not just its centre, must lie inside the CAD bin.
        import mujoco
        i=p.canid;r=p.d.geom_xmat[i].reshape(3,3);axis=r[:,2]
        extent=.0575*np.abs(axis)+.033*np.sqrt(np.maximum(0,1-axis*axis))
        lo=p.obj()-extent;hi=p.obj()+extent
        self.assertGreater(lo[0],-.175);self.assertLess(hi[0],.001)
        self.assertGreater(lo[1],-.105);self.assertLess(hi[1],.105)
        self.assertGreater(lo[2],.110)

    def test_out_of_reach_rejected(self):
        p=Physics();p.reset('far');before=p.d.qpos.copy()
        with self.assertRaisesRegex(ValueError,'outside arm workspace'):p.command('approach')
        np.testing.assert_array_equal(before,p.d.qpos)

class FakePolicy:
    key='test-only';calls=0;cost=0.;max_calls=60
    def decide(self,state,instruction):
        self.calls+=1
        return {'action':'approach','probabilities':{'approach':1.},'latency_ms':1,'cost':0,'source':'test_only'}

class SessionTests(unittest.TestCase):
    def test_repeated_decisions_stop_before_next_motion(self):
        s=Session(FakePolicy());s.enabled=True;s.running=True
        s.recent_actions=['approach','retry','approach','retry','approach']
        class RetryPolicy(FakePolicy):
            def decide(self,*args):return {'action':'retry','probabilities':{},'latency_ms':1}
        s.policy=RetryPolicy();s.request_decision(s.generation,s.physics.observe(),'Collect')
        self.assertFalse(s.running);self.assertEqual(s.error['code'],'policy_loop');self.assertIsNone(s.physics.plan)
    def test_replay_without_key_or_api_calls(self):
        for scene in ('can','miss','far'):
            s=Session(FakePolicy());s.policy.key='';s.command({'command':'reset','scenario':scene})
            self.assertFalse(s.enabled);s.command({'command':'run'})
            for _ in range(8000):
                s.tick(.02)
                if not s.running:break
            self.assertIsNone(s.error, s.error);self.assertFalse(s.running)
            self.assertTrue(s.physics.observe()['settled_in_bin'],scene)
            self.assertEqual(s.policy.calls,0);self.assertEqual(s.decision['source'],'jev_recorded')
            s.command({'command':'run'});self.assertTrue(s.running);self.assertEqual(s.replay_index,0)

    def test_manual_makes_no_calls(self):
        s=Session(FakePolicy());s.command({'command':'manual','action':'approach'})
        for _ in range(400):s.tick(.01)
        self.assertEqual(s.policy.calls,0);self.assertFalse(s.running)

    def test_replay_accepts_linux_resting_pose_after_release(self):
        s=Session(FakePolicy());s.command({'command':'reset','scenario':'miss'})
        s.command({'command':'run'});s.replay_index=len(s.replay_rows)-1
        expected=s.replay_rows[-1]['observation']
        actual={**expected,'object_m':[-.1127,.0319,.1712]}
        with patch.object(s.physics,'observe',return_value=actual):s.tick(.02)
        self.assertIsNone(s.error);self.assertFalse(s.running)
        self.assertEqual(s.decision['action'],'stop');self.assertEqual(s.policy.calls,0)
        self.assertEqual(s.events[-1]['text'],'Collected and settled in bin.')

    def test_replay_rejects_changed_release_outcome_even_at_recorded_position(self):
        for field,value in [('in_bin',False),('settled_in_bin',False),('held',True),('finger_contacts',1)]:
            with self.subTest(field=field):
                s=Session(FakePolicy());s.command({'command':'run'})
                s.replay_index=len(s.replay_rows)-1
                actual={**s.replay_rows[-1]['observation'],field:value}
                with patch.object(s.physics,'observe',return_value=actual):s.tick(.02)
                self.assertEqual(s.error['code'],'replay_mismatch');self.assertIsNone(s.decision)

    def test_replay_keeps_position_check_before_release(self):
        s=Session(FakePolicy());s.command({'command':'run'})
        s.replay_index=next(i for i,row in enumerate(s.replay_rows) if row['decision']['action']=='release')
        expected=s.replay_rows[s.replay_index]['observation']
        actual={**expected,'object_m':[expected['object_m'][0]+.004,*expected['object_m'][1:]]}
        with patch.object(s.physics,'observe',return_value=actual):s.tick(.02)
        self.assertEqual(s.error['code'],'replay_mismatch');self.assertIsNone(s.physics.plan)

    def test_replay_keeps_position_check_for_further_manipulation(self):
        s=Session(FakePolicy());s.command({'command':'run'})
        s.replay_index=len(s.replay_rows)-1
        row=s.replay_rows[-1];row['decision']['action']='approach'
        expected=row['observation']
        actual={**expected,'object_m':[expected['object_m'][0]+.03,*expected['object_m'][1:]]}
        with patch.object(s.physics,'observe',return_value=actual):s.tick(.02)
        self.assertEqual(s.error['code'],'replay_mismatch');self.assertIsNone(s.physics.plan)

    def test_pause_and_resume_freeze_state(self):
        s=Session(FakePolicy());s.command({'command':'manual','action':'approach'})
        s.tick(.02);s.command({'command':'pause'});before=s.physics.d.qpos.copy();t=s.physics.d.time
        for _ in range(50):s.tick(.02)
        np.testing.assert_array_equal(before,s.physics.d.qpos);self.assertEqual(t,s.physics.d.time)
        s.command({'command':'resume'});s.tick(.02);self.assertGreater(s.physics.d.time,t)

    def test_late_decision_is_discarded_on_pause_reset_api_off(self):
        for command in ({'command':'pause'},{'command':'reset'},{'command':'api','enabled':False}):
            gate=threading.Event();started=threading.Event()
            class SlowPolicy(FakePolicy):
                def decide(self,state,instruction):
                    started.set();gate.wait(3);return super().decide(state,instruction)
            s=Session(SlowPolicy());s.command({'command':'api','enabled':True});s.command({'command':'run'})
            s.tick(.01);self.assertTrue(started.wait(1));s.command(command);gate.set()
            import time
            for _ in range(100):
                if not s.pending:break
                time.sleep(.005)
            self.assertFalse(s.pending);self.assertIsNone(s.physics.plan);self.assertFalse(s.running)
            self.assertEqual(s.policy.calls,1)

    def test_no_fake_donation_error(self):
        for code in ('credits_exhausted','rate_limit','invalid_key'):
            class Failure(FakePolicy):
                def decide(self,*args):raise PolicyError('test error',code)
            s=Session(Failure());s.enabled=True;s.running=True;s.pending=True
            s.request_decision(s.generation,s.physics.observe(),'Collect')
            self.assertEqual(s.error['code'],code);self.assertFalse(s.running)
            self.assertEqual(s.enabled,code!='credits_exhausted')

    def test_reset_keeps_cost_and_call_count(self):
        p=FakePolicy();p.calls=17;p.cost=.02;s=Session(p);s.command({'command':'reset'})
        self.assertEqual(s.state()['calls'],17);self.assertEqual(s.state()['cost'],.02)
        json.dumps(s.state(),allow_nan=False)

class APITests(unittest.TestCase):
    def test_http_402_maps_to_credits_exhausted(self):
        class Response:
            status=402
            def read(self,*args):return b'{}'
        class Connection:
            def __init__(self,*args,**kwargs):pass
            def request(self,*args):pass
            def getresponse(self):return Response()
            def close(self):pass
        j=Jev();j.key='test-only'
        with patch('policy.http.client.HTTPSConnection',Connection):
            with self.assertRaises(PolicyError) as error:j.decide({},'Pick up')
        self.assertEqual(error.exception.code,'credits_exhausted');self.assertEqual(j.calls,1)

if __name__=='__main__':unittest.main(verbosity=2)
