"""Physics-only executor. Object motion is exclusively MuJoCo contact dynamics.

Only reset writes object qpos. IK uses a separate scratch MjData, never the live
state. Primitives change actuator targets and observe the resulting motion.
"""
from pathlib import Path
import numpy as np
import mujoco
from build_model import JOINTS

ROOT = Path(__file__).resolve().parent
HOME = np.array([1.34, -.65, .4, 1.25, -.05])
BIN = np.array([-.087, 0., .36])
SCENARIOS = {'can': [.315,0,.058], 'offset': [.30,.055,.058], 'miss': [.315,0,.058], 'far': [.64,0,.058]}

class Physics:
    def __init__(self, model_path=None):
        self.m = mujoco.MjModel.from_xml_path(str(model_path or ROOT/'model/moss.xml'))
        self.d = mujoco.MjData(self.m)
        self.ikd = mujoco.MjData(self.m)
        self.jids = [self.m.joint(n).id for n in JOINTS]
        self.qa = np.array([self.m.jnt_qposadr[j] for j in self.jids])
        self.va = np.array([self.m.jnt_dofadr[j] for j in self.jids])
        self.fqa = [self.m.jnt_qposadr[self.m.joint('finger_'+n).id] for n in ['left','right']]
        self.oq = self.m.jnt_qposadr[self.m.joint('litter_free').id]
        self.bq = self.m.jnt_qposadr[self.m.joint('base_x').id]
        self.sid = self.m.site('tcp').id
        self.padids = [self.m.geom('pad_'+n).id for n in ['left','right']]
        self.canid = self.m.geom('can').id
        self.reset()

    def reset(self, scenario='can'):
        mujoco.mj_resetData(self.m, self.d)
        self.scenario = scenario
        self.d.qpos[self.qa] = HOME
        self.d.qpos[self.fqa] = .037
        self.d.ctrl[:] = [*HOME, .037, .037, 0]
        self.d.qpos[self.oq:self.oq+3] = SCENARIOS[scenario]
        self.d.qpos[self.oq+3:self.oq+7] = [1,0,0,0]
        self.plan = None
        self.last_action = None
        self.last_result = 'Ready'
        self.attempts = 0
        self.failures = 0
        self.failed_attempts = set()
        self.needs_regrasp = False
        self.miss_used = False
        self.success_ticks = 0
        mujoco.mj_forward(self.m,self.d)
        for _ in range(150): self.step()

    def base(self):
        return float(self.d.qpos[self.bq])

    def bin_target(self):
        return BIN + [self.base(),0,0]

    def tcp(self):
        return self.d.site_xpos[self.sid].copy()

    def obj(self):
        return self.d.xpos[self.m.body('litter').id].copy()

    def contacts(self):
        pads = set()
        for c in self.d.contact:
            if c.dist > .001: continue
            pair = {int(c.geom1),int(c.geom2)}
            if self.canid in pair:
                pads.update(pair.intersection(self.padids))
        return len(pads)

    def in_bin(self):
        x,y,z = self.obj()-[self.base(),0,0]
        return bool(-.137 < x < -.037 and abs(y)<.067 and .14 < z < .22)

    def held(self):
        return bool(self.contacts() == 2 and self.obj()[2] > .09)

    def object_speed(self):
        adr=self.m.jnt_dofadr[self.m.joint('litter_free').id]
        return float(np.linalg.norm(self.d.qvel[adr:adr+3]))

    def failed(self):
        self.failed_attempts.add(self.attempts)
        self.failures=len(self.failed_attempts)
        self.needs_regrasp=True

    def ik(self, target, down=True, seed=None):
        d,m = self.ikd,self.m
        best=(None,float('inf'))
        seeds=[self.d.qpos[self.qa].copy() if seed is None else np.asarray(seed).copy()]
        if target[0]-self.base() < .08: seeds += [np.array([-1.79,-.83,-.11,1.41,-1.585])]
        else: seeds += [HOME.copy(), np.array([1.34,.8,.086,.69,-.05])]
        lo,hi = m.jnt_range[self.jids].T
        for q in seeds:
            for _ in range(130):
                d.qpos[:] = self.d.qpos
                d.qpos[self.qa] = q
                mujoco.mj_forward(m,d)
                r = d.site_xmat[self.sid].reshape(3,3)
                err=np.asarray(target)-d.site_xpos[self.sid]
                jp,jr=np.zeros((3,m.nv)),np.zeros((3,m.nv))
                mujoco.mj_jacSite(m,d,jp,jr,self.sid)
                J=jp[:,self.va]
                if down:
                    axis=r[:,2]
                    # d(axis)/dq = angular Jacobian cross axis.
                    ja=np.cross(jr[:,self.va].T,axis).T
                    err=np.r_[err,.08*(np.array([0,0,-1])-axis)]
                    J=np.vstack([J,.08*ja])
                score=np.linalg.norm(err)
                if score < best[1]: best=(q.copy(),float(score))
                if score < .0002: break
                delta=J.T@np.linalg.solve(J@J.T+np.eye(len(err))*.000015,err)
                q=np.clip(q+np.clip(delta,-.12,.12),lo+.005,hi-.005)
        return best

    def command(self, action):
        if self.plan: raise ValueError('Motion already in progress')
        p=self.obj(); tcp=self.tcp(); q=self.d.qpos[self.qa].copy()
        gap=float(np.mean(self.d.ctrl[5:7])); duration=1.2; base_target=float(self.d.ctrl[7])
        if action=='approach':
            target=p+np.array([0,0,.085]); gap=.037
        elif action=='align':
            target=p+np.array([0,0,.003]); gap=.037
            if self.scenario=='miss' and not self.miss_used:
                target[1]+=.085
                self.miss_used=True
        elif action=='close':
            target=None; gap=0.;duration=.65;self.attempts+=1
        elif action=='lift':
            target=tcp+np.array([0,0,.09]);duration=1.4
        elif action=='clear':
            target=np.array([self.base()+.19,0,.40]);duration=2.4
        elif action=='carry':
            target=self.bin_target();duration=3.2
        elif action=='release':
            target=None;gap=.037;duration=3.5
        elif action=='retry':
            target=None;gap=.037;duration=.7
        elif action=='reposition':
            target=None;q=HOME.copy();gap=.037;duration=4.
            base_target=float(np.clip(p[0]-.315,0,.8))
        elif action=='home':
            target=None;q=HOME.copy();gap=.037;duration=2
        else: raise ValueError('Unknown action')
        if target is not None:
            q,error=self.ik(target,down=action not in ('clear','carry'))
            if error > .008: raise ValueError(f'Target outside arm workspace ({error*1000:.0f} mm IK error)')
        start=self.d.ctrl.copy();end=np.r_[q,gap,gap,base_target]
        duration=max(duration,float(np.max(np.abs(end[:5]-start[:5])))*1.35)
        self.plan={'action':action,'start':start,'end':end,'duration':duration,'elapsed':0.,'hold':2. if action=='reposition' else 0.,'object_start_z':float(p[2])}
        self.last_action=action

    def step(self):
        if self.plan:
            plan=self.plan;plan['elapsed']+=float(self.m.opt.timestep)
            u=min(1.,max(0.,plan['elapsed']-plan['hold'])/plan['duration']);u=u*u*u*(10+u*(-15+6*u))
            self.d.ctrl[:]=plan['start']*(1-u)+plan['end']*u
        mujoco.mj_step(self.m,self.d)
        if not np.isfinite(self.d.qpos).all(): raise RuntimeError('Non-finite physics state')
        self.success_ticks=self.success_ticks+1 if self.in_bin() and self.contacts()==0 and self.object_speed()<.02 else 0
        if self.plan and self.plan['elapsed']>=self.plan['duration']+self.plan['hold']+(2.5 if self.plan['action']=='release' else .45):
            a=self.plan['action'];self.plan=None
            if a=='lift' and not self.held():
                self.failed();self.last_result='Missed grasp: object did not rise between both fingers'
            elif a=='close':
                if not self.contacts():self.failed()
                self.last_result=f'Closed: {self.contacts()} finger contacts'
            elif a=='retry':
                self.needs_regrasp=False;self.last_result='Recovery complete: fingers reopened. Ready to approach and align again.'
            elif a=='release': self.last_result='Object settled in bin' if self.success_ticks>150 else ('Object in bin; still settling' if self.in_bin() else 'Released outside bin')
            else:self.last_result=a+' complete'
            return self.last_result

    def finish(self, action):
        """Headless executor helper used by integration checks, not the Jev policy."""
        self.command(action)
        while self.plan:self.step()
        return self.last_result

    def observe(self):
        return {'object_m':self.obj().round(4).tolist(),'tcp_m':self.tcp().round(4).tolist(),
                'finger_contacts':self.contacts(),'held':self.held(),'in_bin':self.in_bin(),
                'settled_in_bin':self.success_ticks>150,'gap_mm':round(float(np.sum(self.d.qpos[self.fqa])+.008)*1000,1),
                'last_action':self.last_action,'last_result':self.last_result,'attempts':self.attempts,'missed_grasps':self.failures,
                'needs_regrasp':self.needs_regrasp,'grasp_attempts_left':max(0,3-self.failures),
                'sim_seconds':round(self.d.time,2),'moving':bool(self.plan),'scenario':self.scenario,
                'object_speed_m_s':round(self.object_speed(),4),
                'base_x_m':round(self.base(),4),'bin_target_m':self.bin_target().round(4).tolist(),
                'reposition_notice':bool(self.plan and self.plan['action']=='reposition' and self.plan['elapsed']<self.plan['hold']),
                'within_work_area':float(np.linalg.norm(self.obj()[:2]-[self.base()+.085,-.03]))<.31}

    def transforms(self):
        bodies={}
        for i in range(1,self.m.nbody):
            n=mujoco.mj_id2name(self.m,mujoco.mjtObj.mjOBJ_BODY,i)
            bodies[n]=[*self.d.xpos[i].tolist(),*self.d.xquat[i].tolist()]
        geoms={}
        for n in ['palm','gripper_mount','pad_left','pad_right','can']:
            i=self.m.geom(n).id
            geoms[n]={'p':self.d.geom_xpos[i].tolist(),'r':self.d.geom_xmat[i].tolist(),'size':self.m.geom_size[i].tolist()}
        return {'bodies':bodies,'geoms':geoms}
