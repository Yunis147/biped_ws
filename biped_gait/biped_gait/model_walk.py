"""Quasi-static walking using URDF geometry, masses and joint feedback only.

Body orientation and ground height are inferred from a flat supporting sole.
They are model assumptions, not measurements of gravity or ground contact.
"""
from dataclasses import dataclass,asdict
import math
import numpy as np
from . import quasi_static_walk as m
from .servo_feedback import ServoFeedback
from .placement import Placement,PhaseGeometry,ground_estimate

NAMES=m.JOINT_ORDER


@dataclass(frozen=True)
class Settings:
    step_length: float=.030
    stance_width: float=.090
    body_roll: float=.040
    crouch: float=.024
    centre_crouch: float=.018
    com_inset: float=.010
    support_margin: float=.008
    clearance: float=.006
    lift_clearance: float=.005
    max_joint_speed: float=.18
    landing_speed: float=.0015
    landing_tracking_error: float=.025
    landing_sole_tilt: float=math.radians(2.)
    landing_clearance: float=0.
    placement_tolerance: float=.0003
    minimum_knee_bend: float=.12
    landing_alignment_clearance: float=.001
    landing_xy_speed: float=.003
    landing_xy_tolerance: float=.001
    placement_ramp_time: float=1.
    placement_capture_acceleration: float=.30
    penetration_tolerance: float=.001
    tracking_pause: float=.04
    tracking_fault: float=.15
    tracking_timeout: float=2.
    settle_tracking_error: float=.03
    settle_joint_speed: float=.08
    settle_timeout: float=4.
    settle_duration: float=.4

    def __post_init__(self):
        if any(not math.isfinite(v) or (v<0 if k in ('body_roll','landing_clearance') else v<=0)
               for k,v in asdict(self).items()):
            raise ValueError('walking settings must be finite and positive')
        if not .080<=self.stance_width<=.110:
            raise ValueError('stance width must be between 80 and 110 mm')
        if not .01<=self.step_length<=.04 or self.body_roll>.06:
            raise ValueError('step/torso angle outside the model walking range')
        if not .012<=self.centre_crouch<=self.crouch<=.04:
            raise ValueError('invalid body heights')
        if not self.lift_clearance<=self.clearance<=.012:
            raise ValueError('invalid foot clearances')
        if self.landing_speed>.002 or self.max_joint_speed>.25:
            raise ValueError('landing/joint speed exceeds the walking limit')
        if not self.landing_tracking_error<=self.tracking_pause<self.tracking_fault<=.20:
            raise ValueError('invalid tracking thresholds')
        if not .006<=self.support_margin<=.010 or self.landing_clearance>.001:
            raise ValueError('invalid support/placement margin')
        if (self.landing_alignment_clearance>self.clearance or self.landing_xy_speed>.005
                or self.landing_xy_tolerance>.002 or self.placement_tolerance>.0005
                or self.minimum_knee_bend<.12):
            raise ValueError('invalid placement bounds')
        if not .5<=self.placement_ramp_time<=2. or self.placement_capture_acceleration>.30:
            raise ValueError('invalid placement smoothing')

    def profile(self):
        fields=('step_length','stance_width','body_roll','crouch','centre_crouch','com_inset',
                'support_margin','clearance','lift_clearance')
        return dict(**{n:getattr(self,n) for n in fields},com_x_midpoint=True)


def body_rotation(q,stance='l'):
    return m._foot_transform(dict(zip(NAMES,q)),stance)[0].T


class Plan:
    def __init__(self,direction,initial,settings):
        self.initial=np.asarray(initial).copy()
        self.profile=settings.profile()
        q=dict(zip(NAMES,self.initial))
        rotation=body_rotation(self.initial)
        if np.linalg.norm(rotation-np.eye(3))>1e-8:
            original=dict(q)
            for side in 'rl':
                q=m._leg_pose_ik(q,side,rotation@m._sole_center(original,side),
                    rotation@m._foot_transform(original,side)[0])
        heights=[m._sole_center(q,s)[2] for s in 'rl']
        if abs(heights[0]-heights[1])>.001:
            raise ValueError('stopped soles differ by over 1 mm; restart from the known standing pose')
        if abs(heights[0]-heights[1])>1e-6:
            for side in 'rl':
                target=m._sole_center(q,side);target[2]=float(np.mean(heights))
                q=m._leg_pose_ik(q,side,target,np.eye(3))
        template=np.array([q[n] for n in NAMES])
        self.keyframes,self.tables=m.build_tables(direction,template,self.profile)
        if not m.report(direction,self.keyframes,self.tables,lambda msg:None,self.profile):
            raise ValueError('walking path failed floor/support/joint checks')
        self.metrics=m.gait_metrics(self.keyframes,self.tables,self.profile)
        self.durations={seg:max(m.segment_duration(seg[1],table),
            1.875*settings.clearance/settings.landing_speed if seg[1].startswith('place_') else 0.)
            for seg,table in self.tables.items()}


class Plans:
    def __init__(self,settings=None,*,precomputed=False):
        self.settings=settings or Settings()
        if precomputed:
            from .plan_store import load
            self.cache={d:load(self.settings,d,np.zeros(12)) for d in ('forward','backward')}
        else:
            self.cache={d:Plan(d,np.zeros(12),self.settings) for d in ('forward','backward')}

    def get(self,direction,initial):
        if not np.allclose(self.cache[direction].initial,initial,atol=1e-10,rtol=0):
            self.cache[direction]=Plan(direction,initial,self.settings)
        return self.cache[direction]


class WalkingController:
    def __init__(self,plans=None,*,simulation=False):
        self.plans=plans or Plans();self.s=self.plans.settings
        self.simulation=simulation;self.feedback=ServoFeedback()
        self.plan=self.plans.cache['forward'];self.direction='forward'
        self.nominal=np.zeros(12);self.commanded=None
        self.state='idle';self.segment=None;self.support='l'
        self.elapsed=self.waited=self.settled=0.;self.last_time=None
        self.stop_requested=self.single_step=self.stand_only=False
        self.fault=self.reason='';self.snapshot=None;self.suppress=False
        self.landings=0;self.tracking_error=0.;self.tracking_joint=None
        self.lag_since=self.pause_since=None
        self.placement=self.phase_geometry=None
        self.clearance=None;self.margin=None;self.landing_cues={}
        self.interruption='';self.interruption_snapshot=None;self.relief_target=None
        self.joint_source_time=None;self.target_problem=''

    def readiness(self,now):
        if self.feedback.joint_problem:return self.feedback.joint_problem
        if self.feedback.joint_time is None or not 0<=now-self.feedback.joint_time<=.25:
            return 'missing/stale joint positions'
        return '' if self.simulation else self.feedback.problem(now)

    def fail(self,reason,now,suppress=False):
        if not self.fault:
            self.fault=reason
            self.snapshot=dict(time=now,state=self.state,segment=self.segment,
                tracking_joint=self.tracking_joint,tracking_error_rad=self.tracking_error,
                measured=None if self.feedback.q is None else self.feedback.q.tolist(),
                target=None if self.commanded is None else self.commanded.tolist(),
                landing_cues=dict(self.landing_cues),joint_source_time=self.joint_source_time,
                diagnostics=getattr(self,'last_diagnostics',None))
        self.state='fault';self.reason=self.fault;self.suppress|=suppress

    def command(self,text,now):
        text=text.strip().lower()
        if text=='status':return
        if text=='hold':
            if self.commanded is not None:self.fail('hold requested; last target retained',now)
            return
        if self.fault:raise ValueError('latched fault: '+self.fault+'; restart from a supported known stance')
        if text=='stop':self.stop_requested=True;return
        if text not in ('start','forward','backward','step','stand'):
            raise ValueError('use stand / start / forward / backward / step / stop / hold / status')
        if self.state not in ('idle','standing'):raise ValueError('already moving; stop and wait for standing')
        problem=self.readiness(now)
        if problem:raise ValueError(problem)
        if self.feedback.velocity is None or np.max(np.abs(self.feedback.velocity))>.08:
            raise ValueError('wait for stationary servos')
        if np.max(np.abs(self.feedback.q-self.nominal))>.05:
            raise ValueError('start near calibrated zero or the last held stance')
        self.direction='backward' if text=='backward' else 'forward'
        self.plan=self.plans.get(self.direction,self.nominal)
        self.stop_requested=False;self.single_step=text=='step';self.stand_only=text=='stand'
        self.landings=0;self.interruption='';self.interruption_snapshot=None
        actual=self.feedback.q.copy()
        self.approach=np.array([actual+m._smoothstep(u)*(self.nominal-actual) for u in np.linspace(0,1,81)])
        speed,accel=m._motion_bounds(self.approach)
        self.approach_duration=max(1.,speed/.1,math.sqrt(accel/.3))
        self.commanded=actual;self.state=self.reason='approach'
        self.last_time=now;self.elapsed=self.settled=0.

    def _begin(self,a,b):
        self.segment=(a,b);self.state=self.reason='moving'
        self.elapsed=self.waited=self.settled=0.
        self.support=m.STANCE.get(b,self.support)
        self.clearance=None;self.landing_cues={};self.placement=None
        try:
            self.phase_geometry=PhaseGeometry(self.plan.tables[(a,b)][0],self.nominal,self.support)
            if b.startswith('place_'):
                self.placement=Placement(self.feedback.q,self.support,self.s,
                    reference_goal=self.plan.tables[(a,b)][-1])
        except ValueError as error:self.fail(str(error),self.last_time or 0.)

    def _advance(self,now):
        arrived=self.segment[1]
        if arrived.startswith('place_'):self.landings+=1
        if arrived=='neutral' or (arrived=='ready' and self.stand_only):
            self.state=self.reason='standing';self.segment=None;return
        stop=(self.stop_requested or self.single_step and self.landings) and arrived in m.SAFE_STOP
        following='neutral' if stop else m.NEXT[arrived]
        if following in m.STANCE:
            stance=m.STANCE[following]
            com,feet,_=m._ground_frame(dict(zip(NAMES,self.feedback.q)),stance)
            self.margin=m._margin(m._hull(feet[stance][:,:2]),com[:2])
            if self.margin<.005:
                self.fail(f'model support margin {self.margin*1000:.2f} mm; foot lift blocked',now);return
        self._begin(arrived,following)

    def _placement_hold(self,reason,now):
        if self.interruption_snapshot is None:
            self.interruption_snapshot=dict(time=now,segment=self.segment,reason=reason,
                measured=self.feedback.q.tolist(),target=self.commanded.tolist(),
                clearance_m=self.clearance,landing_cues=dict(self.landing_cues))
        self.interruption=reason+'; contact not measured; placement held'
        self.state='placement_hold';self.reason=self.interruption
        try:self.relief_target=self.placement.relief(self.commanded,self.feedback.q)
        except ValueError:self.fail('no feasible placement relief: '+reason,now)

    def _placing(self,dt,now):
        self.reason='placing from supporting-leg geometry; no extension below model floor'
        for scale in (1.,.5,.25,.125):
            try:
                target,descent,done,exhausted=self.placement.target(self.commanded,self.feedback.q,dt*scale)
                delta=target-self.commanded
                if np.max(np.abs(delta))>self.s.max_joint_speed*dt+1e-7:continue
                before=ground_estimate(self.commanded,self.support).clearance
                after=ground_estimate(target,self.support).clearance
                if before-after>self.s.landing_speed*dt+1e-7:continue
                self.placement.accept(descent)
                self.nominal=self.commanded=target.copy();self.elapsed+=dt*scale;self.pause_since=None
                if done:self.state='settling';self.waited=self.settled=0.
                elif self.elapsed>self.plan.durations[self.segment]+self.s.settle_timeout+self.s.placement_ramp_time+self.placement.capture_duration:
                    self._placement_hold('measured landing height budget exhausted',now)
                elif exhausted:self.reason='landing height budget reached; holding without further descent'
                return self.commanded.copy()
            except ValueError as error:
                self._placement_hold(str(error),now);return self.commanded.copy()
        self.pause_since=now if self.pause_since is None else self.pause_since
        if now-self.pause_since>1.:self._placement_hold('no bounded landing target',now)
        return self.commanded.copy()

    def tick(self,now):
        dt=0. if self.last_time is None else now-self.last_time;self.last_time=now
        if self.state=='idle':return None
        problem=self.readiness(now)
        if problem:self.fail(problem,now,suppress=True)
        if self.suppress:return None
        if self.commanded is None:return None
        errors=np.abs(self.feedback.q-self.commanded);index=int(np.argmax(errors))
        self.tracking_error=float(errors[index]);self.tracking_joint=NAMES[index]
        if dt<0 or dt>.25:self.fail('control timer gap/reset; last target held',now)
        if self.tracking_error>self.s.tracking_fault:self.fail('large joint tracking error; last target held',now)
        if self.fault:return self.commanded.copy()
        dt=min(dt,.04)
        if self.tracking_error>self.s.tracking_pause:
            self.lag_since=now if self.lag_since is None else self.lag_since
            self.reason='trajectory paused for joint tracking'
            if now-self.lag_since>self.s.tracking_timeout:self.fail('joint tracking did not recover',now)
            return self.commanded.copy()
        self.lag_since=None
        if self.state in ('approach','approach_settle'):
            self.elapsed+=dt
            if self.state=='approach':
                self.nominal=self.commanded=m.table_pose(self.approach,self.elapsed/self.approach_duration)
                if self.elapsed>=self.approach_duration:self.state='approach_settle';self.elapsed=0.
            else:
                quiet=self.tracking_error<.02 and np.max(np.abs(self.feedback.velocity))<.08
                self.settled=self.settled+dt if quiet else 0.
                if self.stop_requested:self.state='standing';self.segment=None
                elif self.settled>=.2:self._begin('neutral','ready')
                elif self.elapsed>5.:self.fail('starting pose did not settle',now)
            self.reason=self.fault or self.state;return self.commanded.copy()
        landing=self.segment is not None and self.segment[1].startswith('place_')
        if landing and self.state not in ('placement_hold','standing'):
            g=ground_estimate(self.feedback.q,self.support);self.clearance=g.clearance
            swing=[NAMES.index(j['name']) for j in m.LEG[g.swing]]
            swing_error=float(errors[swing].max())
            rotation=g.rotation@m._foot_transform(dict(zip(NAMES,self.feedback.q)),g.swing)[0]
            sole_tilt=math.acos(float(np.clip(rotation[2,2],-1.,1.)))
            self.landing_cues={}
            if swing_error>self.s.landing_tracking_error:self.landing_cues['swing_tracking_error_rad']=swing_error
            if g.clearance< -self.s.penetration_tolerance:self.landing_cues['model_clearance_m']=g.clearance
            if g.clearance<.002 and sole_tilt>self.s.landing_sole_tilt:self.landing_cues['sole_tilt_deg']=math.degrees(sole_tilt)
            if self.landing_cues:
                self._placement_hold('landing interrupted: '+str(self.landing_cues),now)
                return self.commanded.copy()
        if landing and self.state=='moving':return self.commanded.copy() if dt<=0 else self._placing(dt,now)
        if self.state=='settling':
            self.waited+=dt
            quiet=self.tracking_error<self.s.settle_tracking_error and np.max(np.abs(self.feedback.velocity))<self.s.settle_joint_speed
            self.settled=self.settled+dt if quiet else 0.
            minimum=m.DWELL_TIME.get(self.segment[1],.4)*m.TIME_SCALE
            if self.waited>=minimum and self.settled>=self.s.settle_duration:self._advance(now)
            elif self.waited>minimum+self.s.settle_timeout:self.fail('joint targets did not settle; next phase blocked',now)
        if self.fault:return self.commanded.copy()
        if not landing and self.state=='moving' and self.segment[1].startswith('place_'):return self.commanded.copy()
        for scale in (1.,.5,.25,.125,0.):
            elapsed=self.elapsed+(dt*scale if self.state=='moving' else 0.)
            target=self.nominal.copy()
            try:
                if self.state=='moving':
                    fraction=elapsed/self.plan.durations[self.segment]
                    target=m.table_pose(self.plan.tables[self.segment],fraction)
                    target=self.phase_geometry.pose(target,self.nominal,fraction)
                elif self.state=='placement_hold':
                    delta=self.relief_target-target
                    target+=delta*min(1.,.1*dt*scale/max(float(np.max(np.abs(delta))),1e-9))
                if np.max(np.abs(target-self.commanded))>self.s.max_joint_speed*dt+1e-7:continue
            except ValueError as error:
                self.target_problem=str(error);continue
            self.nominal=self.commanded=target.copy();self.elapsed=elapsed;self.target_problem=''
            if scale==0. and self.state=='moving':
                self.pause_since=now if self.pause_since is None else self.pause_since
                if now-self.pause_since>2.:self.fail('no feasible synchronized joint progress',now)
            else:self.pause_since=None
            if self.state=='moving' and self.elapsed>=self.plan.durations[self.segment]:
                self.state='settling';self.waited=self.settled=0.
            break
        else:
            self.pause_since=now if self.pause_since is None else self.pause_since
            if now-self.pause_since>2.:self.fail('no feasible synchronized joint target: '+self.target_problem,now)
        self.reason=self.fault or (self.interruption if self.state=='placement_hold' else self.state)
        return self.commanded.copy()

    def status(self,now):
        placement=None if self.placement is None else dict(body_height_m=self.placement.last.body_height,
            initial_clearance_m=self.placement.initial_clearance,measured_clearance_m=self.placement.last.clearance,
            remaining_descent_m=self.placement.remaining,commanded_descent_m=self.placement.commanded_descent,
            xy_error_m=self.placement.xy_error,aligning=bool(self.placement.aligning))
        return dict(state=self.state,segment=self.segment,reason=self.reason,fault=self.fault,fault_snapshot=self.snapshot,
            hardware_ready=not bool(self.readiness(now)),output_suppressed=self.suppress,
            tracking_joint=self.tracking_joint,tracking_error_rad=self.tracking_error,
            step_length_m=self.s.step_length,maximum_planned_roll_deg=math.degrees(self.s.body_roll),
            orientation_source='flat supporting-foot kinematics',support_foot=self.support,
            model_support_margin_m=self.margin,predicted_swing_clearance_m=self.clearance,
            landings=self.landings,contact_measured=False,placement=placement,
            interruption=self.interruption,interruption_snapshot=self.interruption_snapshot,target_problem=self.target_problem)
