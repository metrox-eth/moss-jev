"""Loopback-only MOSS live simulation. No hardware interface and no public proxy."""
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlsplit
import json, threading, time, os
from physics import Physics, SCENARIOS
from policy import Jev, PolicyError, ACTIONS

ROOT=Path(__file__).resolve().parent
PORT=int(os.environ.get('MOSS_SIM_PORT','8875'))
SPONSOR='https://github.com/sponsors/metrox-eth'

def replay_observation_matches(actual, expected, next_action):
    if expected.get('last_action') == 'release':
        outcome = ('in_bin', 'settled_in_bin', 'held', 'finger_contacts')
        if actual.get('last_action') != 'release' or any(actual[k] != expected[k] for k in outcome):
            return False
        # Contact settling can choose a different resting pose across platforms.
        # Only a completed, released collection may ignore that position drift;
        # another manipulation still requires the recorded geometry to match.
        if (next_action == 'stop' and expected['in_bin'] and expected['settled_in_bin']
                and not expected['held'] and expected['finger_contacts'] == 0):
            return True
    return all(abs(x-y) <= .003 for x,y in zip(actual['object_m'], expected['object_m']))

class Session:
    def __init__(self, policy=None):
        self.lock=threading.RLock();self.physics=Physics();self.policy=policy or Jev()
        self.enabled=False;self.running=False;self.pending=False;self.generation=0
        self.error=None;self.events=[];self.decision=None;self.closed=False
        self.instruction='Pick up the can and put it in the bin.'
        self.action_count=0;self.mode='manual';self.last_tick=time.monotonic()
        self.recent_actions=[];self.replay_rows=[];self.replay_index=0
        self.event('ready','Local physics ready. API is off.')

    def event(self,kind,text):
        self.events.append({'kind':kind,'text':text,'t':round(self.physics.d.time,2)})
        self.events=self.events[-40:]

    def state(self):
        with self.lock:
            return {'observation':self.physics.observe(),'transforms':self.physics.transforms(),
                    'running':self.running,'pending':self.pending,'api_enabled':self.enabled,
                    'key_ready':bool(self.policy.key),'calls':self.policy.calls,'cost':self.policy.cost,
                    'call_limit':self.policy.max_calls,'error':self.error,'decision':self.decision,
                    'events':self.events.copy(),'instruction':self.instruction,'mode':self.mode,'sponsor_url':SPONSOR,
                    'generation':self.generation}

    def command(self, cmd):
        with self.lock:
            name=cmd.get('command')
            if name=='api':
                if not isinstance(cmd.get('enabled'),bool):raise ValueError('enabled must be boolean')
                self.enabled=cmd['enabled'];self.generation+=1;self.error=None
                self.running=False;self.physics.reset(self.physics.scenario);self.decision=None;self.mode='manual'
                self.event('api','API enabled. Press Run Jev to start.' if self.enabled else 'Replay ready. No API calls.')
                if self.enabled and not self.policy.key:
                    self.error={'code':'missing_key','message':'Server key missing. Run configure_api.py locally, then restart the server. Your key never goes to the browser.'}
            elif name=='reset':
                scenario=cmd.get('scenario','can')
                if scenario not in SCENARIOS:raise ValueError('Unknown scenario')
                self.generation+=1;self.running=False;self.error=None;self.decision=None;self.action_count=0
                self.recent_actions=[]
                self.physics.reset(scenario);self.events=[];self.event('reset','Scene reset. API usage is retained.')
            elif name in ('pause','stop'):
                self.generation+=1;self.running=False
                if name=='stop':
                    self.physics.plan=None;self.physics.d.ctrl[:5]=self.physics.d.qpos[self.physics.qa]
                self.event(name,'Paused.' if name=='pause' else 'Stopped. Reset or choose a new mission to continue.')
            elif name=='resume':
                if not self.physics.plan and self.mode not in ('replay','jev'):raise ValueError('No paused mission to resume')
                if self.mode=='jev' and not self.enabled:raise ValueError('Enable API LIVE before resuming the mission')
                self.running=True;self.last_tick=time.monotonic();self.event('resume','Motion resumed.')
            elif name=='run':
                if self.pending:raise ValueError('Wait for the previous API request to finish.')
                if not self.enabled:
                    path=ROOT/'recordings'/f'{self.physics.scenario}.json'
                    if not path.is_file():raise ValueError('No recording for this scenario yet.')
                    self.replay_rows=json.loads(path.read_text())['rows'];self.replay_index=0
                    self.physics.reset(self.physics.scenario);self.events=[];self.decision=None
                    self.generation+=1;self.error=None;self.running=True;self.mode='replay';self.last_tick=time.monotonic()
                    self.event('replay','Replaying recorded Jev decisions. No API calls.');return
                if not self.policy.key:
                    self.error={'code':'missing_key','message':'API key missing on server. Add it and restart.'}
                    return
                instruction=cmd.get('instruction',self.instruction)
                if not isinstance(instruction,str) or not 1<=len(instruction.strip())<=300:raise ValueError('Instruction must be 1–300 characters')
                self.physics.reset(self.physics.scenario);self.recent_actions=[];self.action_count=0;self.events=[]
                self.instruction=instruction.strip();self.error=None;self.running=True;self.mode='jev'
                self.generation+=1;self.last_tick=time.monotonic();self.event('run','Live Jev mission started.')
            elif name=='manual':
                if self.pending or self.running:raise ValueError('Pause first and wait for any API call to finish.')
                action=cmd.get('action')
                if action not in ACTIONS or action=='stop':raise ValueError('Unknown manual action')
                self.physics.command(action);self.mode='manual';self.running=True;self.error=None;self.decision=None
                self.event('manual','Manual physics check: '+action);self.last_tick=time.monotonic()
            else:raise ValueError('Unknown command')

    def request_decision(self, generation, observation, instruction):
        try:
            result=self.policy.decide(observation,instruction)
            with self.lock:
                if generation!=self.generation or not self.running or not self.enabled:
                    self.event('cancelled','Late API answer discarded. No motion executed.');return
                self.decision=result;action=result['action'];self.action_count+=1
                self.event('jev','Jev → '+action)
                self.recent_actions=(self.recent_actions+[action])[-8:]
                repeated=False
                for length in (1,2):
                    tail=self.recent_actions[-3*length:]
                    if len(tail)==3*length and tail[:length]==tail[length:2*length]==tail[2*length:]:
                        repeated=True
                if repeated and action!='stop':
                    self.running=False;self.error={'code':'policy_loop','message':'Repeated decision loop detected. Paused before spending more credits.'}
                    self.event('error',self.error['message']);return
                if action=='stop':
                    self.running=False
                    self.event('done','Collected and settled in bin.' if self.physics.observe()['settled_in_bin'] else 'Jev stopped the mission.')
                elif action in ('clear','carry') and not self.physics.held():
                    self.event('feedback','Action rejected: no object held. Jev receives this feedback.')
                    self.physics.last_result='Rejected '+action+': object is not held; retry grasp.'
                elif action=='release' and (abs(self.physics.tcp()[0]-self.physics.bin_target()[0])>.055 or abs(self.physics.tcp()[1])>.075):
                    self.physics.last_result='Rejected release: gripper is not above the bin.'
                    self.event('feedback',self.physics.last_result)
                else:
                    try:self.physics.command(action)
                    except ValueError as exc:
                        self.physics.last_result=str(exc);self.event('feedback',str(exc))
                if self.action_count>=30:
                    self.running=False;self.error={'code':'step_limit','message':'30-action limit reached. Review the scene before restarting.'}
        except PolicyError as exc:
            with self.lock:
                if generation==self.generation:
                    self.running=False;self.error={'code':exc.code,'message':str(exc)}
                    if exc.code=='credits_exhausted':self.enabled=False
                    self.event('error',str(exc))
        finally:
            with self.lock:self.pending=False;self.last_tick=time.monotonic()

    def tick(self, dt):
        with self.lock:
            if not self.running:return
            if self.physics.plan:
                for _ in range(max(1,min(30,round(dt/.002)))):
                    result=self.physics.step()
                    if result:
                        self.event('physics',result)
                        if self.mode=='manual':self.running=False
                        break
            elif self.mode=='replay':
                if self.replay_index>=len(self.replay_rows):self.running=False;return
                row=self.replay_rows[self.replay_index];self.replay_index+=1
                actual=self.physics.observe();expected=row['observation']
                if not replay_observation_matches(actual, expected, row['decision']['action']):
                    self.running=False;self.error={'code':'replay_mismatch','message':'Scene differs from recording. Reset to replay.'};return
                self.decision={**row['decision'],'source':'jev_recorded'};action=self.decision['action']
                self.event('jev','Recorded Jev → '+action)
                if action=='stop':
                    self.running=False;self.event('done','Collected and settled in bin.' if actual['settled_in_bin'] else 'Recorded run complete.')
                else:self.physics.command(action)
            elif self.mode=='jev' and self.enabled and not self.pending:
                self.pending=True
                threading.Thread(target=self.request_decision,args=(self.generation,self.physics.observe(),self.instruction),daemon=True).start()

    def loop(self):
        while not self.closed:
            now=time.monotonic();dt=now-self.last_tick;self.last_tick=now
            try:self.tick(dt)
            except Exception:
                with self.lock:
                    self.running=False;self.error={'code':'physics_error','message':'Physics stopped. Reset the scene.'}
            time.sleep(.01)

class Handler(SimpleHTTPRequestHandler):
    session=None
    def __init__(self,*args,**kw):super().__init__(*args,directory=str(ROOT/'public'),**kw)
    def log_message(self,*args):pass
    def reply(self,status,data):
        raw=json.dumps(data,allow_nan=False).encode()
        self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(raw)));self.end_headers()
        try:self.wfile.write(raw)
        except (BrokenPipeError,ConnectionResetError):pass
    def allowed(self):
        hosts=(f'127.0.0.1:{PORT}',f'localhost:{PORT}')
        return self.headers.get('Host') in hosts and self.headers.get('Origin') in (None,*(f'http://{h}' for h in hosts))
    def do_GET(self):
        if not self.allowed():return self.reply(403,{'error':'Local origin only'})
        if urlsplit(self.path).path=='/api/state':return self.reply(200,self.session.state())
        if urlsplit(self.path).path.startswith('/api/'):return self.reply(404,{'error':'Unknown API route'})
        super().do_GET()
    def list_directory(self,path):return self.reply(404,{'error':'Not found'})
    def do_POST(self):
        if not self.allowed():return self.reply(403,{'error':'Local origin only'})
        if self.path!='/api/command':return self.reply(404,{'error':'Not found'})
        if self.headers.get('Content-Type','').split(';')[0]!='application/json':return self.reply(415,{'error':'JSON required'})
        try:
            n=int(self.headers.get('Content-Length','0'))
            if not 0<n<=4096:return self.reply(413,{'error':'Invalid body size'})
            data=json.loads(self.rfile.read(n))
            if not isinstance(data,dict):raise ValueError('Expected JSON object')
            self.session.command(data);self.reply(200,self.session.state())
        except (ValueError,TypeError) as exc:self.reply(400,{'error':str(exc)})

if __name__=='__main__':
    session=Session();Handler.session=session
    threading.Thread(target=session.loop,daemon=True).start()
    print(f'MOSS live simulation: http://127.0.0.1:{PORT} | API credential: {"ready" if session.policy.key else "missing"}',flush=True)
    ThreadingHTTPServer(('127.0.0.1',PORT),Handler).serve_forever()
