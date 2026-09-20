/** Local playback only: no network, physics engine, or API calls. */
export class ReplayPlayer {
  constructor(){this.data=null;this.time=0;this.running=false;}
  load(data){
    if(!Array.isArray(data.frames)||data.frames.length<2||!Number.isFinite(data.duration)||data.duration<=0)throw new Error('Invalid replay');
    if(data.frames[0].t!==0||data.frames.at(-1).t!==data.duration||data.frames.some((f,i)=>!Number.isFinite(f.t)||(i&&f.t<=data.frames[i-1].t)))throw new Error('Invalid replay timeline');
    this.data=data;this.reset();
  }
  play(){if(!this.data)return;if(this.time>=this.data.duration)this.time=0;this.running=true;}
  pause(){this.running=false;}
  reset(){this.time=0;this.running=false;}
  seek(time){if(!this.data)return;this.time=Math.max(0,Math.min(this.data.duration,Number(time)||0));if(this.time===this.data.duration)this.running=false;}
  advance(seconds){if(this.running&&Number.isFinite(seconds)&&seconds>0)this.seek(this.time+seconds);}
  pair(){
    if(!this.data)return null;
    const frames=this.data.frames;let lo=0,hi=frames.length-1;
    while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(frames[mid].t<=this.time)lo=mid;else hi=mid-1;}
    const a=frames[lo],b=frames[Math.min(lo+1,frames.length-1)];
    return {a,b,t:b.t===a.t?0:(this.time-a.t)/(b.t-a.t),index:lo};
  }
}
