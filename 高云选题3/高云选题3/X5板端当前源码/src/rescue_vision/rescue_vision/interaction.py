"""One validated, expiring intent entry for voice, web and typed commands."""
from collections import OrderedDict
import math
import re


class InteractionRouter:
    def __init__(self,buildings,execute_enabled=False):
        self.ids={b['building_id'] for b in buildings['buildings']}
        self.execute_enabled=execute_enabled;self.seen=OrderedDict();self.resume_mode='idle'
    def route(self,request,now):
        if not isinstance(request,dict):raise ValueError('Command must be a JSON object')
        cid=request.get('command_id');action=request.get('action');source=request.get('source')
        if not isinstance(cid,str) or not re.fullmatch(r'[A-Za-z0-9_:.\-]{1,100}',cid):raise ValueError('Invalid command_id')
        if source not in ('web','voice','text','test'):raise ValueError('Unknown command source')
        if action not in ('navigate','inspect','report','stop','resume'):raise ValueError('Unsupported action')
        stamp=request.get('issued_at');ttl=request.get('ttl_s',5.)
        if (not isinstance(stamp,(float,int)) or not isinstance(ttl,(float,int))
                or not math.isfinite(stamp) or not 0<ttl<=10 or not -.25<=now-stamp<=ttl):raise ValueError('Expired or future command')
        if cid in self.seen:return dict(accepted=False,reason='duplicate',command_id=cid)
        text=request.get('text','')
        if not isinstance(text,str) or len(text)>160:raise ValueError('Invalid command text')
        bid=request.get('building_id')
        if bid is not None and bid not in self.ids:raise ValueError('Unknown building')
        if action=='navigate' and bid is None and not text:raise ValueError('Navigation needs a building or text')
        self.seen[cid]=now
        while len(self.seen)>512:self.seen.popitem(last=False)
        out=dict(accepted=True,command_id=cid,source=source,action=action,mission=None,navigation=None,
                 report_scene=False,speech=None)
        if action=='navigate':
            self.resume_mode='navigate'
            out['navigation']=dict(command_id=cid,building_id=bid,text=text,issued_at=now,ttl_s=5.,
                mode='execute' if self.execute_enabled else 'preview',confirmed=True)
            out['mission']={'mode':'navigate'}
            out['speech']='正在规划观察路线。' if self.execute_enabled else '正在预览观察路线，实车导航尚未启用。'
        elif action=='stop':
            out['mission']={'mode':'idle'};out['navigation']={'action':'pause','command_id':cid,'issued_at':now,'ttl_s':5.}
            out['speech']='已请求停车。'
        elif action=='resume':
            out['navigation']={'action':'resume','command_id':cid,'issued_at':now,'ttl_s':5.}
            out['mission']={'mode':self.resume_mode}
            out['speech']='已请求恢复，正在检查定位和道路状态。'
        elif action=='inspect':
            self.resume_mode='inspect';out['mission']={'mode':'inspect'}
            out['navigation']={'action':'pause','command_id':cid,'issued_at':now,'ttl_s':5.}
            out['speech']='已请求停车观察。'
        else:out['report_scene']=True
        return out


def scene_sentence(report,now,max_age=3.):
    if not isinstance(report,dict) or not report.get('valid'):
        return '当前楼栋场景尚未确认，请停车观察楼牌和楼面。'
    stamp=report.get('stamp')
    if not isinstance(stamp,(int,float)) or not 0<=now-stamp<=max_age:
        return '场景信息已过期，需要重新观察。'
    number=report.get('building_number');name=report.get('building_name') or ''
    where=f'当前观察到{number}号{name}。' if number else '当前楼面。'
    fire={'observed':'发现火焰图案。','not_observed':'本次未检出火焰。','unknown':'火焰情况尚未确认。'}.get(report.get('fire_status'),'火焰情况尚未确认。')
    p=report.get('visible_person_count');f=report.get('visible_firefighter_count')
    counts=(f'当前画面可见{p}名普通人物，{f}名消防员。' if isinstance(p,int) and isinstance(f,int) and p>=0 and f>=0 else '人物数量尚未确认。')
    return where+fire+counts+'是否受困尚不能确定。'
