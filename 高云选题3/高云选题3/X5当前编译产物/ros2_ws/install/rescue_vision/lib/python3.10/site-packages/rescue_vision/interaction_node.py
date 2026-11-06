import json
from .node_common import Node
from std_msgs.msg import String
from .node_common import param,read_json,publish_json,spin
from .interaction import InteractionRouter,scene_sentence
from .semantics import SceneMemory


class Interaction(Node):
    def __init__(self):
        super().__init__('interaction')
        buildings=read_json(param(self,'buildings_config',''))
        self.router=InteractionRouter(buildings,param(self,'navigation_execute_enabled',False))
        self.resolver=SceneMemory(buildings);self.scene=None
        self.mission=self.create_publisher(String,'/mission/request',10)
        self.navigation=self.create_publisher(String,'/navigation/request',10)
        self.speech=self.create_publisher(String,'/speech/say',10)
        self.status=self.create_publisher(String,'/interaction/result',10)
        self.create_subscription(String,'/interaction/command',self.receive,10)
        self.create_subscription(String,'/scene/report',self.scene_update,10)
    def scene_update(self,msg):
        try:self.scene=json.loads(msg.data)
        except ValueError:self.scene=None
    def say(self,text):self.speech.publish(String(data=text))
    def receive(self,msg):
        now=self.get_clock().now().nanoseconds/1e9
        try:
            request=json.loads(msg.data)
            if isinstance(request,dict) and request.get('action')=='navigate' and not request.get('building_id'):
                resolved=self.resolver.resolve_request(request.get('text',''),now)
                if not resolved.get('building_id'):raise ValueError('楼栋不明确，请说一个楼号或完整楼名')
                request['building_id']=resolved['building_id']
            result=self.router.route(request,now)
            if result['accepted']:
                # Stop/observe first; later navigation proposals still pass mission arbitration.
                if result['mission']:publish_json(self.mission,result['mission'])
                if result['navigation']:publish_json(self.navigation,result['navigation'])
                if result['speech']:self.say(result['speech'])
                if result['report_scene']:self.say(scene_sentence(self.scene,now))
            publish_json(self.status,result)
        except (ValueError,TypeError,KeyError) as exc:
            publish_json(self.status,dict(accepted=False,reason=str(exc),stamp=now))

def main():spin(Interaction)
