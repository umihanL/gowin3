"""Map-based route preview, odometry interface and guarded maneuver proposals.

No ROS imports; this module neither drives motors nor assumes OCR gives map pose.
"""
import heapq
import math


def wrap(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


class RoadGraph:
    def __init__(self,data):
        if data.get('units')!='m':raise ValueError('Map units must be metres')
        self.data=data
        self.nodes={n['id']:n for n in data['nodes']}
        self.edges={e['id']:e for e in data['edges']}
        self.buildings={b['id']:b for b in data['buildings']}
        self.adj={n:[] for n in self.nodes}
        for edge in self.edges.values():
            if edge['length_m']<=0:raise ValueError('Nonpositive edge')
            for a,b in ((edge['source'],edge['target']),(edge['target'],edge['source'])):
                self.adj[a].append((b,edge['id']))
        self.blocked=set(data.get('default_blocked_edges',[]))

    def resolve_building(self,value):
        text=str(value).strip()
        if text.isdigit():text=f'{int(text):02d}'
        for b in self.buildings.values():
            if text in (b['id'],b.get('legacy_id'),b['name']):return b
        raise ValueError('Unknown building: '+text)

    def nearest_node(self,xy):
        name=min(self.nodes,key=lambda n:math.dist(xy,self.nodes[n]['xy_m']))
        return name,math.dist(xy,self.nodes[name]['xy_m'])

    def block(self,edge_ids):
        ids=set(edge_ids)
        if ids-self.edges.keys():raise ValueError('Unknown blocked edge')
        changed=bool(ids-self.blocked)
        self.blocked.update(ids)
        return changed

    def plan(self,start,building_id,initial_yaw=None):
        if start not in self.nodes:raise ValueError('Unknown start node')
        building=self.resolve_building(building_id)
        goals={p['node_id']:p for p in building['observation_points']}
        distance={start:0.};previous={};queue=[(0.,start)]
        goal=None
        while queue:
            cost,node=heapq.heappop(queue)
            if cost!=distance[node]:continue
            if node in goals:goal=node;break
            for nxt,edge_id in self.adj[node]:
                if edge_id in self.blocked:continue
                edge=self.edges[edge_id]
                geometry=edge['geometry_m'] if edge['source']==node else list(reversed(edge['geometry_m']))
                yaw=math.atan2(geometry[1][1]-geometry[0][1],geometry[1][0]-geometry[0][0])
                # A forward-only Ackermann controller cannot instantly reverse heading.
                if node==start and initial_yaw is not None and abs(wrap(yaw-initial_yaw))>math.radians(135):continue
                candidate=cost+edge['length_m']
                if candidate<distance.get(nxt,math.inf):
                    distance[nxt]=candidate;previous[nxt]=(node,edge_id);heapq.heappush(queue,(candidate,nxt))
        if goal is None:raise ValueError('No open route to building observation point')
        sequence=[];node=goal
        while node!=start:
            prev,eid=previous[node];sequence.append((prev,node,eid));node=prev
        sequence.reverse();segments=[];last_yaw=initial_yaw
        for a,b,eid in sequence:
            edge=self.edges[eid]
            points=list(edge['geometry_m']) if edge['source']==a else list(reversed(edge['geometry_m']))
            begin=math.atan2(points[1][1]-points[0][1],points[1][0]-points[0][0])
            end=math.atan2(points[-1][1]-points[-2][1],points[-1][0]-points[-2][0])
            angle=wrap(begin-last_yaw) if last_yaw is not None else 0.
            turn='left' if angle>.45 else 'right' if angle<-.45 else 'straight'
            current=dict(edge_id=eid,constituent_edge_ids=[eid],road_id=edge.get('road_id'),from_node=a,to_node=b,
                         geometry_m=points,length_m=edge['length_m'],turn=turn,turn_angle_rad=angle,
                         speed_limit_m_s=edge.get('speed_limit_m_s',.15))
            # Observation-only split points do not require a separate stop/turn action.
            # Keep every genuine branching junction as a maneuver boundary.
            if segments and len(self.adj[a])==2 and abs(angle)<.1 and segments[-1]['road_id']==current['road_id']:
                old=segments[-1];old['constituent_edge_ids'].append(eid)
                old['edge_id']='+'.join(old['constituent_edge_ids'])
                old['to_node']=b;old['geometry_m']+=points[1:];old['length_m']+=edge['length_m']
                old['speed_limit_m_s']=min(old['speed_limit_m_s'],current['speed_limit_m_s'])
            else:segments.append(current)
            last_yaw=end
        return dict(building_id=building['id'],building_name=building['name'],start_node=start,
                    goal_node=goal,length_m=distance[goal],segments=segments,observation=goals[goal],
                    map_frame='map',observation_pose_verified=False)


class Odometry:
    def __init__(self,config):
        self.config=config
        self.x,self.y,self.yaw=map(float,config.get('initial_pose_xy_yaw',[0,0,0]))
        self.initialized=bool(config.get('initial_pose_verified'))
        self.valid=False;self.reason='initial_pose_unverified'
        self.last_mcu=None;self.last_receipt=None;self.total_distance=0.
        self.uncertainty=.015;self.velocity=0.;self.vehicle_stopped=False

    def update(self,t,now):
        stamp=t.get('stamp');mcu=t.get('mcu_time_ms');ds=t.get('delta_s_m');rate=t.get('yaw_rate_rad_s')
        if not finite(stamp) or not 0<=now-stamp<=self.config.get('max_telemetry_age_s',.4):
            self.valid=False;self.reason='stale_telemetry';return False
        if not isinstance(mcu,int) or isinstance(mcu,bool) or not 0<=mcu<2**32:
            self.valid=False;self.reason='invalid_mcu_timestamp';return False
        if not t.get('odometry_valid') or t.get('gyro_valid') is not True or not finite(ds) or not finite(rate):
            self.valid=False;self.reason='invalid_odometry';return False
        if self.last_receipt is not None and stamp<=self.last_receipt:
            self.valid=False;self.reason='repeated_receipt';return False
        if self.last_mcu is None:
            self.last_mcu=mcu;self.last_receipt=stamp
            self.valid=self.initialized;self.reason='odometry_primed' if self.valid else 'initial_pose_unverified'
            return False
        dt=((mcu-self.last_mcu)&0xffffffff)/1000.
        if not 0<dt<=self.config.get('max_mcu_gap_s',.4):
            self.valid=False;self.reason='mcu_gap_reset_or_replay'
            # After a real telemetry gap, lost wheel motion cannot be recovered from IMU.
            if dt>self.config.get('max_mcu_gap_s',.4):self.initialized=False
            return False
        if abs(ds)>self.config.get('max_speed_m_s',.6)*dt+.005 or abs(rate)>self.config.get('max_yaw_rate_rad_s',4.):
            self.valid=False;self.reason='implausible_motion';self.initialized=False;return False
        delta=(rate-self.config.get('gyro_bias_rad_s',0.))*dt
        mid=self.yaw+delta/2
        self.x+=ds*math.cos(mid);self.y+=ds*math.sin(mid)
        self.yaw=wrap(self.yaw+delta)
        # Only a verified absolute heading in map frame can correct gyro drift.
        if self.config.get('absolute_heading_in_map_frame_verified') and t.get('yaw_valid') and finite(t.get('yaw_rad')):
            self.yaw=wrap(self.yaw+self.config.get('heading_correction_gain',.15)*wrap(t['yaw_rad']-self.yaw))
        self.total_distance+=abs(ds);self.velocity=ds/dt
        self.uncertainty+=abs(ds)*self.config.get('odometry_fractional_drift',.025)+dt*self.config.get('uncertainty_growth_m_s',.002)
        self.last_mcu=mcu;self.last_receipt=stamp;self.vehicle_stopped=bool(t.get('vehicle_stopped'))
        self.valid=self.initialized and self.uncertainty<=self.config.get('max_position_uncertainty_m',.25)
        self.reason='dead_reckoning_with_drift_bound' if self.valid else 'localization_requires_verified_reset'
        return True

    def pose_fixture(self,event,now):
        fixture=self.config.get('registered_pose_fixtures',{}).get(event.get('landmark_id'))
        if not fixture or not fixture.get('verified') or event.get('measurement_type')!='fixed_pose_fixture':return False
        if not event.get('vehicle_stopped') or not event.get('observation_verified'):return False
        if not finite(event.get('stamp')) or not 0<=now-event['stamp']<=.4:return False
        if event.get('registration_id')!=self.config.get('map_registration_id'):return False
        # A fixture identifies measured vehicle position AND heading; reading a sign alone cannot call this.
        self.x,self.y,self.yaw=map(float,fixture['xy_yaw'])
        self.uncertainty=float(fixture.get('uncertainty_m',.015));self.initialized=True
        self.last_mcu=None;self.last_receipt=None;self.valid=False;self.reason='fixture_reset_waiting_telemetry'
        return True

    def fresh(self,now):
        return self.valid and self.last_receipt is not None and 0<=now-self.last_receipt<=self.config.get('max_telemetry_age_s',.4)

    def state(self,now):
        return dict(valid=self.fresh(now),x_m=self.x,y_m=self.y,yaw_rad=self.yaw,
                    uncertainty_m=self.uncertainty,reason=self.reason,source='encoder_plus_gyro_dead_reckoning',
                    freshness_s=None if self.last_receipt is None else max(0.,now-self.last_receipt),
                    world_pose_from_ocr=False)


class Navigator:
    def __init__(self,graph,config):
        self.graph=RoadGraph(graph);self.config=config;self.odom=Odometry(config)
        self.route=None;self.command_id=None;self.mode='preview';self.edge_index=0
        self.revision=0;self.edge_start_distance=0.;self.paused=False;self.arrived=False
        self.reason='idle';self.telemetry={};self.last_request_ids=set();self.execution_authorized=False

    def request(self,request,now):
        command_id=str(request.get('command_id',''))
        if not command_id or command_id in self.last_request_ids:raise ValueError('Missing or repeated command_id')
        issued=request.get('issued_at');ttl=request.get('ttl_s',10.)
        if not finite(issued) or not finite(ttl) or not 0<ttl<=300 or not 0<=now-issued<=ttl:
            raise ValueError('Expired or invalid request timestamp')
        self.last_request_ids.add(command_id)
        if len(self.last_request_ids)>1024:self.last_request_ids={command_id}
        action=request.get('action','navigate')
        if action=='pause':self.paused=True;self.reason='paused';return
        if action=='resume':self.paused=False;self.reason='resumed_waiting_gate';return
        if action=='cancel':self.route=None;self.command_id=None;self.execution_authorized=False;self.reason='cancelled';return
        if action!='navigate':raise ValueError('Unsupported navigation action')
        mode=request.get('mode','preview')
        if mode not in ('preview','execute'):raise ValueError('Invalid mode')
        if 'blocked_edge_ids' in request:self.graph.block(request['blocked_edge_ids'])
        self.execution_authorized=mode=='execute' and request.get('confirmed') is True
        if mode=='execute':
            if not self.odom.fresh(now):
                raise ValueError('Execution requires fresh verified localization at request time; submit again after initialization')
            start,dist=self.graph.nearest_node([self.odom.x,self.odom.y])
            if dist>.15:raise ValueError('Execution must start at a confirmed graph node')
            yaw=self.odom.yaw
        else:
            start=self.graph.data['preview_start_node'];yaw=self.graph.data.get('preview_start_yaw_rad')
        self.route=self.graph.plan(start,request['building_id'],yaw)
        self.command_id=command_id;self.mode=mode;self.edge_index=0;self.revision+=1
        self.edge_start_distance=self.odom.total_distance;self.paused=False;self.arrived=False
        self.reason='route_preview' if mode=='preview' else 'waiting_execution_gate'

    def update_telemetry(self,t,now):
        accepted=self.odom.update(t,now)
        if not accepted:return
        self.telemetry=t
        if not self.route or self.mode!='execute' or not self.execution_authorized or self.paused:return
        if not self.gates(now)[0] or self.edge_index>=len(self.route['segments']):return
        edge=self.route['segments'][self.edge_index]
        matched=(t.get('maneuver_done') is True and t.get('maneuver_command_id')==self.command_id and
                 t.get('maneuver_edge_id')==edge['edge_id'] and t.get('maneuver_route_revision')==self.revision)
        if not matched:return
        minimum=edge['length_m']*self.config.get('edge_distance_min_ratio',.8)
        endpoint=self.graph.nodes[edge['to_node']]['xy_m']
        if self.odom.total_distance-self.edge_start_distance<minimum or math.dist([self.odom.x,self.odom.y],endpoint)>self.config.get('edge_endpoint_tolerance_m',.35):
            self.paused=True;self.reason='maneuver_done_inconsistent_with_odometry';return
        final=self.edge_index==len(self.route['segments'])-1
        if final and not t.get('vehicle_stopped'):
            self.reason='waiting_final_stop_ack';return
        self.edge_index+=1;self.edge_start_distance=self.odom.total_distance
        self.arrived=self.edge_index==len(self.route['segments'])
        self.reason='arrived_with_completion_and_stop_ack' if self.arrived else 'next_edge'

    def gates(self,now):
        if not self.config.get('map_registration_verified') or not self.config.get('map_registration_id'):return False,'map_registration_unverified'
        if not self.config.get('execution_adapter_verified'):return False,'fpga_execution_adapter_unverified'
        if not self.execution_authorized:return False,'goal_not_authorized_for_execution'
        if not self.odom.fresh(now):return False,'localization_not_fresh_or_verified'
        if self.telemetry.get('estop'):return False,'estop'
        if self.telemetry.get('line_valid') is not True:return False,'line_not_valid'
        obstacle=self.telemetry.get('obstacle_distance_m')
        if finite(obstacle) and 0<obstacle<self.config.get('obstacle_stop_distance_m',.22):return False,'obstacle_stop_required'
        return True,'ready'

    def block_edges(self,ids,now):
        changed=self.graph.block(ids)
        if not changed or not self.route:return False
        if self.mode=='execute':
            start,dist=self.graph.nearest_node([self.odom.x,self.odom.y])
            if not self.odom.fresh(now) or dist>.15 or not self.telemetry.get('vehicle_stopped'):
                self.paused=True;self.reason='blocked_route_requires_confirmed_stopped_node';return False
            yaw=self.odom.yaw
        else:start=self.route['start_node'];yaw=self.graph.data.get('preview_start_yaw_rad')
        self.route=self.graph.plan(start,self.route['building_id'],yaw)
        self.edge_index=0;self.edge_start_distance=self.odom.total_distance;self.revision+=1;self.arrived=False
        self.reason='replanned_after_new_block';return True

    def proposal(self,now):
        ok,reason=self.gates(now)
        result=dict(stamp=now,command_id=self.command_id,building_id=self.route['building_id'] if self.route else None,
                    mode=self.mode,route_revision=self.revision,edge_id=None,edge_index=self.edge_index,
                    turn='straight',speed_limit_m_s=0.,distance_m=0.,stop_distance_m=0.,requires_stop=True,
                    ready_to_move=False,valid=False,arrived=self.arrived,reason=reason,
                    freshness_s=self.odom.state(now)['freshness_s'])
        if not self.route:result['reason']=self.reason;return result
        if self.mode=='preview':result['reason']='preview_only';return result
        if self.paused:result['reason']=self.reason;return result
        if self.arrived:
            result.update(valid=ok and self.telemetry.get('vehicle_stopped') is True,
                          arrived=ok and self.telemetry.get('vehicle_stopped') is True,
                          arrival_achieved_previously=True,reason=self.reason if ok else reason)
            return result
        if not self.route['segments']:
            # Being nearest to the target coordinate is insufficient evidence of arrival.
            result['reason']='already_at_goal_requires_stationary_observation_verification';return result
        edge=self.route['segments'][self.edge_index]
        if any(e in self.graph.blocked for e in edge['constituent_edge_ids']):ok=False;reason='current_edge_blocked'
        if abs(edge['turn_angle_rad'])>math.radians(135):ok=False;reason='ackermann_uturn_not_supported'
        final=self.edge_index==len(self.route['segments'])-1
        result.update(edge_id=edge['edge_id'],turn=edge['turn'],distance_m=edge['length_m'],
                      stop_distance_m=edge['length_m'] if final else None,requires_stop=final or not ok,
                      speed_limit_m_s=min(self.config.get('speed_limit_m_s',.15),edge['speed_limit_m_s']) if ok else 0.,
                      ready_to_move=ok,valid=ok,reason=reason)
        return result

    def state(self,now):
        return dict(stamp=now,command_id=self.command_id,mode=self.mode,paused=self.paused,arrived=self.arrived,
                    route_revision=self.revision,edge_index=self.edge_index,route=self.route,reason=self.reason,
                    blocked_edges=sorted(self.graph.blocked),localization=self.odom.state(now),
                    map_registration_verified=bool(self.config.get('map_registration_verified')),
                    proposal=self.proposal(now))
