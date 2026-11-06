"""Geometry and measured progress for one retained FPGA route.

Coordinates are the FPGA path RAM's image millimetres. The vehicle reference
stays its nose midpoint. Region waypoints are route references only: passing
one neither requires a straight stopping corridor nor commands a stop.
"""
import hashlib
import json
import math


def angle_error(a, b):
    return abs((a - b + 180.) % 360. - 180.)


def heading(a, b):
    return math.degrees(math.atan2(a[1] - b[1], b[0] - a[0]))


def checked_route(route):
    points, count = route.get('points_mm'), route.get('total_count')
    generation = route.get('generation')
    if (route.get('valid') is not True or route.get('sampled') is not False or
            type(generation) is not int or not 1 <= generation <= 65534 or
            type(count) is not int or not 2 <= count <= 2048 or
            not isinstance(points, list) or len(points) != count or
            route.get('sample_indices', list(range(count))) != list(range(count))):
        raise ValueError('continuous_complete_route_required')
    for point in points:
        if (not isinstance(point, (list, tuple)) or len(point) != 2 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in point) or
                not 0 <= point[0] <= 3520 or not 0 <= point[1] <= 3200):
            raise ValueError('continuous_route_point_invalid')
    length = [0.]
    for a, b in zip(points, points[1:]):
        step = math.dist(a, b)
        if step > 180.:
            raise ValueError('continuous_route_segment_gap')
        length.append(length[-1] + step)
    if length[-1] < 20.:
        raise ValueError('continuous_route_empty')
    return points, length


def waypoint_references(route, waypoints):
    """Associate each reference with its ordered occurrence on the real path."""
    points, lengths = checked_route(route)
    anchors, previous = [], 0
    for waypoint in waypoints:
        center = [v * 20 + 10 for v in waypoint['cell']]
        # FPGA intermediate WAYPOINT accepts a 20 cm junction neighbourhood;
        # the executed front-midpoint arc may legitimately miss the cell by
        # more than the 6 cm terminal GOAL radius. Match that real path first.
        candidates = [i for i in range(previous, len(points)) if math.dist(points[i], center) <= 200.]
        if not candidates:
            raise ValueError('continuous_checkpoint_not_on_route:' + waypoint['id'])
        # The first near-pass is one contiguous visit. Do not jump to a closer
        # point on a later lap through the same road crossing.
        first = candidates[0]
        visit = [i for i in candidates if lengths[i] - lengths[first] <= 400.]
        anchor = min(visit, key=lambda i: (math.dist(points[i], center), i))
        anchors.append(anchor)
        previous = anchor + 1
    references = []
    for number, (waypoint, anchor) in enumerate(zip(waypoints, anchors)):
        references.append(dict(id=waypoint['id'], waypoint_index=number, route_index=anchor,
            progress_mm=lengths[anchor], point_mm=list(points[anchor]), reference_only=True))
    return points, lengths, references


def project_progress(points, lengths, point, execution_index, previous_index=0, previous_mm=0.):
    """Project a real pose locally; the FPGA target index is only a search bound.

    The target index is a lookahead pointer, never evidence of distance driven.
    A local monotonic window prevents a repeated crossing jumping to a later lap.
    """
    if type(execution_index) is not int or not 0 <= execution_index < len(points):
        raise ValueError('continuous_execution_index_invalid')
    start = max(0, previous_index - 2)
    end = min(len(points) - 1, max(previous_index + 2, execution_index + 1), previous_index + 64)
    candidates = []
    for index in range(start, end):
        a, b = points[index:index + 2]
        dx, dy = b[0] - a[0], b[1] - a[1]
        square = dx * dx + dy * dy
        if square <= .01:
            continue
        fraction = max(0., min(1., ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / square))
        projected = [a[0] + fraction * dx, a[1] + fraction * dy]
        progress = lengths[index] + fraction * math.sqrt(square)
        if progress + 50. < previous_mm:
            continue
        distance = math.dist(point, projected)
        candidates.append((distance + .01 * max(0., progress - previous_mm), index, progress, distance))
    if not candidates:
        raise ValueError('continuous_pose_has_no_route_projection')
    _, index, progress, distance = min(candidates)
    if distance > 120.:
        raise ValueError('continuous_pose_too_far_from_retained_route')
    return index, max(previous_mm, progress), distance


def forward_proposal(route, reference, execution_index, progress_index, progress_mm, now,
                     max_distance_cm=30.):
    """Expose a bounded straight prefix of the existing route, without planning."""
    base = dict(stamp=now, valid=False, road_path_verified=False,
                reason='checkpoint_alignment_requires_retained_forward_route')
    if (not isinstance(reference, dict) or reference.get('valid') is not True or
            type(max_distance_cm) not in (int, float) or not math.isfinite(max_distance_cm)):
        return base
    try:
        points, lengths = checked_route(route)
        cell, direction = reference['cell'], reference['heading_deg']
        position = reference.get('image_mm') or [v * 20 + 10 for v in cell]
        index, progress, _ = project_progress(points, lengths, position, execution_index, progress_index, progress_mm)
    except (KeyError, TypeError, ValueError):
        return base
    # Preserve room before the terminal exit so an explicit fire-alignment
    # movement cannot consume DONE before the fire action finishes.
    maximum = min(min(30., max_distance_cm) * 10., lengths[-1] - progress - 100.)
    angle = math.radians(direction)
    ux, uy = math.cos(angle), -math.sin(angle)
    # Test the route's own segment tangent. A 3 mm lateral quantization error
    # against a point only 9 mm ahead must not masquerade as a 20-degree turn.
    last, previous = None, points[index]
    for point_index in range(index + 1, len(points)):
        point = points[point_index]
        dx, dy = point[0] - position[0], point[1] - position[1]
        forward, lateral = dx * ux + dy * uy, abs(-dx * uy + dy * ux)
        length = lengths[point_index] - progress
        if (length > maximum or forward < -2. or lateral > 60. or
                (math.dist(previous, point) > 1. and angle_error(heading(previous, point), direction) > 15.)):
            break
        if length >= 120. and forward >= 100.:
            last = (point_index, point, length)
        previous = point
    if last is None:
        return dict(base, reason='checkpoint_alignment_no_straight_retained_route_budget')
    end_index, point, length = last
    return dict(base, valid=True, road_path_verified=True,
        reason='checkpoint_alignment_retained_route_forward_proposed',
        goal_cell=[int(v // 20) for v in point], from_pose_cell=list(cell),
        from_heading_deg=direction, path_length_cm=round(length / 10., 3),
        retained_route=True, route_generation=route['generation'],
        stop_progress_mm=lengths[end_index], stop_route_index=end_index,
        alignment_guard=dict(start_cell=list(cell), heading_deg=direction,
            max_advance_cm=min(30., max_distance_cm), lateral_tolerance_cm=6.,
            heading_tolerance_deg=15., speed_mm_s=60))


def local_coordinates(anchor, image_mm):
    """Signed forward/lateral offset; intentionally permits reverse travel."""
    radians = math.radians(anchor['heading_deg'])
    dx, dy = image_mm[0]-anchor['image_mm'][0], image_mm[1]-anchor['image_mm'][1]
    return dx*math.cos(radians)-dy*math.sin(radians), dx*math.sin(radians)+dy*math.cos(radians)


def _route_only_corridor(route, anchor, execution_index, progress_index, progress_mm,
                         maximum_mm=600.):
    """Find the straight route corridor around a stopped fire-patrol anchor.

    This does not advance global route progress. Bounds may be asymmetric, and
    never extend through a turn, across a different visit, or beyond the route.
    """
    points, lengths = checked_route(route)
    index, progress, _ = project_progress(points, lengths, anchor['image_mm'],
        execution_index, progress_index, progress_mm)
    lo = hi = 0.
    stop_lo = stop_hi = 0.
    for direction in (-1, 1):
        previous = points[index+1] if direction < 0 else points[index]
        indices = range(index, -1, -1) if direction < 0 else range(index+1, len(points))
        bound = 0.
        for cursor in indices:
            point = points[cursor]
            forward, lateral = local_coordinates(anchor, point)
            segment_heading = heading(point, previous) if direction < 0 else heading(previous, point)
            if (abs(lateral) > 50. or direction*forward < -20. or
                    (math.dist(point, previous) > 1. and angle_error(segment_heading, anchor['heading_deg']) > 12.)):
                break
            bound = max(bound, min(maximum_mm, direction*forward))
            previous = point
            if bound >= maximum_mm:
                break
        # Leave room for the physical stop before a turn/route boundary. At a
        # full +/-600 mm span the per-command distance bound adds the same margin.
        safe_bound = max(0., bound-20.)
        if direction < 0:
            lo = -safe_bound
            stop_lo = -bound
        else:
            hi = safe_bound
            stop_hi = bound
    return dict(backward_mm=lo, forward_mm=hi, route_index=index,
                stop_bounds_mm=[stop_lo,stop_hi],
                anchor_route_progress_mm=progress, lateral_limit_mm=40., heading_limit_deg=8.)


def _checked_free_grid(document, map_signature, map_sha256):
    if (document.get('schema_version')!=1 or document.get('width_cells')!=176 or
            document.get('height_cells')!=160 or document.get('cell_cm')!=2 or
            document.get('free_value')!=0 or document.get('reference')!='nose_midpoint' or
            type(document.get('planner_inflation_m')) not in (int,float) or
            not 0<document['planner_inflation_m']<=.5 or not isinstance(document.get('free_runs'),list)):
        raise ValueError('centering_traversability_invalid')
    if (type(map_signature) is not int or document.get('map_signature')!=map_signature or
            map_sha256 is not None and document.get('map_sha256')!=map_sha256):
        raise ValueError('centering_traversability_map_identity_mismatch')
    cells=bytearray([1]*(176*160))
    for run in document['free_runs']:
        if (not isinstance(run,list) or len(run)!=3 or any(type(value) is not int for value in run)
                or not 0<=run[0]<160 or not 0<=run[1]<176 or not 1<=run[2]<=176-run[1]):
            raise ValueError('centering_traversability_run_invalid')
        start=run[0]*176+run[1]
        if 0 in cells[start:start+run[2]]:
            raise ValueError('centering_traversability_overlapping_runs')
        cells[start:start+run[2]]=bytes(run[2])
    if hashlib.sha256(cells).hexdigest()!=document.get('cells_sha256'):
        raise ValueError('centering_traversability_hash_mismatch')
    nodes=document.get('directed_nodes')
    branches=[]
    if nodes is not None:
        if (not isinstance(nodes,list) or hashlib.sha256(json.dumps(nodes,separators=(',',':')).encode('ascii')).hexdigest()
                !=document.get('directed_nodes_sha256')):
            raise ValueError('centering_route_graph_identity_mismatch')
        for row in nodes:
            if (not isinstance(row,list) or len(row)!=4 or any(type(value) is not int for value in row[:3])
                    or not isinstance(row[3],list)):
                raise ValueError('centering_route_graph_invalid')
            if len(row[3])>1: branches.append([row[0]/25.6,row[1]/25.6])
    return cells,branches


def _segment_cells(a, b):
    """All 2 cm cells touched by a line, including grid-corner neighbours."""
    cuts={0.,1.}
    for axis in (0,1):
        delta=b[axis]-a[axis]
        if abs(delta)<1e-9: continue
        for edge in range(math.floor(min(a[axis],b[axis])/20)+1,math.floor(max(a[axis],b[axis])/20)+1):
            t=(edge*20-a[axis])/delta
            if 0<t<1: cuts.add(t)
    ordered=sorted(cuts)
    probes=ordered+[(left+right)/2 for left,right in zip(ordered,ordered[1:])]
    cells=set()
    for fraction in probes:
        x=a[0]+fraction*(b[0]-a[0]);y=a[1]+fraction*(b[1]-a[1])
        for dx in (-1e-7,1e-7):
            for dy in (-1e-7,1e-7):cells.add((math.floor((x+dx)/20),math.floor((y+dy)/20)))
    return cells


def _segment_hits_box(a,b,box):
    lower,upper=0.,1.
    for axis in (0,1):
        delta=b[axis]-a[axis]
        if abs(delta)<1e-9:
            if not box[axis]<=a[axis]<=box[axis+2]: return False
            continue
        first=(box[axis]-a[axis])/delta;last=(box[axis+2]-a[axis])/delta
        lower=max(lower,min(first,last));upper=min(upper,max(first,last))
        if lower>upper:return False
    return True


def grid_line_reason(cells, start, end, *, barriers=None, inflation_mm=0.):
    """Validate an actual short segment against an already verified grid."""
    for x,y in _segment_cells(start,end):
        if not 0<=x<176 or not 0<=y<160 or cells[y*176+x]!=0:
            return 'centering_corridor_static_obstacle'
    if isinstance(barriers,dict) and barriers.get('valid') is True:
        for line in barriers.get('segments_mm',[]):
            if not isinstance(line,(list,tuple)) or len(line)!=4 or any(type(v) is not int for v in line):
                return 'centering_live_barrier_invalid'
            box=[min(line[0],line[2])-inflation_mm,min(line[1],line[3])-inflation_mm,
                 max(line[0],line[2])+inflation_mm,max(line[1],line[3])+inflation_mm]
            if _segment_hits_box(start,end,box):return 'centering_corridor_live_barrier'
    return ''


def _project_span(points, lengths, point, lo, hi):
    best=None
    for cursor in range(lo,hi):
        a,b=points[cursor:cursor+2];dx,dy=b[0]-a[0],b[1]-a[1];square=dx*dx+dy*dy
        if square<.01:continue
        fraction=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/square))
        distance=math.dist(point,[a[0]+fraction*dx,a[1]+fraction*dy])
        candidate=(distance,lengths[cursor]+fraction*math.sqrt(square))
        if best is None or candidate<best:best=candidate
    return best


def retained_span_line_reason(points, lengths, span, start, end, direction, *, verified_return_anchor=None):
    """Revalidate the actual driven line against the captured route visit."""
    lo,hi=span
    initial=_project_span(points,lengths,start,lo,hi)
    if initial is None or initial[0]>120.:return 'centering_retained_straight_span_ended'
    distance=math.dist(start,end)
    steps=max(1,math.ceil(distance/5.))
    # The pose is transmitted as integer X/Y millimetres. Returning to this
    # session's actually measured anchor can therefore put the integer jog's
    # full braking envelope up to sqrt(2) mm across an otherwise equal endpoint.
    # This exception applies only at that endpoint; the caller still checks
    # the complete, unshortened envelope against the inflated grid/barriers.
    anchor_endpoint=bool(verified_return_anchor is not None and direction and
        math.dist(end,verified_return_anchor)<=2.)
    for step in range(steps+1):
        point=[start[i]+(end[i]-start[i])*step/steps for i in (0,1)]
        lateral,progress=_project_span(points,lengths,point,lo,hi)
        endpoint_equal=anchor_endpoint and step==steps
        if (lateral>120. or direction<0 and progress<=lengths[lo]+.01 and not endpoint_equal or
                direction>0 and progress>=lengths[hi]-.01 and not endpoint_equal or
                direction and direction*(progress-initial[1])<.8*distance*step/steps):
            return 'centering_retained_straight_span_ended'
    return ''


def route_local_corridor(route, anchor, execution_index, progress_index, progress_mm,
                         maximum_mm=600., *, traversability=None, map_signature=None,
                         map_sha256=None, barriers=None):
    """Check the real vehicle's straight jog within its existing route visit.

    The inflated free grid permits a small initial centreline/yaw discrepancy;
    movement still projects onto this one route span and cannot cross its turn,
    branch, end, wall, or known inflated live barrier. No new route is planned.
    """
    if traversability is None:
        return dict(_route_only_corridor(route,anchor,execution_index,progress_index,progress_mm,maximum_mm),
                    geometry_verified=False,geometry_source='conservative_route_only',reason='centering_route_only_corridor')
    points,lengths=checked_route(route)
    index,_,_=project_progress(points,lengths,anchor['image_mm'],execution_index,progress_index,progress_mm)
    base=dict(backward_mm=0.,forward_mm=0.,stop_bounds_mm=[0.,0.],route_index=index,lateral_limit_mm=40.,heading_limit_deg=8.,
              geometry_verified=False,geometry_source='inflated_free_grid_and_retained_route')
    try: cells,branches=_checked_free_grid(traversability,map_signature,map_sha256)
    except ValueError as exc:return dict(base,reason=str(exc),backward_reason=str(exc),forward_reason=str(exc))
    origin=anchor['image_mm'];tangent=heading(points[index],points[index+1])
    def projection(point):
        return _project_span(points,lengths,point,lo,hi)
    lo,hi=index,index+1
    def branch(point):return any(math.dist(point,other)<=10. for other in branches)
    if any(math.dist(origin,point)<=20. for point in branches):
        return dict(base,reason='centering_anchor_at_route_branch',backward_reason='centering_anchor_at_route_branch',
                    forward_reason='centering_anchor_at_route_branch')
    for direction in (-1,1):
        while (lo>0 if direction<0 else hi<len(points)-1):
            boundary=lo if direction<0 else hi
            if branch(points[boundary]):break
            a,b=(points[lo-1],points[lo]) if direction<0 else (points[hi],points[hi+1])
            if math.dist(a,b)>1. and angle_error(heading(a,b),tangent)>8.:break
            if direction<0:lo-=1
            else:hi+=1
    anchor_distance,anchor_progress=projection(origin)
    base.update(anchor_route_progress_mm=anchor_progress,route_span_indices=[lo,hi],
        initial_route_distance_mm=anchor_distance,initial_route_heading_error_deg=angle_error(anchor['heading_deg'],tangent),
        cells_sha256=traversability['cells_sha256'],map_signature=map_signature,_free_cells=bytes(cells),
        planner_inflation_mm=traversability['planner_inflation_m']*1000.)
    boxes=[]
    if isinstance(barriers,dict) and barriers.get('valid') is True:
        inflate=traversability['planner_inflation_m']*1000.
        for line in barriers.get('segments_mm',[]):
            if not isinstance(line,(list,tuple)) or len(line)!=4 or any(type(v) is not int for v in line):
                return dict(base,reason='centering_live_barrier_invalid',backward_reason='centering_live_barrier_invalid',
                            forward_reason='centering_live_barrier_invalid')
            boxes.append([min(line[0],line[2])-inflate,min(line[1],line[3])-inflate,
                          max(line[0],line[2])+inflate,max(line[1],line[3])+inflate])
    def clear(endpoint):
        for x,y in _segment_cells(origin,endpoint):
            if not 0<=x<176 or not 0<=y<160 or cells[y*176+x]!=0:return 'centering_corridor_static_obstacle'
        if any(_segment_hits_box(origin,endpoint,box) for box in boxes):return 'centering_corridor_live_barrier'
        return ''
    initial=clear(origin)
    if initial or anchor_distance>120.:
        reason=initial or 'centering_anchor_outside_retained_route_corridor'
        return dict(base,reason=reason,backward_reason=reason,forward_reason=reason)
    angle=math.radians(anchor['heading_deg']);ux,uy=math.cos(angle),-math.sin(angle)
    for direction,key in ((-1,'backward'),(1,'forward')):
        bound=0.;reason='centering_local_distance_limit'
        for distance in range(5,int(min(maximum_mm,600.))+1,5):
            endpoint=[origin[0]+direction*distance*ux,origin[1]+direction*distance*uy]
            lateral,progress=projection(endpoint)
            if (lateral>120. or direction*(progress-anchor_progress)<.8*distance or
                    not lengths[lo]<=progress<=lengths[hi] or
                    direction<0 and progress<=lengths[lo]+.01 or direction>0 and progress>=lengths[hi]-.01):
                reason='centering_retained_straight_span_ended';break
            reason=clear(endpoint)
            if reason:break
            bound=float(distance)
        # Stop margin applies to the actual driven line, including a grid wall.
        base[key+'_mm']=direction*max(0.,bound-20.)
        # Keep the independently verified raw extent. Reconstructing it as
        # safe +/-20 would invent room when the measured raw bound is <20.
        base['stop_bounds_mm'][0 if direction<0 else 1]=direction*bound
        base[key+'_reason']=reason or 'centering_local_distance_limit'
    return dict(base,geometry_verified=True,reason='centering_actual_line_verified_on_retained_grid')
