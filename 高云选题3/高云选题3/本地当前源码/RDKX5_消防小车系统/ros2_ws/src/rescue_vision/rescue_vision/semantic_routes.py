"""Translate 'turn at the Nth junction' into constraints for the FPGA planner.

This does not integrate odometry or drive the vehicle. The FPGA computes and
executes the actual directed curved route. A physically initialized nose pose
is required by the caller for every new route. No timed junction counting.
"""
import math


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def build_junction_route(catalog, reference, nth, direction):
    if catalog.get('schema_version') != 1:
        raise ValueError('unsupported_semantic_map')
    if type(nth) is not int or not 1 <= nth <= 8 or direction not in ('left', 'right'):
        raise ValueError('invalid_junction_instruction')
    if not isinstance(reference, dict) or not all(_number(reference.get(k)) for k in ('x_cm', 'y_cm', 'heading_deg')):
        raise ValueError('initialized_nose_pose_required')
    width, height = catalog['width_cm'], catalog['height_cm']
    px, py = float(reference['x_cm']), height-float(reference['y_cm'])
    if not 0 <= px < width or not 0 <= py < height:
        raise ValueError('start_outside_map')
    angle = math.radians(reference['heading_deg'])
    forward = (math.cos(angle), -math.sin(angle))  # image coordinates, y down
    points = catalog['vertices_image_cm']
    edges = catalog['straight_edges']
    adjacency = {name: [] for name in points}
    for a, b in edges:
        adjacency[a].append(b)
        adjacency[b].append(a)
    junctions = set(catalog['junctions'])
    candidates = []
    for a, b in edges:
        ax, ay = points[a]; bx, by = points[b]
        dx, dy = bx-ax, by-ay
        length = math.hypot(dx, dy)
        ux, uy = dx/length, dy/length
        along = (px-ax)*ux+(py-ay)*uy
        lateral = abs((px-ax)*uy-(py-ay)*ux)
        alignment = ux*forward[0]+uy*forward[1]
        if lateral <= 4 and 0 <= along <= length and abs(alignment) >= math.cos(math.radians(10)):
            end = b if alignment > 0 else a
            begin = a if alignment > 0 else b
            unit = (ux, uy) if alignment > 0 else (-ux, -uy)
            distance = length-along if alignment > 0 else along
            candidates.append((lateral, begin, end, unit, distance, (ax+along*ux, ay+along*uy)))
    if len(candidates) != 1:
        raise ValueError('start_not_on_unique_straight_road')
    _, previous, current, forward, remaining, projected = candidates[0]
    # Leave enough room for the existing Ackermann entry curve and a distinct
    # heading witness. A new command immediately inside a turn is rejected.
    entry = catalog['turn_entry_cm']
    exit_cm = catalog['turn_exit_cm']
    if remaining < entry+8+16+12:
        raise ValueError('start_too_close_to_junction_or_curve')
    constraints = []
    visited = []
    constraints.append((projected[0]+forward[0]*16, projected[1]+forward[1]*16))
    total_distance = remaining
    traversed = set()
    while True:
        edge = (previous, current)
        if edge in traversed:
            raise ValueError('junction_route_cycle')
        traversed.add(edge)
        if current in junctions:
            visited.append(current)
        cx, cy = points[current]
        choices = []
        for neighbor in adjacency[current]:
            if neighbor == previous:
                continue
            nx, ny = points[neighbor]
            length = math.hypot(nx-cx, ny-cy)
            unit = ((nx-cx)/length, (ny-cy)/length)
            dot = forward[0]*unit[0]+forward[1]*unit[1]
            cross = forward[0]*unit[1]-forward[1]*unit[0]
            choices.append((neighbor, unit, length, dot, cross))
        if len(visited) == nth:
            # In image coordinates a left turn has a negative cross product.
            turns = [c for c in choices if abs(c[3]) < .1 and
                     (c[4] < -.9 if direction == 'left' else c[4] > .9)]
            if len(turns) != 1:
                raise ValueError('requested_turn_not_present_at_junction')
            target, outgoing, length, _, _ = turns[0]
            after = max(30., exit_cm+8.)
            if length < after+8:
                raise ValueError('insufficient_straight_exit_for_ackermann_turn')
            before = entry+8.
            constraints.append((cx-forward[0]*before, cy-forward[1]*before))
            goal = (cx+outgoing[0]*after, cy+outgoing[1]*after)
            break
        straight = [c for c in choices if c[3] > .99]
        if len(straight) != 1:
            raise ValueError('straight_road_ends_before_requested_junction')
        neighbor, unit, length, _, _ = straight[0]
        # A midpoint after each passed junction disallows taking a shortcut
        # through another block before returning to the desired final branch.
        if current in junctions:
            offset = min(length/2, max(22., exit_cm+4.))
            constraints.append((cx+unit[0]*offset, cy+unit[1]*offset))
        previous, current, forward = current, neighbor, unit
        total_distance += length
        if len(constraints) > 8:
            raise ValueError('too_many_waypoint_constraints')
    def grid(point):
        x, y = point
        if not 0 <= x < width or not 0 <= y < height:
            raise ValueError('route_constraint_outside_map')
        return [int(x//2), int(y//2)]
    waypoints = []
    start = grid(projected)
    for point in constraints:
        quantized = grid(point)
        if quantized != start and (not waypoints or quantized != waypoints[-1]):
            waypoints.append(quantized)
    if len(waypoints) > 8:
        raise ValueError('too_many_waypoint_constraints')
    return dict(start=start, waypoints=waypoints, goal=grid(goal),
                junctions=visited, target_junction=current, exit_toward=target,
                requested_junction=nth, direction=direction,
                heading_deg=float(reference['heading_deg']),
                map_sha256=catalog['map_sha256'], reference='nose_midpoint',
                planner='fpga_directed_centerline',
                explanation='先直行经过指定数量的地图路口，在目标路口转弯，出弯后停车；路径由FPGA规划。')
