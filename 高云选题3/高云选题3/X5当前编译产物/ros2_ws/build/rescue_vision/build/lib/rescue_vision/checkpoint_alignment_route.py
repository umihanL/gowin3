"""Propose short forward road legs for a fixed side camera's image alignment.

Only geometry from the FPGA road ROM is used here. The FPGA's returned path
must also pass AreaNavigation's corridor check before START is sent. A centred
image box is not a metric depth, a nozzle alignment, or proof of extinguishing.
"""
import heapq
import math

from .region_route import inside


def _quantized_stop_has_continuation(row, heading, tolerance):
    """Prove this reachable node is a legal FPGA fallback from its 2 cm cell.

    Calling _seeds for every point and three headings needlessly rescans the
    complete ROM. The existing reachable node itself supplies the witness:
    <=6 cm access, <=35 degree direction error and the same forward half-plane.
    """
    x, y = ((value >> 9) * 512 + 256 for value in row[:2])
    if abs(row[0] - x) + abs(row[1] - y) > 6 * 256:
        return False
    for delta in (-tolerance, 0., tolerance):
        angle = (heading + delta + 180.) % 360. - 180.
        if abs((row[2] - angle + 180.) % 360. - 180.) > 35.:
            return False
        forward = (row[0] >= x - 2 * 256 if -45 <= angle <= 45 else
                   row[1] <= y + 2 * 256 if 45 < angle < 135 else
                   row[0] <= x + 2 * 256 if angle >= 135 or angle <= -135 else
                   row[1] >= y - 2 * 256)
        if not forward:
            return False
    return True


def forward_road_proposal(planner, region_id, reference, now, max_distance_cm=30.):
    base = dict(stamp=now, valid=False, road_path_verified=False,
                reason='checkpoint_alignment_requires_stopped_road_pose')
    if (planner is None or not isinstance(reference, dict) or reference.get('valid') is not True or
            not isinstance(reference.get('cell'), (list, tuple)) or len(reference['cell']) != 2 or
            any(type(v) is not int for v in reference['cell']) or
            type(reference.get('heading_deg')) not in (int, float) or
            not math.isfinite(reference['heading_deg']) or
            type(max_distance_cm) not in (int, float) or not math.isfinite(max_distance_cm)):
        return base
    max_distance_cm = min(30., float(max_distance_cm))
    if max_distance_cm < 12.:
        return dict(base, reason='checkpoint_alignment_forward_budget_exhausted')
    cell, heading = reference['cell'], reference['heading_deg']
    if region_id not in planner.catalog['regions'] or not -180 <= heading <= 180:
        return base
    bounds = planner.catalog['regions'][region_id]['bounds_image_cm']
    if not inside(cell, bounds):
        return dict(base, reason='checkpoint_alignment_start_outside_region')
    ox, oy = (2 * value + 1 for value in cell)
    rad = math.radians(heading)
    ux, uy = math.cos(rad), -math.sin(rad)  # Road coordinates have image Y down.
    lateral_limit, heading_limit = 6., 15.
    blocked = [planner._blocked(row) for row in planner.nodes]
    if all(blocked):
        return dict(base, reason='checkpoint_alignment_forward_road_blocked')
    reachable = planner.exit_reachable_nodes(region_id)
    forward = []
    allowed = []
    for index, row in enumerate(planner.nodes):
        x, y = row[0] / 256., row[1] / 256.
        dx, dy = x - ox, y - oy
        longitudinal, lateral = dx * ux + dy * uy, abs(dx * uy - dy * ux)
        forward.append(longitudinal)
        error = abs((row[2] - heading + 180.) % 360. - 180.)
        geometrically_allowed = (not blocked[index] and row[2] != 255 and index in reachable and
                       bounds[0] <= x < bounds[2] and bounds[1] <= y < bounds[3] and
                       -2. <= longitudinal <= max_distance_cm and lateral <= lateral_limit and
                       error <= heading_limit)
        # A centreline trigger may stop at any intermediate node. Keep every
        # usable part of the forward leg connected to a legal region exit,
        # including the permitted heading error after the physical stop.
        can_depart = geometrically_allowed and _quantized_stop_has_continuation(row, heading, heading_limit)
        allowed.append(can_depart)
    try:
        choices = planner._seeds(cell, heading, blocked)
    except ValueError:
        return dict(base, reason='checkpoint_alignment_no_forward_road_entry')
    # Match the FPGA: try nearest aliases first, then its local forward fallback.
    for seeds in choices:
        distance, queue = {}, []
        for index, _ in seeds:
            row = planner.nodes[index]
            access = math.hypot(row[0] / 256. - ox, row[1] / 256. - oy)
            if allowed[index] and access <= 6.:
                distance[index] = access
                heapq.heappush(queue, (access, index))
        while queue:
            cost, index = heapq.heappop(queue)
            if cost != distance.get(index):
                continue
            for neighbor, length in planner.nodes[index][3]:
                candidate = cost + length
                if (allowed[neighbor] and forward[neighbor] >= forward[index] - .1 and
                        candidate <= max_distance_cm and candidate < distance.get(neighbor, math.inf)):
                    distance[neighbor] = candidate
                    heapq.heappush(queue, (candidate, neighbor))
        goals = [index for index in distance if forward[index] >= 12. and distance[index] > 8.]
        # SET_GOAL transmits a 2 cm cell, not a ROM node ID. At intersections
        # its centre can be nearer a different directed branch than the node
        # we selected. Match the FPGA's nearest-coordinate goal resolution;
        # otherwise a nominal 30 cm advance may require a whole-road loop.
        realizable = []
        resolved_cells = {}
        for index in goals:
            row = planner.nodes[index]
            cell_key = (row[0] >> 9, row[1] >> 9)
            if cell_key not in resolved_cells:
                gx, gy = (value * 512 + 256 for value in cell_key)
                nearest = min(range(len(planner.nodes)), key=lambda i:
                    abs(planner.nodes[i][0] - gx) + abs(planner.nodes[i][1] - gy))
                xy = planner.nodes[nearest][:2]
                resolved_cells[cell_key] = xy
            if row[:2] == resolved_cells[cell_key]:
                realizable.append(index)
        goals = realizable
        if goals:
            selected = max(goals, key=lambda index: (forward[index], -distance[index], -index))
            row = planner.nodes[selected]
            goal = [row[0] >> 9, row[1] >> 9]
            return dict(base, valid=True, road_path_verified=True,
                        reason='checkpoint_alignment_forward_road_proposed', goal_cell=goal,
                        from_pose_cell=list(cell), from_heading_deg=heading,
                        path_length_cm=round(distance[selected], 3),
                        alignment_guard=dict(start_cell=list(cell), heading_deg=heading,
                            max_advance_cm=max_distance_cm, lateral_tolerance_cm=lateral_limit,
                            heading_tolerance_deg=heading_limit, speed_mm_s=60))
    return dict(base, reason='checkpoint_alignment_no_straight_road_in_region')
