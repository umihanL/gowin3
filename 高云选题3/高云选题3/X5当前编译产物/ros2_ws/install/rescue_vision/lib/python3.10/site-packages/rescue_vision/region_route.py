"""Order up to four search waypoints on the FPGA's actual directed road ROM.

This is a scheduling estimate, never permission to drive. FPGA A* chooses each
actual trajectory from the starting heading. Continuous patrol plans once;
these waypoints shape the route and do not request intermediate stops.
"""
import hashlib
import heapq
import json
import math


OUTSIDE_EXTRA_MULTIPLIER = 32


def inside(cell, bounds):
    x, y = (2 * int(value) + 1 for value in cell)
    return bounds[0] <= x < bounds[2] and bounds[1] <= y < bounds[3]


class RegionRoutePlanner:
    def __init__(self, document, catalog, map_signature):
        self.catalog = catalog
        self.nodes = document.get('directed_nodes', [])
        if (document.get('map_sha256') != catalog.get('map_sha256') or
                document.get('map_signature') != map_signature or not self.nodes or
                hashlib.sha256(json.dumps(self.nodes, separators=(',', ':')).encode('ascii')).hexdigest()
                != document.get('directed_nodes_sha256')):
            raise ValueError('region_directed_road_graph_identity_invalid')
        for row in self.nodes:
            if (not isinstance(row, list) or len(row) != 4 or
                    any(type(value) is not int for value in row[:3]) or
                    not 0 <= row[0] < 352 * 256 or not 0 <= row[1] < 320 * 256 or
                    not (row[2] == 255 or -180 <= row[2] <= 180) or
                    not isinstance(row[3], list) or len(row[3]) > 3):
                raise ValueError('region_directed_road_graph_invalid')
            for edge in row[3]:
                if (not isinstance(edge, list) or len(edge) != 2 or
                        any(type(value) is not int for value in edge) or
                        not 0 <= edge[0] < len(self.nodes) or not 1 <= edge[1] <= 15):
                    raise ValueError('region_directed_road_edge_invalid')
        self.barriers = []
        self.reverse_edges = [[] for _ in self.nodes]
        for source, row in enumerate(self.nodes):
            for destination, _ in row[3]:
                self.reverse_edges[destination].append(source)
        self._exit_reachability_cache = {}

    def observe_map(self, value):
        # The live FPGA owns obstacle inflation and route validation. The same
        # rectangles are excluded here solely to avoid misleading scheduling.
        document = value.get('barriers') or {}
        self.barriers = []
        if document.get('valid') is True:
            for line in document.get('segments_mm', []):
                if isinstance(line, list) and len(line) == 4 and all(type(v) is int for v in line):
                    # Barrier telemetry uses image-mm, y downwards.
                    x1, y1, x2, y2 = line
                    self.barriers.append(dict(bounds_cm=[min(x1, x2) / 10., min(y1, y2) / 10.,
                                                         max(x1, x2) / 10., max(y1, y2) / 10.]))

    def _blocked(self, node):
        x, y = node[0] / 256., node[1] / 256.
        for barrier in self.barriers:
            box = barrier.get('bounds_cm')
            if (isinstance(box, (list, tuple)) and len(box) == 4 and
                    all(type(v) in (int, float) and math.isfinite(v) for v in box) and
                    box[0] - 11 <= x <= box[2] + 11 and box[1] - 11 <= y <= box[3] + 11):
                return True
        return False

    def _seeds(self, cell, heading, blocked):
        x, y = (int(v) * 512 + 256 for v in cell)
        nearest = min((abs(node[0] - x) + abs(node[1] - y), index)
                      for index, node in enumerate(self.nodes) if not blocked[index])[1]
        nearest_xy = self.nodes[nearest][:2]
        primary, fallback = [], []
        for index, node in enumerate(self.nodes):
            if blocked[index] or node[2] == 255:
                continue
            error = abs((node[2] - heading + 180.) % 360. - 180.)
            distance = (abs(node[0] - x) + abs(node[1] - y)) / 256.
            forward = (node[0] >= x - 2 * 256 if -45 <= heading <= 45 else
                       node[1] <= y + 2 * 256 if 45 < heading < 135 else
                       node[0] <= x + 2 * 256 if heading >= 135 or heading <= -135 else
                       node[1] >= y - 2 * 256)
            if error <= 35.:
                if node[:2] == nearest_xy:
                    primary.append((index, 0.))
                if distance <= 6. and forward:
                    fallback.append((index, math.ceil(distance)))
        if not primary and not fallback:
            raise ValueError('region_start_heading_has_no_directed_road_entry')
        return primary, fallback

    def _exit_nodes(self, bounds, blocked):
        """The same physical exit clearance used by the task scheduler."""
        exits = set()
        for index, row in enumerate(self.nodes):
            if blocked[index] or row[2] == 255:
                continue
            x, y = row[0] / 256., row[1] / 256.
            if bounds[0] <= x < bounds[2] and bounds[1] <= y < bounds[3]:
                continue
            dx = max(bounds[0] - x, 0., x - bounds[2])
            dy = max(bounds[1] - y, 0., y - bounds[3])
            if 14. <= dx + dy <= 24.:
                exits.add(index)
        return exits

    def exit_reachable_nodes(self, region_id):
        """Road nodes that can still leave this region in the legal direction.

        A straight-looking ROM edge may end at a tangent-255 sink after its
        turn-off. Never advance for image alignment beyond that turn-off: the
        next task would otherwise be stranded even with perfect odometry.
        This does not grant motion permission or bypass FPGA route validation.
        """
        blocked = tuple(self._blocked(row) for row in self.nodes)
        cache_key = (region_id, blocked)
        if cache_key in self._exit_reachability_cache:
            return self._exit_reachability_cache[cache_key]
        reachable = self._exit_nodes(self.catalog['regions'][region_id]['bounds_image_cm'], blocked)
        pending = list(reachable)
        while pending:
            destination = pending.pop()
            for source in self.reverse_edges[destination]:
                if not blocked[source] and source not in reachable:
                    reachable.add(source)
                    pending.append(source)
        # Only four regions are normally used. Avoid retaining stale obstacle
        # snapshots forever if a live map changes repeatedly.
        if len(self._exit_reachability_cache) >= 8:
            self._exit_reachability_cache.clear()
        result = frozenset(reachable)
        self._exit_reachability_cache[cache_key] = result
        return result

    def can_continue_from_pose(self, region_id, cell, heading):
        """Check quantized stop pose/real heading against FPGA start aliases."""
        reachable = self.exit_reachable_nodes(region_id)
        blocked = [self._blocked(row) for row in self.nodes]
        if not reachable or all(blocked):
            return False
        try:
            choices = self._seeds(cell, heading, blocked)
        except ValueError:
            return False
        return any(index in reachable for seeds in choices for index, _ in seeds)

    def plan(self, region_id, remaining, reference):
        if (reference.get('valid') is not True or
                not isinstance(reference.get('cell'), (tuple, list)) or len(reference['cell']) != 2 or
                type(reference.get('heading_deg')) not in (int, float) or
                not math.isfinite(reference['heading_deg'])):
            raise ValueError('region_replan_requires_actual_pose_and_heading')
        bounds = self.catalog['regions'][region_id]['bounds_image_cm']
        blocked = [self._blocked(node) for node in self.nodes]
        if all(blocked):
            raise ValueError('region_all_directed_roads_blocked')
        in_region = [bounds[0] * 256 <= row[0] < bounds[2] * 256 and
                     bounds[1] * 256 <= row[1] < bounds[3] * 256 for row in self.nodes]
        initial_inside = inside(reference['cell'], bounds)
        masks = [0] * len(self.nodes)
        postpone_mask = 0
        for bit, point in enumerate(remaining):
            if all(abs(reference['cell'][i] - point['cell'][i]) <= 4 for i in range(2)):
                postpone_mask |= 1 << bit
            px, py = (v * 512 + 256 for v in point['cell'])
            # Match the FPGA's snapped goal and terminal 6 cm L1 tolerance.
            nearest = min((abs(row[0] - px) + abs(row[1] - py), index)
                          for index, row in enumerate(self.nodes) if not blocked[index])[1]
            gx, gy = self.nodes[nearest][:2]
            for index, row in enumerate(self.nodes):
                if abs(row[0] - gx) + abs(row[1] - gy) <= 6 * 256:
                    masks[index] |= 1 << bit
        # >6 cm terminal tolerance plus raster rounding remains outside.
        exits = self._exit_nodes(bounds, blocked)
        if not exits:
            raise ValueError('region_road_exit_unavailable')
        full = (1 << len(remaining)) - 1
        if full and postpone_mask == full:
            raise ValueError('region_remaining_waypoint_requires_stationary_observation')
        for attempt, seeds in enumerate(self._seeds(reference['cell'], reference['heading_deg'], blocked)):
            distances, parent, queue = {}, {}, []
            for node, access in seeds:
                key = (masks[node] & ~postpone_mask, node)
                cost = (access, 0., access)
                if cost < distances.get(key, (math.inf,) * 3):
                    distances[key], parent[key] = cost, None
                    heapq.heappush(queue, (*cost, key))
            terminal = None
            while queue:
                weighted, outside_cm, distance, key = heapq.heappop(queue)
                if distances.get(key) != (weighted, outside_cm, distance):
                    continue
                mask, node = key
                if mask == full and node in exits:
                    terminal = key
                    break
                for neighbor, length in self.nodes[node][3]:
                    if blocked[neighbor]:
                        continue
                    # A target at our present stopped cell must be visited
                    # after another stop; sending a zero-length FPGA route is
                    # neither an observation interval nor valid motion.
                    collect = masks[neighbor] if mask & ~postpone_mask else masks[neighbor] & ~postpone_mask
                    new_mask = mask | collect
                    # Entry and final departure can naturally be outside. Between
                    # searches, use the identical 32x penalty as FPGA A*.
                    search_leg = mask != full and (mask != 0 or initial_inside)
                    extra = length if search_leg and not in_region[node] else 0
                    new_cost = (weighted + length + OUTSIDE_EXTRA_MULTIPLIER * extra,
                                outside_cm + extra, distance + length)
                    nxt = (new_mask, neighbor)
                    if new_cost < distances.get(nxt, (math.inf,) * 3):
                        distances[nxt], parent[nxt] = new_cost, key
                        heapq.heappush(queue, (*new_cost, nxt))
            if terminal is not None:
                break
        if terminal is None:
            raise ValueError('region_waypoints_or_exit_unreachable_from_actual_heading')
        trace, key = [], terminal
        while key is not None:
            trace.append(key)
            key = parent[key]
        trace.reverse()
        order, seen = [], 0
        for mask, node in trace:
            for bit, point in enumerate(remaining):
                if mask & (1 << bit) and not seen & (1 << bit):
                    order.append(point)
            seen |= mask
        end = self.nodes[terminal[1]]
        outside_cm, distance = distances[terminal][1:]
        return dict(waypoints=order,
            exit=dict(id=region_id + '_EXIT', cell=[end[0] >> 9, end[1] >> 9],
                      heading_deg=end[2], confirmed_stopped=False),
            route_policy=dict(strategy='directed_graph_region_preferred',
                outside_extra_multiplier=OUTSIDE_EXTRA_MULTIPLIER,
                region_preference=initial_inside,
                fallback_reason='road_turns_require_outside_detour' if outside_cm else None,
                start_entry_mode='nearby_forward_heading' if attempt else 'nearest_directed_alias',
                planned_from=dict(cell=list(reference['cell']), heading_deg=reference['heading_deg']),
                estimated_outside_cm=round(outside_cm, 1), estimated_distance_cm=round(distance, 1),
                actual_trajectory_owner='fpga', physical_position_verified=False))
