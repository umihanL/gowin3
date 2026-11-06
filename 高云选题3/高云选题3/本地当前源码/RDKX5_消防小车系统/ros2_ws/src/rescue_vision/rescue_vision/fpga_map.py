"""Truthful HMI map view: static FPGA geometry plus verified UART telemetry.

This module does not localize, plan, interpolate a car, or perform any I/O.
"""
import json
import math


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _recent(stamp, now, ttl):
    return _finite(stamp) and _finite(now) and 0 <= now-stamp <= ttl


def _point(value):
    return value if type(value) is int and 1 <= value <= 9 else None


def load_catalog(path):
    with open(path, encoding='utf-8-sig') as source:
        catalog = json.load(source)
    geometry = catalog.get('geometry', {})
    if catalog.get('schema_version') != 1 or geometry.get('width_cm') != 352 or geometry.get('height_cm') != 320:
        raise ValueError('unsupported FPGA map geometry')
    if {p.get('id') for p in catalog.get('points', [])} != set(range(1, 10)):
        raise ValueError('FPGA map must define exactly nine destinations')
    for run in geometry.get('road_runs', []):
        if (len(run) != 3 or any(type(v) is not int for v in run) or
                not 0 <= run[0] < 320 or not 0 <= run[1] < 352 or
                not 0 < run[2] <= 352-run[1]):
            raise ValueError('invalid ROM raster run')
    return catalog


def _raster_points(values, length):
    if not isinstance(values, list) or not values:
        return None
    result = []
    for item in values:
        if not isinstance(item, (list, tuple)) or len(item) != length:
            return None
        if any(not _finite(v) for v in item):
            return None
        if any(not 0 <= item[i] <= (3520 if i % 2 == 0 else 3200) for i in range(length)):
            return None
        result.append([v/10 for v in item])
    return result


def live_map_snapshot(catalog, extension, navigation, now, telemetry=None):
    """Normalize the root bridge's telemetry without fabricating missing data.

    Incoming pose is world mm (Y up); route/barriers already use raster mm.
    A source header refreshed with unchanged generation may refresh its stamp.
    """
    extension = extension if isinstance(extension, dict) else {}
    navigation = navigation if isinstance(navigation, dict) else {}
    telemetry = telemetry if isinstance(telemetry, dict) else {}
    transport = (extension.get('real_uart') is True and extension.get('protocol_ready') is True
                 and extension.get('status_fresh') is True)
    supported = telemetry.get('supported') is True
    fresh = transport and supported and _recent(telemetry.get('stamp'), now, 3.)
    signature = telemetry.get('map_signature')
    identity = fresh and type(signature) is int and signature == catalog.get('map_signature')
    reason = ('ready' if identity else 'map_signature_mismatch' if fresh and signature is not None
              else 'map_signature_unverified' if fresh else 'telemetry_stale' if supported
              else 'firmware_telemetry_not_available')
    live = dict(supported=supported, fresh=bool(identity), reason=reason,
                pose=None, route=None, barriers=None,
                physical_position_verified=False,
                pose_source='FPGA wheel encoders and IMU dead reckoning')
    pose = telemetry.get('pose') or {}
    if identity and pose.get('valid') is True and _recent(pose.get('stamp'), now, 2.):
        x, y, heading = pose.get('x_mm'), pose.get('y_mm'), pose.get('heading_tenths')
        if (_finite(x) and _finite(y) and _finite(heading) and 0 <= x <= 3520
                and 0 <= y <= 3200 and -1800 <= heading <= 1800
                and pose.get('mode') == 9 and pose.get('status') in (2, 3)):
            live['pose'] = dict(valid=True, fresh=True, stamp=pose['stamp'],
                x_cm=x/10, y_cm=(3200-y)/10, heading_deg=heading/10,
                source='fpga_encoder_imu_odometry', task_id=pose.get('task_id'),
                status=pose.get('status'), phase=pose.get('phase'))
    route = telemetry.get('route') or {}
    generation, count = route.get('generation'), route.get('total_count')
    if (identity and route.get('valid') is True and _recent(route.get('stamp'), now, 20.)
            and type(generation) is int and 1 <= generation <= 65534
            and type(count) is int and 2 <= count <= 2048):
        points = _raster_points(route.get('points_mm'), 2)
        if points and 2 <= len(points) <= count:
            live['route'] = dict(valid=True, fresh=True, stamp=route['stamp'],
                generation=generation, total_count=count, sampled=len(points) < count,
                points_cm=points, source='fpga_execution_path_ram')
    barriers = telemetry.get('barriers') or {}
    generation = barriers.get('generation')
    if (identity and barriers.get('valid') is True and _recent(barriers.get('stamp'), now, 20.)
            and type(generation) is int and 1 <= generation <= 65534):
        segments = [] if barriers.get('segments_mm') == [] else _raster_points(barriers.get('segments_mm'), 4)
        if segments is not None and len(segments) <= 4:
            live['barriers'] = dict(valid=True, fresh=True, stamp=barriers['stamp'],
                generation=generation, segments_cm=segments, source='fpga_touch_barrier_registers',
                planner_inflation_cm=11)
    reference = extension.get('point_reference') or {}
    reference = dict(reference) if isinstance(reference, dict) else {}
    if not _point(reference.get('point_id')):
        reference = dict(confirmed=False, reason='no_confirmed_numbered_reference')
    # Never turn a confirmed starting point or an arrival event into live pose.
    reference['is_live_pose'] = False
    point_status = navigation.get('selected_point') or {}
    selected = _point(point_status.get('selected_point')) if isinstance(point_status, dict) else None
    if not (transport and navigation.get('status_fresh') is True and _recent(navigation.get('stamp'), now, 2.)):
        selected = None
    return dict(schema_version=1, geometry=catalog['geometry'], points=catalog['points'],
        map_signature=catalog.get('map_signature'), source_hashes=catalog.get('source_hashes', {}),
        firmware_map_verified=bool(identity), live=live, reference=reference,
        selection=dict(requested_start=_point(navigation.get('start_point')),
                       requested_target=_point(navigation.get('target_point')), fpga_selected_point=selected),
        limits=catalog.get('limits', []))
