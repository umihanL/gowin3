"""Evidence-based current-view scene reports and symbolic navigation requests.

No language model, global localization or route execution is implied.
"""
from pathlib import Path
import copy
import json
import math


def _contained(box, roi):
    return (len(box) == 4 and all(math.isfinite(float(v)) for v in box)
            and box[0] < box[2] and box[1] < box[3]
            and roi[0] <= box[0] and roi[1] <= box[1]
            and box[2] <= roi[2] and box[3] <= roi[3])


class SceneMemory:
    """Observation history stays historical; expired evidence is never current."""

    def __init__(self, config, ttl_s=2.0, min_detection_confidence=0.4):
        if isinstance(config, (str, Path)):
            config = json.loads(Path(config).read_text(encoding='utf-8'))
        self.config = config
        self.buildings = {b['building_id']: b for b in config['buildings']}
        self.ttl_s = float(ttl_s)
        self.min_confidence = float(min_detection_confidence)
        self.latest = None
        self.history = []

    def observe(self, building_evidence, detections, stamp, frame_id,
                facade_roi=None, roi_verified=False):
        """detections are same-frame dicts with class_name/confidence/bbox.

        roi_verified means a separately established single-facade ROI (operator
        commissioning or geometric facade association). Merely seeing one sign
        somewhere in a full image does NOT associate every object with its building.
        """
        stamp = float(stamp)
        report = {'stamp': stamp, 'frame_id': frame_id, 'building_id': None,
                  'building_number': None, 'building_name': None,
                  'fire_status': 'unknown', 'visible_person_count': None,
                  'visible_firefighter_count': None, 'casualty_count': None,
                  'observation_scope': 'current_view', 'valid': False,
                  'reason': 'unconfirmed_building_or_facade', 'evidence': [],
                  'recommended_action': 'observe_and_confirm_building',
                  'physical_fire_verified': False}
        if not math.isfinite(stamp) or (self.latest and stamp <= self.latest['stamp']):
            report['reason'] = 'invalid_or_out_of_order_timestamp'
            return report
        b = building_evidence or {}
        if (b.get('confirmed') and b.get('building_id') in self.buildings
                and b.get('frame_id') == frame_id and b.get('stamp') == stamp):
            record = self.buildings[b['building_id']]
            report.update(building_id=b['building_id'], building_number=record['number'],
                          building_name=record['name'])
            report['evidence'].append({'type': 'building_ocr', 'text': b.get('text'),
                                       'confidence': b.get('confidence'), 'bbox': b.get('bbox'),
                                       'stamp': stamp, 'frame_id': frame_id})
            if (roi_verified and facade_roi is not None and len(facade_roi) == 4
                    and _contained(b.get('bbox', []), facade_roi)):
                accepted = []
                for detection in detections:
                    name = detection.get('class_name', detection.get('class', ''))
                    conf = float(detection.get('confidence', 0))
                    box = detection.get('bbox', detection.get('box', []))
                    if (name in ('fire', 'person', 'firefighter') and math.isfinite(conf)
                            and conf >= self.min_confidence and _contained(box, facade_roi)):
                        accepted.append({'type': 'object_detection', 'class_name': name,
                                         'confidence': conf, 'bbox': list(box), 'stamp': stamp,
                                         'frame_id': frame_id})
                # Detector performs NMS upstream. Counts refer to this frame only.
                names = [d['class_name'] for d in accepted]
                report.update(valid=True, reason='same_frame_verified_facade',
                              fire_status='observed' if 'fire' in names else 'not_observed',
                              visible_person_count=names.count('person'),
                              visible_firefighter_count=names.count('firefighter'),
                              facade_roi=list(facade_roi),
                              recommended_action='stop_and_observe_fire' if 'fire' in names
                              else 'continue_inspection')
                report['evidence'].extend(accepted)
        self.latest = copy.deepcopy(report)
        self.history.append(copy.deepcopy(report))
        self.history = self.history[-100:]
        return report

    def current(self, now):
        if self.latest is None:
            return {'valid': False, 'fire_status': 'unknown', 'reason': 'no_observation'}
        result = copy.deepcopy(self.latest)
        if not math.isfinite(float(now)) or not 0 <= float(now) - result['stamp'] <= self.ttl_s:
            result.update(valid=False, fire_status='unknown', visible_person_count=None,
                          visible_firefighter_count=None, casualty_count=None,
                          reason='expired_observation', recommended_action='observe_again')
        return result

    def route_request(self, building_id, now):
        if building_id not in self.buildings:
            raise ValueError('Unknown building id')
        record = self.buildings[building_id]
        return {'stamp': float(now), 'building_id': building_id,
                'building_name': record['name'], 'intent': 'inspect_building',
                'observation_point': copy.deepcopy(record.get('observation_point')),
                'status': 'symbolic_goal_only', 'can_execute': False,
                'requires': ['map_to_real_registration', 'localization', 'route_planner',
                             'obstacle_aware_ackermann_controller']}

    def resolve_request(self, text, now):
        """Explicit building mentions only, not unrestricted language understanding."""
        from .building_ocr import normalize_text
        import re
        text = normalize_text(text)
        ids = set()
        for bid, b in self.buildings.items():
            if normalize_text(b['name']) in text:
                ids.add(bid)
        for number in re.findall(r'(?<!\d)(\d{1,3})(?:号楼|栋|号栋)(?!\d)', text):
            ids.update(bid for bid, b in self.buildings.items() if int(b['number']) == int(number))
        if len(ids) != 1:
            return {'status': 'ambiguous_or_unknown_goal', 'can_execute': False,
                    'candidate_building_ids': sorted(ids)}
        return self.route_request(next(iter(ids)), now)
