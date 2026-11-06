"""CPU OCR and conservative, spatially consistent building sign confirmation."""
from collections import deque
from pathlib import Path
import hashlib
import json
import math
import re
import unicodedata
import time


class OCRReader:
    """Pinned PP-OCRv4 mobile via ONNX Runtime CPU; BGR single-eye input."""

    def __init__(self, model_dir, threads=2, analysis_max_side=512, max_text_regions=6, use_angle_cls=False):
        from rapidocr_onnxruntime import RapidOCR
        folder = Path(model_dir)
        manifest = json.loads((folder / 'manifest.json').read_text(encoding='utf-8'))
        for record in manifest['models']:
            payload = (folder / record['file']).read_bytes()
            if hashlib.sha256(payload).hexdigest() != record['sha256']:
                raise ValueError('OCR model hash mismatch: ' + record['file'])
        self.engine = RapidOCR(
            det_model_path=str(folder / 'ch_PP-OCRv4_det_infer.onnx'),
            rec_model_path=str(folder / 'ch_PP-OCRv4_rec_infer.onnx'),
            cls_model_path=str(folder / 'ch_ppocr_mobile_v2.0_cls_infer.onnx'),
            intra_op_num_threads=int(threads), inter_op_num_threads=1,
            det_use_cuda=False, cls_use_cuda=False, rec_use_cuda=False,
            det_use_dml=False, cls_use_dml=False, rec_use_dml=False,
            det_limit_type='max', det_limit_side_len=960,
        )
        if type(analysis_max_side) is not int or not 320 <= analysis_max_side <= 960:
            raise ValueError('OCR analysis_max_side must be an integer in 320..960')
        if type(max_text_regions) is not int or not 1 <= max_text_regions <= 12:
            raise ValueError('OCR max_text_regions must be an integer in 1..12')
        self.analysis_max_side = analysis_max_side
        self.max_text_regions = max_text_regions
        self.use_angle_cls = bool(use_angle_cls)
        self.last_timings = {}

    def predict(self, bgr):
        import cv2
        if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('OCR expects a BGR HxWx3 image')
        started = time.perf_counter()
        height, width = bgr.shape[:2]
        scale = min(1., self.analysis_max_side/max(height, width))
        analysis_size = (max(1, round(width*scale)), max(1, round(height*scale)))
        analysis = cv2.resize(bgr, analysis_size, interpolation=cv2.INTER_AREA) if scale < 1 else bgr
        # RapidOCR 1.4.4's det_limit_type=max selects an internal 960/1500/2000
        # bound from the incoming size. Explicitly resize before invoking it;
        # det_limit_side_len=512 alone does NOT constrain its actual input.
        boxes, det_elapsed = self.engine.text_det(analysis)
        selected = bounded_text_boxes(boxes, analysis_size, (width, height), self.max_text_regions)
        rec_elapsed = cls_elapsed = 0.
        results = []
        if selected:
            # Detection is coarse; recognition still sees the ORIGINAL corrected
            # pixels. Published quadrilaterals remain in full-resolution YOLO space.
            crops = self.engine.get_crop_img_list(bgr, selected)
            if self.use_angle_cls:
                crops, _, cls_elapsed = self.engine.text_cls(crops)
            recognized, rec_elapsed = self.engine.text_rec(crops)
            results = [{'text': str(text), 'confidence': float(confidence),
                        'quad': [[float(x), float(y)] for x, y in quad]}
                       for quad, (text, confidence) in zip(selected, recognized)]
        self.last_timings = dict(total_ms=(time.perf_counter()-started)*1000,
            detection_ms=float(det_elapsed)*1000, recognition_ms=float(rec_elapsed)*1000,
            classification_ms=float(cls_elapsed)*1000, analysis_size=list(analysis_size),
            detected_regions=0 if boxes is None else len(boxes), recognized_regions=len(selected),
            max_text_regions=self.max_text_regions, original_recognition_pixels=True)
        return results


def bounded_text_boxes(boxes, analysis_size, source_size, limit):
    """Map detector quads to source pixels and bound expensive text recognition."""
    import numpy as np
    if boxes is None:
        return []
    aw, ah = analysis_size
    width, height = source_size
    accepted = []
    for box in boxes:
        quad = np.asarray(box, dtype=np.float32)
        if quad.shape != (4, 2) or not np.isfinite(quad).all():
            continue
        quad = quad.copy()
        quad[:, 0] *= width/aw
        quad[:, 1] *= height/ah
        quad[:, 0] = np.clip(quad[:, 0], 0, width-1)
        quad[:, 1] = np.clip(quad[:, 1], 0, height-1)
        sides = np.linalg.norm(np.roll(quad, -1, axis=0)-quad, axis=1)
        long, short = max(sides), min(sides)
        # Avoid tiny noise and extremely long screen-text lines whose recognition
        # tensor width otherwise grows without bound. Building signs fit this cap.
        if short < 10 or long/short > 12:
            continue
        area = .5*abs(float(np.dot(quad[:, 0], np.roll(quad[:, 1], -1))
                            -np.dot(quad[:, 1], np.roll(quad[:, 0], -1))))
        if area > 0:
            accepted.append((area, quad))
    accepted.sort(key=lambda item: item[0], reverse=True)
    return [quad for _, quad in accepted[:limit]]


def normalize_text(text):
    """Never fuzzy-correct digits or substitute similar Chinese characters."""
    return re.sub(r'[\s·•,，:：;；._\-—]', '', unicodedata.normalize('NFKC', str(text)))


def _box(quad):
    return [min(p[0] for p in quad), min(p[1] for p in quad),
            max(p[0] for p in quad), max(p[1] for p in quad)]


def _iou(a, b):
    w = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    h = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = w * h
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / union if union > 0 else 0.0


class BuildingMatcher:
    def __init__(self, config):
        if isinstance(config, (str, Path)):
            config = json.loads(Path(config).read_text(encoding='utf-8'))
        self.config = config
        self.buildings = {b['building_id']: b for b in config['buildings']}
        self.minimum_confidence = float(config.get('ocr_min_confidence', 0.85))

    def match(self, readings):
        candidates = []
        for item in readings:
            confidence = float(item.get('confidence', 0))
            if not math.isfinite(confidence) or confidence < self.minimum_confidence:
                continue
            quad = item.get('quad', [])
            if len(quad) != 4 or any(len(p) != 2 or not all(math.isfinite(float(v)) for v in p) for p in quad):
                continue
            text = normalize_text(item.get('text', ''))
            # Parse the WHOLE number token: 1 and 11 can never substring-match.
            number_match = re.fullmatch(r'(\d{1,3})(?:号?楼|号栋|栋)?', text)
            matches = []
            for bid, b in self.buildings.items():
                name = normalize_text(b['name'])
                numeric = int(b['number'])
                if text == name or (number_match and int(number_match[1]) == numeric):
                    matches.append(bid)
                elif re.fullmatch(r'0*' + str(numeric) + r'(?:号?楼|栋)?' + re.escape(name), text):
                    matches.append(bid)
            for bid in matches:
                candidates.append({'building_id': bid, 'name': self.buildings[bid]['name'],
                                   'number': self.buildings[bid]['number'], 'text': item['text'],
                                   'confidence': confidence, 'quad': quad, 'bbox': _box(quad)})
        ids = {c['building_id'] for c in candidates}
        if len(ids) != 1:
            return {'status': 'ambiguous' if ids else 'unmatched', 'candidate': None,
                    'candidates': candidates}
        # The strongest single recognized word gives a stable spatial anchor.
        best = max(candidates, key=lambda c: c['confidence'])
        return {'status': 'candidate', 'candidate': best, 'candidates': candidates}


class BuildingTracker:
    def __init__(self, config, hits=3, window=5, max_span_s=2.0):
        self.matcher = BuildingMatcher(config)
        self.hits, self.max_span_s = int(hits), float(max_span_s)
        self.history = deque(maxlen=int(window))
        self.last_stamp = None
        self.last_frame = None

    def update(self, readings, stamp, frame_id):
        stamp = float(stamp)
        base = {'confirmed': False, 'status': 'acquiring', 'building_id': None,
                'stamp': stamp, 'frame_id': frame_id}
        if not math.isfinite(stamp):
            return dict(base, status='invalid_timestamp')
        if self.last_stamp is not None and (stamp <= self.last_stamp or frame_id == self.last_frame):
            return dict(base, status='duplicate_or_out_of_order')
        self.last_stamp, self.last_frame = stamp, frame_id
        match = self.matcher.match(readings)
        candidate = match['candidate']
        self.history.append((stamp, candidate))
        if not candidate:
            return dict(base, status=match['status'])
        good = [c for t, c in self.history if stamp - t <= self.max_span_s and c
                and c['building_id'] == candidate['building_id']
                and _iou(c['bbox'], candidate['bbox']) >= 0.25]
        confirmed = len(good) >= self.hits
        return dict(base, **candidate, confirmed=confirmed,
                    status='confirmed' if confirmed else 'acquiring',
                    support_frames=len(good))
