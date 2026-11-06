"""Three-class YOLO11, fixed 640 letterbox, raw NHWC head decoding.

RGB/DFL/NMS numerical code is shared by desktop verification and board inference.
No camera, model runtime, or hardware action occurs at import time.
"""
from pathlib import Path
import time
import cv2
import numpy as np

CLASSES = ('fire', 'firefighter', 'person')
OUTPUT_NAMES = ('cls_s8', 'box_s8', 'cls_s16', 'box_s16', 'cls_s32', 'box_s32')
_DFL_BINS = np.arange(16, dtype=np.float32)


def read_image(path):
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('Cannot decode image: ' + str(path))
    return image


def letterbox(image, size=640):
    h, w = image.shape[:2]
    gain = min(size / h, size / w)
    nw, nh = round(w * gain), round(h * gain)
    dw, dh = (size - nw) / 2, (size - nh) / 2
    left, top = round(dw - .1), round(dh - .1)
    right, bottom = round(dw + .1), round(dh + .1)
    resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR) if (nw, nh) != (w, h) else image
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114,114,114))
    return padded, dict(gain=gain, left=left, top=top, width=w, height=h)


def rgb_tensor(padded, normalize=True):
    a = np.ascontiguousarray(padded[..., ::-1].transpose(2,0,1)[None], dtype=np.float32)
    return a / 255.0 if normalize else a


def nv12_tensor(padded):
    """Packed BT.601 FULL-range NV12 for mapper input_space_and_range=regular.

    OpenCV COLOR_BGR2YUV_I420 emits limited/video-range Y (16..235).
    That is not compatible with the compiler's YUV_BT601_FULL_RANGE_128
    preprocessing node. Chroma sampling follows the toolchain's rgb_calc.
    """
    h, w = padded.shape[:2]
    if h % 2 or w % 2:
        raise ValueError('NV12 requires even dimensions')
    b,g,r = (padded[...,i].astype(np.float32) for i in range(3))
    y = .299*r + .587*g + .114*b
    u = (-.169*r-.331*g+.5*b+128)[::2,::2]
    v = (.5*r-.419*g-.081*b+128)[::2,::2]
    uv = np.stack((u,v),axis=-1).reshape(-1)
    return np.clip(np.concatenate((y.reshape(-1),uv)),0,255).astype(np.uint8)


def _validated_heads(outputs):
    if len(outputs) != 6:
        raise ValueError('Expected six NHWC outputs, [class, DFL box] at strides 8/16/32')
    heads = []
    for i, stride in enumerate((8,16,32)):
        side = 640 // stride
        cls, box = np.asarray(outputs[2*i]), np.asarray(outputs[2*i+1])
        if cls.shape != (1, side, side, 3) or box.shape != (1, side, side, 64):
            raise ValueError(f'Unexpected output shape: {cls.shape}, {box.shape}')
        if not np.issubdtype(cls.dtype, np.floating) or not np.issubdtype(box.dtype, np.floating):
            raise ValueError('Decoder requires dequantized floating-point outputs')
        heads.append((cls, box, stride, side))
    return heads


def decode_dense(outputs):
    """Return [8400,7] for export/verification; live detection decodes candidates only."""
    dense = []
    for cls, box, stride, side in _validated_heads(outputs):
        logits = box.reshape(-1, 4, 16).astype(np.float32)
        exp = np.exp(logits - logits.max(axis=-1, keepdims=True))
        distance = (exp / exp.sum(axis=-1, keepdims=True) * _DFL_BINS).sum(axis=-1)
        yy, xx = np.meshgrid(np.arange(side), np.arange(side), indexing='ij')
        anchors = np.stack((xx.ravel()+.5, yy.ravel()+.5), axis=1).astype(np.float32)
        xyxy = np.concatenate((anchors-distance[:,:2], anchors+distance[:,2:]),axis=1)*stride
        scores = 1.0 / (1.0 + np.exp(-np.clip(cls.reshape(-1,3), -80,80)))
        dense.append(np.concatenate((xyxy,scores),axis=1))
    return np.concatenate(dense,axis=0)


def _class_filter(class_ids):
    if class_ids is None:
        return None
    selected = tuple(class_ids)
    if any(isinstance(value, (bool, np.bool_)) or
           not isinstance(value, (int, np.integer)) or not 0 <= value < len(CLASSES)
           for value in selected):
        raise ValueError('class_ids must contain integer IDs 0, 1 or 2')
    return tuple(sorted(set(map(int, selected))))


def _decode_candidates(outputs, conf, class_ids):
    """Keep legacy three-class argmax semantics, then decode only retained DFLs.

    Applying a class filter never promotes a runner-up fire score over a winning
    person/firefighter class. max_det is applied to the requested classes later.
    """
    heads = _validated_heads(outputs)
    # The old dense concatenation promoted all heads to a common dtype. Preserve
    # that behavior even if a higher-precision head has no surviving candidates.
    dtype = np.result_type(np.float32, *(head[0].dtype for head in heads))
    all_boxes, all_scores, all_ids = [], [], []
    for cls, box, stride, side in heads:
        class_logits = cls.reshape(-1, 3)
        preselected = None
        if conf >= .25:
            # sigmoid(-2) < .12 in every supported floating-point dtype, well
            # below .25. This deliberately loose bound avoids a logit(conf)
            # rounding boundary and never changes the original winning class.
            # Evaluate all three sigmoids for survivors: saturated ties must
            # still select the first class, even when its raw logit is lower.
            if class_ids is not None and len(class_ids) == 1:
                possible = class_logits[:, class_ids[0]] >= -2
            else:
                # Most camera anchors are background. Three strided comparisons
                # avoid evaluating exp for all 25,200 logits while preserving
                # every possible winner, including multi-class filtered calls.
                possible = ((class_logits[:, 0] >= -2) |
                            (class_logits[:, 1] >= -2) |
                            (class_logits[:, 2] >= -2))
            preselected = np.flatnonzero(possible)
            if not len(preselected):
                continue
            class_logits = class_logits[preselected]
        probabilities = 1.0 / (1.0 + np.exp(-np.clip(class_logits, -80, 80)))
        ids = probabilities.argmax(axis=1)
        scores = probabilities[np.arange(len(ids)), ids]
        eligible = scores >= conf
        if class_ids is not None:
            eligible &= ids == class_ids[0] if len(class_ids) == 1 else np.isin(ids, class_ids)
        chosen = np.flatnonzero(eligible)
        if not len(chosen):
            continue
        scores, ids = scores[chosen], ids[chosen]
        if preselected is not None:
            chosen = preselected[chosen]
        logits = box.reshape(-1, 4, 16)[chosen].astype(np.float32)
        exp = np.exp(logits - logits.max(axis=-1, keepdims=True))
        distance = (exp / exp.sum(axis=-1, keepdims=True) * _DFL_BINS).sum(axis=-1)
        anchors = np.stack((chosen % side + .5, chosen // side + .5), axis=1).astype(np.float32)
        xyxy = np.concatenate((anchors - distance[:, :2], anchors + distance[:, 2:]), axis=1) * stride
        all_boxes.append(xyxy.astype(dtype, copy=False))
        all_scores.append(scores.astype(dtype, copy=False))
        all_ids.append(ids)
    if not all_boxes:
        return np.empty((0, 4), dtype=dtype), np.empty(0, dtype=dtype), np.empty(0, dtype=np.int64)
    return np.concatenate(all_boxes), np.concatenate(all_scores), np.concatenate(all_ids)


def nms(boxes, scores, iou_threshold):
    order = np.argsort(-scores, kind='stable')
    keep = []
    while len(order):
        i = order[0]; keep.append(int(i)); order = order[1:]
        if not len(order):
            break
        a, b = boxes[i], boxes[order]
        wh = np.maximum(np.minimum(a[2:], b[:,2:]) - np.maximum(a[:2], b[:,:2]), 0)
        inter = wh[:,0]*wh[:,1]
        area_a = np.prod(np.maximum(a[2:]-a[:2],0))
        area_b = np.prod(np.maximum(b[:,2:]-b[:,:2],0),axis=1)
        iou = inter / np.maximum(area_a+area_b-inter, 1e-9)
        order = order[iou <= iou_threshold]
    return keep


def detections(outputs, meta, conf=.25, iou=.7, max_det=300, class_ids=None, timings=None):
    """Decode/NMS selected classes; default remains the existing three-class API.

    `class_ids=(0,)` returns fire only. It filters the original winning class,
    does not alter model logits, and applies max_det after the class filter.
    An optional mutable timings dict receives CPU stage times in seconds.
    """
    started = time.perf_counter()
    if not 0 < conf < 1 or not 0 < iou <= 1:
        raise ValueError('Confidence must be in (0,1), IoU in (0,1]')
    boxes, scores, ids = _decode_candidates(outputs, conf, _class_filter(class_ids))
    decoded_at = time.perf_counter()
    keep = []
    for cls in range(3):
        indices = np.flatnonzero(ids == cls)
        keep.extend(indices[nms(boxes[indices], scores[indices], iou)].tolist())
    keep = sorted(keep, key=lambda j: -scores[j])[:max_det]
    result = []
    for j in keep:
        b = boxes[j].copy()
        b[[0,2]] = np.clip((b[[0,2]]-meta['left'])/meta['gain'], 0, meta['width'])
        b[[1,3]] = np.clip((b[[1,3]]-meta['top'])/meta['gain'], 0, meta['height'])
        if b[2] <= b[0] or b[3] <= b[1]:
            continue
        result.append(dict(class_id=int(ids[j]), class_name=CLASSES[ids[j]], confidence=float(scores[j]), bbox_xyxy=b.tolist()))
    if timings is not None:
        finished = time.perf_counter()
        timings.update(candidate_decode_s=decoded_at-started, nms_format_s=finished-decoded_at,
                       postprocess_s=finished-started, decoded_candidates=len(boxes))
    return result


class BoardDetector:
    """RDK OS runtime adapter. X5 hardware is required for these backends."""
    def __init__(self, model, backend='auto', conf=.25, iou=.7, class_ids=None,
                 fused_preprocess=True, preprocess_threads=2):
        self.conf, self.iou = conf, iou
        self.fused_preprocess = bool(fused_preprocess)
        self.preprocess_threads = int(preprocess_threads)
        if not 1 <= self.preprocess_threads <= 8:
            raise ValueError('preprocess_threads must be in 1..8')
        self.class_ids = _class_filter(class_ids)
        self.last_timings = {}
        from .native_preprocess import choose_preprocessor
        self.preprocessor, self.preprocess_backend = choose_preprocessor()
        self.runtime_fallback_reason = None
        if backend == 'native_or_pyeasy':
            from .native_runtime import try_native_runtime
            runtime, self.runtime_fallback_reason = try_native_runtime(model)
            if runtime is not None:
                self.backend, self.model, self.names = 'native_hbdnn', runtime, list(runtime.names)
                return
            backend = 'pyeasy'
        elif backend == 'native':
            from .native_runtime import NativeRuntime
            self.backend, self.model = 'native_hbdnn', NativeRuntime(model)
            self.names = list(self.model.names)
            return
        if backend == 'auto':
            try:
                import hbm_runtime
                backend = 'hbm'
            except ImportError:
                backend = 'pyeasy'
        self.backend = backend
        if backend == 'hbm':
            import hbm_runtime
            self.model = hbm_runtime.HB_HBMRuntime(str(model))
            self.name = self.model.model_names[0]
            self.input_name = self.model.input_names[self.name][0]
            self.names = list(self.model.output_names[self.name])
        elif backend == 'pyeasy':
            from hobot_dnn import pyeasy_dnn
            self.model = pyeasy_dnn.load(str(model))[0]
            self.names = [o.name for o in self.model.outputs]
        else:
            raise ValueError('backend must be auto, hbm, pyeasy, native, or native_or_pyeasy')
        if set(self.names) != set(OUTPUT_NAMES):
            raise ValueError('Model output names differ from deployment contract: '+repr(self.names))

    def forward(self, packed_nv12):
        if self.backend == 'native_hbdnn':
            return self.model.forward(packed_nv12)
        if self.backend == 'hbm':
            raw = self.model.run({self.name:{self.input_name:packed_nv12}})[self.name]
        else:
            tensors = self.model.forward(packed_nv12)
            raw = {}
            for name, tensor in zip(self.names, tensors):
                a = np.asarray(tensor.buffer)
                if not np.issubdtype(a.dtype, np.floating):
                    raise RuntimeError('Integer output encountered. Use the supplied float-output BIN; do not silently ignore runtime quantization metadata.')
                raw[name] = a
        outputs = [np.asarray(raw[n]) for n in OUTPUT_NAMES]
        # Shape and dtype validation is performed by detections/decode_dense.
        # Do not decode twice in the board's camera loop.
        return outputs

    def close(self):
        if self.backend == 'native_hbdnn':
            self.model.close()

    def prepare(self, image):
        """Pure per-frame CPU preparation; no model state or shared output buffer."""
        started = time.perf_counter()
        converter = getattr(self, 'preprocessor', None)
        if (getattr(self, 'fused_preprocess', False) and image.shape == (1088, 1280, 3)
                and getattr(converter, 'hd_parallel_available', False)):
            packed = converter.prepare_hd(image, threads=self.preprocess_threads)
            elapsed = time.perf_counter()-started
            return packed, dict(gain=.5, left=0, top=48, width=1280, height=1088), dict(
                letterbox_s=0.0, nv12_s=0.0, fused_prepare_s=elapsed, preprocess_s=elapsed,
                preprocess_path='native_fused_hd')
        padded, meta = letterbox(image)
        letterboxed_at = time.perf_counter()
        packed = getattr(self, 'preprocessor', nv12_tensor)(padded)
        preprocessed_at = time.perf_counter()
        return packed, meta, dict(letterbox_s=letterboxed_at-started,
            nv12_s=preprocessed_at-letterboxed_at, preprocess_s=preprocessed_at-started,
            preprocess_path=getattr(self, 'preprocess_backend', 'legacy_numpy'))

    def predict_prepared(self, prepared):
        """Single inference worker owns forward and last_timings."""
        self.last_timings = {}
        packed, meta, preparation_timing = prepared
        started = time.perf_counter()
        raw = self.forward(packed)
        inferred_at = time.perf_counter()
        timing = {}
        items = detections(raw, meta, self.conf, self.iou, class_ids=self.class_ids, timings=timing)
        finished = time.perf_counter()
        timing.update(preparation_timing)
        timing.update(inference_s=inferred_at-started,
                      total_s=finished-started+preparation_timing['preprocess_s'])
        self.last_timings = timing
        return items

    def predict(self, image):
        return self.predict_prepared(self.prepare(image))
