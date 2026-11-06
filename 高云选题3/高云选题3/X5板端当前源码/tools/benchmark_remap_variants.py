#!/usr/bin/env python3
"""Alternate two kernels on identical saved pixels/maps; no camera or model."""
import argparse
import json
from pathlib import Path
import statistics
import time
import cv2
import numpy as np
from rescue_vision.lens_correction import LensCorrector
from rescue_vision.native_remap import NativeRemap


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--repeats',type=int,default=24)
    args=p.parse_args();root=Path(__file__).resolve().parents[1]
    cv2.setNumThreads(1)
    image=cv2.imread(str(root/'logs/fire_lens_20261010/raw_physical_left.png'))
    if image is None:raise RuntimeError('Saved full resolution reference image missing')
    lens=LensCorrector.from_json(root/'config/gs130w_intrinsics_20261010.json',
        role_to_source={'left':'/image_right_raw'})
    maps,_,_=lens._get_maps('/image_right_raw',(1280,1088))
    rows=[]
    for threads in (1,2,4,8):
        operations={'scalar':NativeRemap(threads=threads),'candidate':NativeRemap(args.candidate,threads=threads)}
        expected=operations['scalar'](image,maps)
        np.testing.assert_array_equal(operations['candidate'](image,maps),expected)
        for fn in operations.values():fn(image,maps)
        measurements={name:[] for name in operations}
        for i in range(args.repeats):
            # Reverse order every other pair to reduce short-term bias.
            for name in (list(operations) if i%2 else list(reversed(operations))):
                start=time.perf_counter();operations[name](image,maps)
                measurements[name].append((time.perf_counter()-start)*1000)
        for name,values in measurements.items():
            rows.append(dict(kernel=name,threads=threads,median_ms=statistics.median(values),
                             p90_ms=float(np.percentile(values,90)),samples=values))
    report=dict(pixels_equal=True,shape=list(image.shape),benchmark=rows)
    Path(args.output).write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps([dict(kernel=r['kernel'],threads=r['threads'],median_ms=r['median_ms'],p90_ms=r['p90_ms']) for r in rows]),flush=True)


if __name__=='__main__':main()
