#!/usr/bin/env python3
"""Convert the saved GS130W EEPROM snapshot into a stereo parameter file."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rescue_vision'))
from rescue_vision.factory_stereo import factory_lens_snapshot, import_factory_stereo


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path,
                        default=ROOT / 'config/gs130w_intrinsics_20261010.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--left-source', help='Raw image topic used as the first input; supply both or neither')
    parser.add_argument('--right-source', help='Raw image topic used as the second input')
    parser.add_argument('--left-calibration-source', help='Snapshot EEPROM parameter slot for the first input')
    parser.add_argument('--right-calibration-source', help='Snapshot EEPROM parameter slot for the second input')
    parser.add_argument('--mapping-evidence', type=Path, help='Existing mapping evidence report to record and hash')
    parser.add_argument('--lens-output', type=Path, help='Write a separate mapped snapshot for LensCorrector')
    parser.add_argument('--translation-unit', choices=('mm', 'm'), default='mm',
                        help='Units of factory P-derived translation before conversion (GS130W: mm)')
    parser.add_argument('--force', action='store_true', help='Replace an existing output file')
    args = parser.parse_args(argv)
    try:
        output = args.output.resolve()
        outputs = [output] + ([args.lens_output.resolve()] if args.lens_output else [])
        for candidate in outputs:
            if (candidate == args.snapshot.resolve() or
                    (candidate.exists() and candidate.samefile(args.snapshot))):
                raise ValueError('Output must not replace the source EEPROM snapshot')
            if candidate.exists() and not args.force:
                raise ValueError('Output already exists; choose a new path or use --force: ' + str(candidate))
        if len(outputs) == 2 and (outputs[0] == outputs[1] or
                (all(path.exists() for path in outputs) and outputs[0].samefile(outputs[1]))):
            raise ValueError('Stereo and lens outputs must be different files')
        document = import_factory_stereo(args.snapshot, left_source=args.left_source,
                                         right_source=args.right_source,
                                         left_calibration_source=args.left_calibration_source,
                                         right_calibration_source=args.right_calibration_source,
                                         translation_unit=args.translation_unit,
                                         mapping_evidence=args.mapping_evidence)
        documents = [document] + ([factory_lens_snapshot(args.snapshot, document)] if args.lens_output else [])
        payloads = [json.dumps(item, ensure_ascii=False, indent=2, allow_nan=False) + '\n' for item in documents]
        for destination, payload in zip(outputs, payloads):
            destination.parent.mkdir(parents=True, exist_ok=True)
            with destination.open('w' if args.force else 'x', encoding='utf-8', newline='\n') as stream:
                stream.write(payload)
    except (OSError, ValueError) as exc:
        parser.exit(2, 'Factory stereo import failed: ' + str(exc) + '\n')
    print(json.dumps({
        'output': str(output), 'calibration_id': document['calibration_id'],
        'source_topics': document['source_topics'], 'image_size': document['image_size'],
        'calibration_source_topics': document['calibration_source_topics'],
        'lens_output': str(outputs[1]) if args.lens_output else None,
        'baseline_m': document['baseline_m'],
        'positive_disparity_compatible': document['positive_disparity_compatible'],
        'metric_depth_verified': False, 'physical_eye_mapping_verified': False,
    }, ensure_ascii=False, indent=2))
    if not document['positive_disparity_compatible']:
        print('The requested source order is incompatible with the current positive-disparity SGBM. '
              'Its real R/T were retained; no sign correction was applied.', file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
