#!/usr/bin/env python3
"""Select saved stereo/lens files consistently for perception and tasks."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--lens-only', action='store_true')
    args = parser.parse_args()
    config = args.workspace.resolve() / 'config'
    choices = {
        'lens_intrinsics': ('lens_intrinsics.local.json', 'gs130w_intrinsics.factory-mapped.json', 'gs130w_intrinsics_20261010.json'),
        'stereo_calibration': ('stereo.local.json', 'stereo.factory.json', 'stereo.template.json'),
        'stereo_runtime': ('stereo_runtime.local.json', 'stereo_runtime.factory.json', 'stereo_runtime.template.json'),
    }
    for key, names in choices.items():
        if args.lens_only and key != 'lens_intrinsics':
            continue
        path = next((config / name for name in names if (config / name).is_file()), None)
        if path is None:
            parser.error('Missing ' + key + ' configuration')
        try:
            value = json.loads(path.read_text(encoding='utf-8-sig'))
            if not isinstance(value, dict):
                raise ValueError('JSON object required')
        except (OSError, ValueError) as exc:
            parser.error(str(path) + ': ' + str(exc))
        print(key + ':=' + str(path))


if __name__ == '__main__':
    main()
