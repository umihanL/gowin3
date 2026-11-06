"""Vendor the previously verified YOLO/aiming code and candidate BIN with hashes."""
from pathlib import Path
import hashlib
import json
import shutil
import argparse

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description='Historical vendor refresh only. Normal deployment does not run this tool.')
parser.add_argument('--legacy-package',type=Path,required=True,help='Explicit previous rdkx5_firefighter_v4 directory')
parser.add_argument('--refresh-vendored-files',action='store_true',help='Overwrite vendored algorithm files and calibration templates from the source')
args=parser.parse_args()
if not args.refresh_vendored_files:parser.error('Normal setup already includes these files. Explicit --refresh-vendored-files is required.')
previous=args.legacy_package.resolve()
package=root/'src/rescue_vision/rescue_vision'
records=[]
for original,new in [('detector.py','legacy_detector.py'),('aiming.py','legacy_aiming.py'),
                     ('visual_servo.py','legacy_visual_servo.py'),('landing_calibration.py','legacy_landing_calibration.py')]:
    src=previous/'runtime'/original;dst=package/new
    shutil.copy2(src,dst)
    records.append(dict(source=str(src),path=dst.relative_to(root).as_posix(),sha256=hashlib.sha256(dst.read_bytes()).hexdigest()))
dest=root/'models/yolo';dest.mkdir(parents=True,exist_ok=True)
for name in ['wallpaper_yolo11n_3class_bayese_640_nv12.bin']:
    shutil.copy2(previous/'models'/name,dest/name)
for name in ['model_contract.json','deployment_status.json']:
    shutil.copy2(previous/name,dest/name)
shutil.copy2(previous/'runtime/servo_config.template.json',root/'config/servo.template.json')
shutil.copy2(previous/'runtime/landing_calibration.template.json',root/'config/landing.template.json')
vendor=root/'vendor';vendor.mkdir(exist_ok=True)
(vendor/'previous_runtime.json').write_text(json.dumps(records,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
shutil.copy2(previous/'vendor/LICENSE',vendor/'D-Robotics-LICENSE')
print('Copied existing runtime and candidate BIN; original package unchanged.')
