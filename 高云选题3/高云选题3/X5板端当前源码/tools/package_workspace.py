"""Package the canonical system, portable models, guides and measured evidence.

Run after core, ROS/UART, web and audio checks. Does not include runtime tokens,
training environments, original datasets, download caches or container layers.
"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import re
import zipfile

WS=Path(__file__).resolve().parents[1]
SYSTEM=WS.parent

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def read(path):return json.loads((WS/path).read_text(encoding='utf-8-sig'))

def main():
    core=(WS/'reports/system_core_tests.log').read_text(encoding='utf-8-sig')
    if not re.search(r'^OK\s*$',core,re.M):raise RuntimeError('Core tests not passed')
    count=int(re.search(r'Ran (\d+) tests',core).group(1))
    required=['ros_integration','system_ros_integration','launch_smoke','telemetry_validation','telemetry_ros_integration']
    results={name:read('reports/'+name+'.json') for name in required}
    for name,result in results.items():
        if result.get('passed') is not True:raise RuntimeError(name+' not passed')
    if len(results['launch_smoke'].get('nodes',[]))!=10:raise RuntimeError('Expected 10 nodes')
    if not results['launch_smoke'].get('installed_web_assets'):raise RuntimeError('Installed web assets not verified')
    voice_ros=read('reports/voice_ros_validation.json')
    audio=read('reports/voice_audio_huihui_validation.json')
    machine_audio=read('reports/voice_audio_validation.json')
    if voice_ros['passed']!=voice_ros['total'] or audio['passed']!=audio['total']:
        raise RuntimeError('Required voice logic or natural synthetic speech check failed')
    status=dict(
        generated_utc=datetime.now(timezone.utc).isoformat(),
        state='integrated_host_verified_pending_real_robot_commissioning',
        core_tests_passed=count,ros_packages_built=2,ros_nodes=10,
        build_mode='ordinary colcon build; workspace web/config copied into share/rescue_vision',
        ros_distribution_tested='ROS 2 Humble / Ubuntu 22.04 x86_64 container',
        baseline_ros_uart_checks=len(results['ros_integration']['checks']),
        navigation_ros_uart_checks=len(results['system_ros_integration']['checks']),
        http_core_tests=results['telemetry_validation']['tests'],
        http_ros_checks=results['telemetry_ros_integration'].get('checks'),
        voice_ros_checks=voice_ros['passed'],
        natural_synthetic_pcm_asr=dict(passed=audio['passed'],total=audio['total'],source=audio['audio_source']),
        machine_synthetic_pcm_asr=dict(passed=machine_audio['passed'],total=machine_audio['total'],source=machine_audio['audio_source']),
        map=dict(nodes=49,edges=55,buildings=13,real_map_registration=False),
        navigation='Dijkstra topology routes; encoder/gyro odometry interface; guarded maneuvers; default preview',
        wire='CRC UART with Linux PTY verification; no physical UART or firmware verification',
        semantic_scene='OCR plus current-view detections; casualties unknown; facade association needs calibration',
        voice='offline Vosk small-cn-0.22 and espeak-ng; USB mic/speaker default disabled',
        wireless='authenticated local HTTP; same-LAN configuration required; no cloud dependency',
        browser_visual_qa=True,
        x5_tested=False,bpu_tested=False,gs130w_calibrated=False,
        real_voice_noise_tested=False,real_fpga_tested=False,
        yolo_quantization_quality_gate_passed=False,fpga_firmware_provided=False,
        physical_navigation_tested=False,pump_actuation_implemented=False)
    (WS/'deployment_status.json').write_text(json.dumps(status,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    paths=set()
    for name in ('src','config','models','docs','tools','tests','reports','web','vendor'):
        for p in (WS/name).rglob('*'):
            if not p.is_file():continue
            if any(part in ('__pycache__','.git','.venv','ocr_python','wheels') or part.endswith('.egg-info') for part in p.parts):continue
            if p.suffix in ('.pyc','.zip','.whl','.token'):continue
            if p.name in ('delivery_check.json','telemetry_demo.log','telemetry_demo_error.log'):continue
            paths.add(p)
    for p in (SYSTEM/'docs').rglob('*'):
        if p.is_file():paths.add(p)
    for name in ('开始学习与使用.md','migration.json'):
        paths.add(SYSTEM/name)
    for name in ('README_开始使用.md','.gitignore','deployment_status.json'):
        paths.add(WS/name)
    # Keep compact integration evidence, not transient runtime logs or secrets.
    for name in ('system_build_final.log','ros_system_regression_driver.log','system_ros_integration_run.log','launch_smoke.log','launch_validation_driver.log','telemetry_ros_integration_run.log'):
        p=WS/'logs'/name
        if p.exists():paths.add(p)
    inventory=[dict(path=p.relative_to(SYSTEM).as_posix(),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(paths)]
    manifest=SYSTEM/'delivery_manifest.json'
    manifest.write_text(json.dumps(inventory,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    # Replace the inherited six-node manifest with the current workspace subset.
    ws_manifest=WS/'delivery_manifest.json'
    ws_manifest.write_text(json.dumps([dict(e,path=e['path'][8:]) for e in inventory if e['path'].startswith('ros2_ws/')],ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    target=SYSTEM/'RDKX5_消防小车系统_部署包.zip'
    with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for p in sorted(paths|{manifest,ws_manifest}):archive.write(p,SYSTEM.name+'/'+p.relative_to(SYSTEM).as_posix())
    with zipfile.ZipFile(target) as archive:
        bad=archive.testzip()
        if bad:raise RuntimeError('ZIP CRC failed: '+bad)
        for entry in inventory:
            data=archive.read(SYSTEM.name+'/'+entry['path'])
            if hashlib.sha256(data).hexdigest()!=entry['sha256']:raise RuntimeError('Archive differs: '+entry['path'])
    result=dict(passed=True,files=len(paths)+2,zip=str(target),bytes=target.stat().st_size,sha256=sha(target),
                excluded='training data/environments; download ZIP cache; wheels; runtime access tokens; Docker layers')
    (WS/'reports/delivery_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))

if __name__=='__main__':main()
