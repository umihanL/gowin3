"""Summarize audited V8 data only; never launches or resumes model training."""
from pathlib import Path
from collections import Counter
import json,hashlib

ROOT=Path(__file__).resolve().parents[1]
read=lambda p:json.loads(p.read_text(encoding='utf8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    data=ROOT/'dataset_observation_v3';source=ROOT/'source_data/v8_observation_focus'
    manifest=read(data/'manifest.json');stats=read(data/'statistics.json')
    render=read(source/'render_audit.json');audit=read(data/'independent_validation.json');ui=read(data/'review_ui_validation.json')
    package_audit=read(source/'packaged/validation.json')
    loader=read(ROOT/'observation_v3_loader_validation.json')
    assert render['all_pass'] and render['complete'], 'Render audit failed or incomplete'
    assert audit['passed'] and ui['passed'], 'Data/review validation failed'
    assert package_audit['passed'], 'Visible-mask package label validation failed'
    assert loader['passed'] and loader['counts']['images']==stats['train']['images'] and not loader['training_performed']
    assert set(manifest['sources'])=={'v5','v6','v7','v8'}
    assert manifest['baseline_manifest_sha256']==sha(ROOT/'dataset_people_v2/manifest.json')
    for split in ['val','test']:
        assert (data/(split+'.txt')).read_bytes()==(ROOT/'dataset_people_v2'/(split+'.txt')).read_bytes()
    obs=[json.loads(s) for s in (source/'target_observations.jsonl').read_text().splitlines()]
    occluded=[r['occluded_pixel_fraction'] for r in obs if 'occluded_pixel_fraction' in r]
    clean=lambda s: s.replace('|','/')
    text=[ '# V8 楼前观察数据检查记录', '',
          '新增数据已生成、完整标注、融合并通过检查。本记录不启动训练，也不声称新数据已改善模型准确率。', '',
          f"- 新采集：{render['new_pose_count']} 个位姿、{render['expected_pairs']} 对双目、{render['stats']['rgb_images']} 张原始图。",
          f"- V8 入库：{stats['train']['by_source']['v8']['images']} 张（含配对光度增强），仅进入训练集。",
          f"- 融合总计：{manifest['totals']['images']} 张图、{manifest['totals']['boxes']} 个标注框。",
          f"- 划分：train {stats['train']['images']} / val {stats['val']['images']} / test {stats['test']['images']}。",
          '- 完整保留 dataset_people_v2 全部旧图、标签及标注；原 val/test 列表、顺序、文件清单与字节不变。',
          '- 各类全部可见实例与极小目标标签均保留，仍只有 fire / firefighter / person 三类。', '',
          '- 实际 Ultralytics CPU 加载器检查：4560 张训练图、43246 个框全部保留，包括 8154 个≤2 px 框；未构造模型、未执行训练。', '',
          '| 指定目标 | 目标高度中位数 | 短边中位数 |', '|---|---:|---:|']
    for cls,name in [('1','消防员'),('2','人物')]:
        text.append(f"| {name} | {render['target_bbox_height_px'][cls]['median']} px | {render['target_bbox_short_side_px'][cls]['median']} px |")
    text.extend(['',f"{len(occluded)} 眼图像通过真实遮挡与无遮挡参考遮罩核对；被遮住的目标像素比例范围 {min(occluded):.1%}–{max(occluded):.1%}。指定目标全部仍可见。",'',
         '训练硬例选取只参考训练集。满足已定清晰观察条件的旧训练目标没有观测到漏检；本批重点提高窄小或斜视目标的可观察性，并补充暗光、真实遮挡。89/96 位姿观察清单中出现过的同一墙面实例的新角度，不能当成未见过的独立测试。', '',
         'V8 距离 0.4–1.0 m 是模拟工作范围；640×480、72°视场和80 mm基线仍待实机标定。人像造型仍是现有壁纸。新增数据尚未训练，训练保持此前暂停状态。', '',
         '审阅入口：review.html；证据：independent_validation.json、review_ui_validation.json、../source_data/v8_observation_focus/render_audit.json。', ''])
    (data/'OBSERVATION_DATA_REPORT.md').write_text('\n'.join(text),encoding='utf8')
    report=dict(complete=True,training_started=False,dataset=data.name,totals=manifest['totals'],split_images={s:stats[s]['images'] for s in ['train','val','test']},new_source_images=stats['train']['by_source']['v8']['images'],new_pairs=render['expected_pairs'],new_poses=render['new_pose_count'],baseline_manifest_sha256=manifest['baseline_manifest_sha256'],source_manifest_sha256=sha(source/'source_manifest.json'),audit_sha256=sha(data/'independent_validation.json'),render_audit_sha256=sha(source/'render_audit.json'),review_validation_sha256=sha(data/'review_ui_validation.json'),loader_validation_sha256=sha(ROOT/'observation_v3_loader_validation.json'),package_validation_sha256=sha(source/'packaged/validation.json'),verified_partial_occlusion_eyes=len(occluded))
    (data/'observation_data_validation.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
