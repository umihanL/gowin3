"""Offline image/encoder control acceptance. Never opens UART, ROS or GPIO."""
import json
import argparse
from pathlib import Path
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests'))
from test_measured_visual_servo import run_simulation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'reports/axis_swap_20261031/gimbal_closed_loop')
    out=parser.parse_args().output_dir
    out.mkdir(parents=True,exist_ok=True)
    summary={'scope':'OFFLINE SYNTHETIC IMAGE + TWO MOTOR ENCODER PLANT; NOT HARDWARE ACCEPTANCE',
             'hardware_verified':False,'pump_used':False,'real_serial_opened':False,
             'image_size_wh':[1280,1088],'image_delay_s':.1,
             'axis_protocols':['zdt_xs','pd42'],
             'axis_ports':['/dev/ttyS3','/dev/ttyS1'],
             'detector':'Known coloured raster marker; YOLO accuracy is not evaluated', 'scenarios':[]}
    fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    for row,dynamic in enumerate((False,True)):
        name='moving_target' if dynamic else 'static_target'
        records,frames,events=run_simulation(dynamic=dynamic)
        (out/(name+'.json')).write_text(json.dumps(records,indent=2),encoding='utf-8')
        t=np.array([r['t'] for r in records])
        actual=np.array([r['actual_angles_deg'] for r in records])
        target=np.array([r['target_deg'] for r in records])
        command=np.array([r['commanded_deg'] for r in records])
        residual=np.array([r['error_deg'] for r in records])
        summary['scenarios'].append(dict(name=name,final_error_deg=residual[-1].tolist(),
            rms_error_after_6_5s_deg=float(np.sqrt(np.mean(residual[t>=6.5]**2))),
            maximum_overshoot_deg=np.maximum(0,np.max(actual-target,axis=0)).tolist(),
            image_lock_seen=any(r['image_locked'] for r in records),
            serial_exchanges=len(events)))
        for col,label in enumerate(('Pan (ZDT_XS / PIN 3,5)','Tilt (PD42 / PIN 8,10)')):
            ax=axes[row,col]
            ax.plot(t,target[:,col],color='#bc4b21',label='World target',linewidth=1.5)
            ax.plot(t,actual[:,col],color='#176b92',label='Encoder measurement',linewidth=1.7)
            ax.plot(t,command[:,col],color='#69895d',alpha=.6,linestyle='--',label='Motor command')
            ax.set(title=name.replace('_',' ')+' / '+label,xlabel='Time (s)',ylabel='Angle (deg)')
            ax.grid(alpha=.2);ax.legend(fontsize=8)
        if not dynamic:
            for frame,image in frames.items():cv2.imwrite(str(out/f'frame_{frame:03}.png'),image)
    fig.suptitle('Offline closed-loop acceptance: 1280 x 1088 images, 100 ms delay; no physical hardware')
    fig.savefig(out/'trajectory.png',dpi=160)
    plt.close(fig)
    (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    (out/'review.html').write_text('''<!doctype html><meta charset="utf-8"><title>两轴离线闭环验收</title>
<style>body{background:#121b25;color:#e9f0f8;font:16px system-ui;margin:32px;max-width:1200px}img{max-width:100%;border-radius:8px}.frames{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}small{color:#b3c4d3}</style>
<h1>两轴视觉闭环：离线验证</h1><p>此页是确定性合成图像与电机编码器模型的验收记录，不是实机演示。没有连接串口、ROS 图或喷水；未验证 YOLO 准确率、水流命中、真实电机方向与负载。</p>
<p>1280×1088 图像 → 测试标记提取 → 正式 FireAimTracker → 标定角度误差 → 编码器历史补偿 → 正式混合电机协议 → 带速度上限的两轴模型 → 编码器回读。</p>
<p>当前分工：水平为ZDT_XS（PIN3/5），俯仰为PD42（PIN8/10）；接线未交换。</p>
<img src="trajectory.png" alt="两轴目标、命令与编码器角度轨迹"><h2>输入图像序列</h2><div class="frames">'''+''.join(
        f'<div><img src="frame_{n:03}.png"><small>第 {n} 帧</small></div>' for n in (0,20,50,100,160,240))+'''
</div><p>原始时间序列：static_target.json、moving_target.json；误差统计：summary.json。每一条命令保留原编码器参考，未执行清零或回零。</p>''',encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
