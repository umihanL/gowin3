"""Plot measured RTL outputs for the 9-degree-misaligned pitch round trip."""
from pathlib import Path
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parents[1]
with (root/'sim/imu_rotation_trace.csv').open() as f:
    rows=list(csv.DictReader(f))
fig,axes=plt.subplots(1,2,figsize=(12,4),sharex=True,sharey=True,layout='constrained')
for ax,cols,title in zip(axes,
    [('euler_roll','euler_pitch','euler_yaw'),('rot_x','rot_y','rot_z')],
    ['Previous ZYX Euler output','New quaternion rotation-vector output']):
    for col,color in zip(cols,('#ca7a00','#007fa3','#b5344b')):
        ax.plot([int(r['sample']) for r in rows],[int(r[col])/10 for r in rows],
                color=color,label=col.replace('euler_','').replace('rot_','Rot ').title(),linewidth=1.6)
    ax.set_title(title);ax.set_xlabel('Sample: +150 deg, return, -150 deg, return')
    ax.grid(alpha=.25);ax.set_ylim(-185,185);ax.legend(loc='upper right',fontsize=9)
axes[0].set_ylabel('Degrees')
fig.suptitle('RTL simulation: rotate about an axis 9 degrees from Y; same input for both outputs',fontsize=11)
fig.savefig(root/'docs/imu_pitch_continuity.png',dpi=160)
print('Saved docs/imu_pitch_continuity.png (simulation, not a hardware capture).')
print('Maximum Euler Pitch:',max(abs(int(r['euler_pitch'])) for r in rows)/10)
print('Final rotation vector:',[int(rows[-1][k])/10 for k in ('rot_x','rot_y','rot_z')])
