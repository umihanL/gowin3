"""Build the static map ROMs and the small Chinese OSD font from local assets."""
from pathlib import Path
import hashlib
import json
import math
import re
from collections import deque
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'src/navigation'
LABELS = [
    '', '智能小车', '实时数据', '遥控模式', '实时避障', '自主导航',
    '障碍物检测', '自动减速停车', '自主绕障', '路径设置', '开始导航',
    '起点', '终点', '路径点', '清除', '规划路径', '返回',
    '选择位置', '规划中', '规划完成', '无法到达', '请设置起终点',
    '位置不可通行', '路径点已满', '等待规划', '功能待接入',
    '导航就绪', '地图', '厘米', '路线已失效', '代价超限',
    '起', '终', '点', '地图未标定', '停止导航', '路程',
    '平滑校验中', '转弯半径不足', '车身碰撞风险', '路线点数超限',
    '前方障碍物距离：', '左前方', '右前方', '左侧', '右侧', '两秒后启动',
    '运动中', '已完成', '传感器异常', '通道不足', '朝向不符', '未规划',
    '编码器异常', '车轮无反馈', '速度', '转角', '位置', '航向', '目标',
    '剩余', '阶段', '遥控未连接', '等待启动',
    '设置', '模式选择', '参数设置', '手动模式', '目标速度',
    '键盘输入', '按键调节', '删除', '确认', '粗调', '细调',
    '前进', '后退', '左转', '右转', '回正', '停止', '触摸驾驶',
    '速度调节中', '输入超限', '已保存',
    '速度设置', '减速起始距离', '输入厘米', '两侧受阻减速停车',
    '绕障触发距离', '减速初始速度', '禁行挡板', '清除挡板', '依次点击挡板两端',
    '原始图像', '二值图像', '通信模式', '', '阈值',
    '', '', '',
    '自主避障', '关闭', '开启',
    '拓展1视觉识别', '拓展2对话控制', '上位机选择任务', '斑马线检测',
    '舵机中值', '输出限位', '允许范围', '度',
]


def write_mem(name, values, digits=1):
    (OUT / name).write_text(''.join(f'{int(v):0{digits}x}\n' for v in values), encoding='ascii')


def build_map():
    source = ROOT / '\u5730\u56fe.jpg'
    image = Image.open(source).convert('RGB')
    rgb = np.asarray(image)
    # Segment the uniform charcoal road material, then restore painted markings.
    road = Image.fromarray(np.uint8((rgb.max(axis=2) < 100) & (rgb.min(axis=2) > 22)) * 255)
    draw = ImageDraw.Draw(road)
    for box in [(600, 194, 660, 245), (234, 591, 296, 642), (597, 596, 662, 647)]:
        draw.rectangle(box, fill=255)
    free = np.asarray(road.resize((352, 320), Image.Resampling.BOX)) >= 250
    # Keep the connected road network, discard dark decorative building outlines.
    visited = set()
    components = []
    for row, col in zip(*np.nonzero(free)):
        if (row, col) in visited:
            continue
        todo = deque([(row, col)])
        visited.add((row, col))
        component = []
        while todo:
            r, c = todo.popleft()
            component.append((r, c))
            for nr, nc in ((r-1,c), (r+1,c), (r,c-1), (r,c+1)):
                if 0 <= nr < 320 and 0 <= nc < 352 and free[nr,nc] and (nr,nc) not in visited:
                    visited.add((nr,nc))
                    todo.append((nr,nc))
        components.append(component)
    road_cells = max(components, key=len)
    free = np.zeros_like(free)
    for r, c in road_cells:
        free[r,c] = True
    # Euclidean disk; out-of-map cells are obstacles as well.
    padded = np.pad(~free, 10, constant_values=True)
    blocked = np.zeros_like(free)
    for dy in range(-10, 11):
        for dx in range(-10, 11):
            if dx*dx + dy*dy <= 9.5**2:
                blocked |= padded[10+dy:330+dy, 10+dx:362+dx]
    coarse = blocked.reshape(160, 2, 176, 2).any(axis=(1, 3))
    # Eight body orientations at each rear-axle grid centre. This steers A*
    # away from inside corners where width-only inflation is insufficient.
    heading = np.zeros((160, 176), dtype=np.uint8)
    exact_heading = np.zeros_like(heading)
    padding = 28
    raw_padded = np.pad(~free, padding, constant_values=True)
    for angle_step in range(32):
        angle = angle_step*math.pi/16
        hx, hy = math.cos(angle), math.sin(angle)
        norm = 1
        hit = np.zeros_like(free)
        exact_hit = np.zeros_like(free)
        for oy in range(-27, 28):
            for ox in range(-27, 28):
                forward = ((ox+.5)*hx+(oy+.5)*hy)/norm
                side = ((oy+.5)*hx-(ox+.5)*hy)/norm
                # Search filter is optimistic by one coarse-cell half diagonal.
                # Exact dimensions plus sampling margin are applied after smoothing.
                if -5.5 <= forward <= 17.5 and abs(side) <= 8.0:
                    hit |= raw_padded[padding+oy:padding+oy+320,
                                      padding+ox:padding+ox+352]
                if -7.8 <= forward <= 19.8 and abs(side) <= 10.3:
                    exact_hit |= raw_padded[padding+oy:padding+oy+320,
                                            padding+ox:padding+ox+352]
        for direction, (dx, dy) in enumerate([(1,0),(-1,0),(0,1),(0,-1),
                                             (1,1),(-1,-1),(1,-1),(-1,1)]):
            delta = (angle-math.atan2(dy,dx)+math.pi)%(2*math.pi)-math.pi
            # Optimistic octant bins avoid excluding an intermediate tangent;
            # the refined, continuous heading is checked separately in RTL.
            if abs(delta) <= math.pi/8+1e-8:
                heading |= np.uint8(~hit[1::2, 1::2]) << direction
                exact_heading |= np.uint8(~exact_hit[1::2, 1::2]) << direction
    coarse = heading == 0
    axes = np.zeros_like(heading)
    for axis in range(4):
        axes |= np.uint8((exact_heading & (3 << (axis*2))) != 0) << axis
    # Search does not track heading state; opposite directions share one axis.
    heading = np.zeros_like(axes)
    for axis in range(4):
        heading |= np.uint8((axes & (1 << axis)) != 0) * (3 << (axis*2))
    central_hit = np.zeros_like(free)
    for oy in range(-12, 13):
        for ox in range(-12, 13):
            if ox*ox+oy*oy <= 11.5**2:
                central_hit |= raw_padded[padding+oy:padding+oy+320,
                                         padding+ox:padding+ox+352]
    central = ~central_hit[1::2, 1::2]
    write_mem('map_raw_1cm.mem', (~free).flat)
    write_mem('map_plan_2cm.mem', coarse.flat)
    write_mem('map_heading_2cm.mem', heading.flat, 2)
    write_mem('map_central_2cm.mem', central.flat)
    # Supplied annotations are measurements, not independently surveyed geometry.
    measured_mm = [3271, 2952, 715, 1200, 1430, 807, 1468, 533, 545, 510, 794, 1210, 262]
    header = [0x4e415631, 3520, 3200, 10, 190, 300, 95, 20, 352, 320, 176, 160,
              1, len(measured_mm), *measured_mm]
    # Bounds of the seven non-road islands: millimetres in the world frame.
    island_pixels = [(99,102,236,282), (99,336,236,538), (99,591,236,718),
                     (293,102,601,410), (293,346,601,718),
                     (659,102,797,465), (659,518,797,718)]
    island_bounds_mm = [
        [round(x0*3520/895), round((817-y1)*3200/817),
         round(x1*3520/895), round((817-y0)*3200/817)]
        for x0,y0,x1,y1 in island_pixels]
    header += [len(island_bounds_mm), *[v for box in island_bounds_mm for v in box]]
    # Provisional Ackermann geometry supplied by the owner. Body overhangs
    # assume centred axles: 26 cm body, 12 cm wheelbase, 7 cm each overhang.
    vehicle_header = (ROOT/'src/vehicle/vehicle_config.vh').read_text(encoding='ascii')
    vehicle_cfg = {k: int(v) for k,v in re.findall(r'^`define (VEHICLE_\w+) (-?\d+)$', vehicle_header, re.M)}
    header += [vehicle_cfg['VEHICLE_WHEELBASE_MM'], vehicle_cfg['VEHICLE_MAX_STEER_DEG'],
               vehicle_cfg['VEHICLE_MIN_RADIUS_MM'], 230, 70, 8, 512, 3]
    write_mem('map_metadata.mem', header + [0]*(64-len(header)), 8)
    # Gowin silently optimized the old scalar $readmemb arrays to constants.
    # Use explicit 16-bit initializers in a Verilog include; no runtime path.
    init = ['// Generated map words. Bit 0 is the lowest raster address.']
    for array, values in [('raw_map', (~free).flat), ('plan_map', coarse.flat)]:
        values = list(values)
        for offset in range(0, len(values), 16):
            word = sum(int(v) << bit for bit, v in enumerate(values[offset:offset+16]))
            init.append(f"{array}[{offset//16}]=16'h{word:04x};")
    for index, word in enumerate(header + [0]*(64-len(header))):
        init.append(f"dimensions[{index}]=32'h{word:08x};")
    heading_values = list((axes | (np.uint8(central)<<4)).flat)
    for index in range(0, len(heading_values), 16):
        word = sum(int(v) << (5*bit) for bit, v in enumerate(heading_values[index:index+16]))
        init.append(f"heading_map[{index//16}]=80'h{word:020x};")
    (OUT/'nav_map_init.vh').write_text('\n'.join(init)+'\n', encoding='ascii')
    signature = 0
    for value in header:
        signature ^= value
    (OUT/'nav_metadata.vh').write_text(
        f"localparam [31:0] MAP_SIGNATURE=32'h{signature:08x};\n", encoding='ascii')
    metadata = dict(width_cm=352, height_cm=320, rom_resolution_cm=1,
                    planner_resolution_cm=2, car_width_mm=190, car_length_mm=300,
                    inflation_radius_cm=9.5, origin='bottom-left; x right; y up',
                    wheelbase_mm=vehicle_cfg['VEHICLE_WHEELBASE_MM'], max_front_wheel_angle_deg=vehicle_cfg['VEHICLE_MAX_STEER_DEG'],
                    min_rear_axle_turn_radius_mm=vehicle_cfg['VEHICLE_MIN_RADIUS_MM'],
                    front_from_rear_axle_mm=230, rear_overhang_mm=70,
                    path_reference='rear axle midpoint; provisional centred axles',
                    address='top-down raster: row*width+column',
                    planner_cell_centre='x_mm=20*x+10; y_mm=3200-(20*y+10)',
                    measured_dimensions_mm=measured_mm,
                    island_bounds_mm=island_bounds_mm,
                    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                    raw_free_cells=int(free.sum()), planner_free_cells=int((~coarse).sum()),
                    bridge='static deck assumed traversable; no live bridge state',
                    calibration='image-derived; confirm against physical map before driving')
    (OUT / 'map_metadata.json').write_text(json.dumps(metadata, indent=2)+'\n', encoding='ascii')
    colors = np.empty((320, 352, 3), dtype=np.uint8)
    colors[:] = (55, 64, 68)
    colors[free] = (236, 242, 242)
    colors[free & blocked] = (183, 199, 193)
    preview = Image.fromarray(colors).resize((704, 640), Image.Resampling.NEAREST)
    preview.save(ROOT / 'docs/map_rom_preview.png')


def build_font():
    ascii_chars = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZacdegimnoprstuvxyz:.,/+-> '
    chars = list(dict.fromkeys(''.join(LABELS) + ascii_chars))
    assert len(chars)<=255, 'OSD glyph code exceeds eight bits'
    font = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 15)
    lines = ['// Generated by tools/generate_navigation_assets.py.',
             'function [255:0] nav_glyph;', 'input [7:0] code;', 'begin case(code)']
    rows = ['0000'] * 4096
    for index, char in enumerate(chars, 1):
        tile = Image.new('1', (16, 16))
        painter = ImageDraw.Draw(tile)
        box = painter.textbbox((0, 0), char, font=font)
        painter.text(((16-box[2]+box[0])//2-box[0], -box[1]), char, font=font, fill=1)
        bits = ''.join('1' if p else '0' for p in tile.get_flattened_data())
        for y in range(16):
            rows[(index << 4) + y] = f'{int(bits[y*16:(y+1)*16], 2):04x}'
        lines.append(f"8'd{index}: nav_glyph=256'h{int(bits, 2):064x};")
    lines += ["default:nav_glyph=0;", 'endcase end endfunction',
              'function [7:0] nav_label;', 'input [6:0] label;', 'input [3:0] column;',
              'begin nav_label=0; case(label)']
    for index, label in enumerate(LABELS):
        if label:
            lines.append(f"7'd{index}: case(column)")
            for column, char in enumerate(label):
                lines.append(f"4'd{column}:nav_label=8'd{chars.index(char)+1};")
            lines.append('default:nav_label=0; endcase')
    lines += ['default:nav_label=0;', 'endcase end endfunction',
              'function [7:0] nav_ascii;', 'input [7:0] ascii;', 'begin case(ascii)']
    for char in ascii_chars:
        lines.append(f"8'd{ord(char)}:nav_ascii=8'd{chars.index(char)+1};")
    lines += ['default:nav_ascii=0;', 'endcase end endfunction']
    lines += ['function [3:0] nav_label_length;', 'input [6:0] label;', 'begin case(label)']
    for index, label in enumerate(LABELS):
        lines.append(f"7'd{index}:nav_label_length=4'd{len(label)};")
    lines += ['default:nav_label_length=0;', 'endcase end endfunction']
    (OUT/'nav_font.vh').write_text('\n'.join(lines)+'\n', encoding='ascii')
    (OUT/'nav_font_rows.mem').write_text('\n'.join(rows)+'\n', encoding='ascii')


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    build_map()
    build_font()
    print('Generated 352x320 raw ROM, conservative 176x160 planning ROM, metadata and OSD font.')
