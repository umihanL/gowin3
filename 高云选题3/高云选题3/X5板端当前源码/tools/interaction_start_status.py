"""Describe live startup readiness without issuing commands or changing profiles."""

def describe(state):
    topics = state.get('topics', {})
    actuator_topic = topics.get('actuators', {})
    actuator = actuator_topic.get('data') or {}
    manual_topic = topics.get('gimbal_manual', {})
    manual = manual_topic.get('data') or {}
    voice_topic = topics.get('voice', {})
    voice = voice_topic.get('data') or {}
    voice_off = voice_topic.get('fresh') is True and voice.get('voice_enabled') is False
    lines = [('文字指令接口已就绪；语音总开关已关闭，收音、识别与播报暂停。'
              if voice_off else '任务与点击说话接口已就绪；启动检查没有发出录音、运动或喷水指令。')]
    if voice_off:
        lines.append('需要语音时，在任务驾驶台“车载语音”中点击“开启语音”；文字导航与文字问答继续可用。')
    if actuator_topic.get('fresh') is not True:
        lines.append('云台：尚未收到新鲜执行器状态，不能据此认定电机连接成功。')
    elif actuator.get('error'):
        lines.append('云台：执行器报告异常：' + str(actuator['error']))
    elif actuator.get('reference_confirmed_this_start') is not True:
        lines.append('云台：本次开机参考尚未确认。启动服务不会自动把当前位置设为零位。')
        if actuator.get('dry_run') is False and actuator.get('raw_feedback_fresh') is True:
            lines.append('双轴真实串口回读已在线；到“设备调试 → 云台到角度”，摆好机械零位后点击确认即可使用角度控制。')
    elif actuator.get('dry_run') is True or actuator.get('enabled') is not True:
        lines.append('云台：真实输出尚未启用。')
    elif manual_topic.get('fresh') is True and manual.get('can_start') is True:
        lines.append('云台：当前可以接受角度指令。进入“设备调试 → 云台到角度”。')
    else:
        lines.append('云台：当前不能接受新运动指令，原因：' + str(manual.get('ready_reason') or '等待实时状态'))
    if actuator_topic.get('fresh') is True and actuator.get('enabled') is True and actuator.get('dry_run') is False:
        lines.append('正式云台正在占用电机串口；“双电机空载调试”的连接按钮会禁用，不代表串口断线。')
    lines.append('“回待机位”目标为水平0°、俯仰30°；它沿用已确认参考，不是重新清零。')
    return lines
