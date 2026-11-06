/* Live robot control: exact-frame camera transport; all actions use the ROS task router. */
'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  const text = (id, value) => { const node = $(id); const next = String(value ?? '—'); if (node && node.textContent !== next) node.textContent = next; };
  const html = (id, value) => { if ($(id).innerHTML !== value) $(id).innerHTML = value; };
  const num = (value, digits = 1) => finite(value) ? value.toFixed(digits) : '—';
  const clock = value => { const d = new Date(finite(value) ? value * 1000 : value); return Number.isNaN(d.getTime()) ? '—' : d.toLocaleTimeString('zh-CN', {hour12:false}); };
  const names = {fire:'火焰',person:'人物',firefighter:'消防员'};
  const colors = {fire:'#FF6A2B',person:'#5B8CFF',firefighter:'#C6F135'};
  const modeNames = {standby:'待机观察',idle:'待机',patrol:'道路巡逻',navigate:'语义导航',inspect:'火场观察',fire:'火焰瞄准',scene:'场景理解',fault:'故障停止',estop:'紧急停止'};
  const phaseNames = {IDLE:'等待红绿灯任务',WAIT_STOP:'停车等待 2 秒',SCANNING:'云台寻找信号灯',WAIT_GREEN:'等待绿灯确认',RELEASED:'已给过街许可',FAULT:'异常 / 保持停车',CROSSING:'等待过街完成',WAIT_CROSSING_COMPLETE:'等待过街完成'};
  const reasons = {mission_not_inspect_or_fire:'当前任务未启用火场识别',mission_timeout:'任务状态超时',traffic_active:'正在处理信号灯',detector_disabled:'目标识别按任务关闭',microphone_disabled:'麦克风未启用',tts_disabled:'车载播报未启用',navigation_disabled:'真实导航尚未启用',fpga_link_not_ready:'FPGA 链路尚未就绪',no_target:'尚未选定目标',explicit_selection_required:'等待稳定火焰目标',unconfigured:'尚未安装标定',mission_not_stopped:'尚无车辆停稳反馈',vehicle_not_stopped:'尚无车辆停稳反馈',no_traffic_light:'尚未找到信号灯',not_detected:'当前未检测到',no_new_frame:'等待新帧',not_ready:'尚未就绪'};
  Object.assign(reasons,{
    manual_pump_disabled:'手动试喷入口尚未在车载启动配置中启用',
    gimbal_raw_position_invalid:'云台位置回读无效',
    manual_pump_fault_or_estop:'存在执行器故障或急停，不能试喷',
    manual_pump_operator_stop_confirmed:'已收到本次停止请求对应的 B13 低电平反馈',
    manual_pump_mission_stale:'任务状态已过期，等待新反馈',
    manual_pump_vehicle_not_stopped:'FPGA 尚未确认车辆停稳，当前不能试喷',
    manual_pump_fpga_reconnecting:'FPGA 串口正在重连，尚无新鲜回包，无法控制 B13',
    manual_pump_fpga_feedback_unavailable:'FPGA 串口无反馈，无法控制 B13',
    waiting_for_three_stationary_readbacks:'等待连续三次静止回读',
    raw_encoder_feedback_stale:'编码器回读已过期',
    gimbal_raw_speed_invalid:'未取得有效的两轴转速',
    gimbal_raw_motion_present:'仍检测到电机转动',
    gimbal_driver_fault_present:'驱动器仍报告故障',
    gimbal_enable_state_unknown:'驱动器使能状态未知',
    gimbal_enable_state_changed:'驱动器使能状态变化，重新确认静止',
    zdt_enabled_retained_target_not_settled:'水平轴仍未到达驱动器保留的目标',
    pd42_enabled_retained_target_not_settled:'俯仰轴仍未到达驱动器保留的目标',
    pan_moved_during_stationary_readback:'静止确认期间水平轴发生移动',
    tilt_moved_during_stationary_readback:'静止确认期间俯仰轴发生移动',
    uart_recovery_handshake_timeout:'FPGA 串口握手超时，尚未收到回包',
    uart_reconnecting_wait_for_fresh_feedback:'FPGA 串口正在重连，等待新鲜回包',
    manual_pump_idle_observation_required:'先停止自动任务，切到火场观察或待机',
    manual_pump_fpga_idle_required:'FPGA尚未确认空闲，先结束导航或交通任务',
    manual_pump_other_actuator_active:'有云台或电机动作正在执行，请先停止',
    manual_pump_waiting_off:'上一轮关水尚未确认，请先检查水泵反馈',
    manual_pump_running:'手动试喷正在执行',
    manual_pump_cooldown:'正在等待两次短喷之间的冷却间隔',
    manual_pump_fresh_off_required:'需要新鲜的B13低电平反馈',
    manual_pump_fpga_not_ready:'FPGA水泵输出尚未就绪',
    manual_pump_physical_control_page_required:'FPGA 未进入“拓展2对话控制”页，B13 未收到试喷开启命令',
    manual_pump_fpga_link_required:'FPGA 控制页已打开，但控制链路尚未确认，未发送试喷开启命令',
    manual_pump_gimbal_moving:'云台仍在转动，停稳后才能按当前喷嘴方向试喷',
    fixed_nozzle_motion_retirement_unverified:'尚未确认之前的云台运动已结束，本次固定喷嘴确认不能替代运动停止回执',
    manual_pump_preparing_off:'试喷准备中：先确认 B13 低电平及可开启状态，尚未喷水',
    manual_pump_prepare_timeout:'试喷准备超时：未确认本次关水与就绪，未发送开启命令',
    manual_pump_prepare_not_submitted:'关水准备命令未送达，未发送开启命令',
    manual_pump_prepare_interrupted:'试喷准备被中止，未发送开启命令',
    manual_pump_waiting_high_readback:'已提交一次短喷，等待B13高电平反馈',
    manual_pump_high_observed:'已收到B13高电平反馈',
    manual_pump_confirming_off:'正在确认B13恢复低电平',
    manual_pump_duration_finished:'喷时已到，等待关水确认',
    manual_pump_high_then_low_confirmed:'已确认B13高电平再回到低电平；实际出水以现场为准',
    manual_pump_high_not_observed_no_retry:'已确认关水，但未采到高电平；不确认本次出水，不会重试',
    manual_pump_off_unconfirmed:'关水反馈尚未确认，保持禁止运动；请检查连接',
    manual_pump_feedback_lost:'水泵反馈失效，已请求关水',
    manual_pump_operator_stop:'已请求立即关水',
    manual_pump_stale_request:'试喷请求已过期，未重发',
    manual_pump_duration_outside_limits:'单次试喷必须在0.1～1秒之间',
    manual_pump_owns_output:'手动试喷正在占用水泵输出',
    selected_fire_evidence_stale:'目标、瞄准或双轴状态已过期，等待新反馈',
    selected_fire_changed_before_submit:'选定火焰已变化，请重新选择后提交',
    one_click_fire_requires_stopped_idle_observation:'请停车并开启火场识别，结束其他自动任务',
    one_click_fire_requires_idle_extension:'请先停止正在运行的导航或区域搜索任务',
    fresh_explicit_fire_selection_required:'请先点击一个稳定火焰编号',
    selected_fire_round_requires_explicit_text:'请明确下达对选定火焰执行一轮灭火的指令',
    selected_fire_round_independent_asr_required:'喷水语音需通过独立复核；也可点击一键灭火按钮',
    selected_required_fire:'正在处理你选定的火焰',
    selected_largest_visible_fire:'已选择当前稳定火焰',
    waiting_for_burst:'等待水泵反馈',
    burst_finished_not_extinguish_proof:'已完成短喷，灭火效果待现场确认',
    fixed_camera_uncalibrated:'相机到喷嘴云台的安装关系尚未标定',
    fixed_camera_selection_required:'请先按画面编号选定火焰',
    fixed_camera_stereo_depth_required:'该火焰暂无已验证的双目三维位置',
    fixed_camera_stereo_calibration_mismatch:'双目标定与当前瞄准配置不一致',
    fixed_camera_frame_mismatch:'火焰坐标系与固定相机标定不一致',
    fixed_camera_depth_stale:'目标三维位置已过期，等待新鲜双目数据',
    fixed_camera_encoder_feedback_required:'等待双轴真实编码器角度',
    fixed_camera_encoder_outside_limits:'编码器角度超出云台机械限位',
    fixed_camera_invalid_actuator_resolution:'电机角度步距参数无效',
    fixed_camera_actuator_resolution_insufficient:'当前电机角度步距过大，无法满足瞄准容差',
    fixed_camera_reference_mismatch:'云台零位与安装标定参考不一致',
    fixed_camera_target_outside_limits:'目标方向超出云台允许范围',
    fixed_camera_target_unreachable:'当前安装位置无法指向该目标',
    fixed_camera_acquiring:'正在确认选定火焰的三维位置',
    fixed_camera_aligning:'正在将喷嘴转向目标三维位置',
    fixed_camera_geometry_locked:'喷嘴角度已到位，请查看水流落点验收状态',
    fixed_camera_water_impact_unmeasured:'固定相机的水流落点尚未实测',
    fixed_camera_straight_ray_policy_mismatch:'直线瞄准配置不一致，请核对几何与执行器的瞄准策略',
    fixed_camera_water_aim_policy_invalid:'瞄准策略无效，尚未开启喷水',
    stationary_flame_confirmation_before_gimbal_aim:'车辆保持停稳，确认当前火焰后由云台瞄准',
    stationary_fire_requires_fresh_confirmation:'当前火焰需要新的稳定观测，车辆保持停稳',
    stationary_fire_waiting_stereo_gimbal_geometry:'车辆已停稳，等待该火焰的双目位置和云台角度计算',
    flame_short_spray_and_output_low_confirmed:'该目标短喷及 B13 恢复低电平已确认，实际灭火效果待观察',
    region_confirmed_fire_lost_before_gimbal_aim:'已确认的火焰在瞄准前丢失，本批未完成，车辆保持停车',
    region_fire_batch_target_identity_mismatch:'本轮喷射与目标编号不一致，任务中止并保持停车',
    region_fire_batch_spray_evidence_missing:'本目标缺少完整的开启、关水回执，未计为已喷，车辆保持停车',
    region_fire_batch_has_unsprayed_targets:'本批仍有已确认火焰未完成短喷，车辆保持停车',
    region_fire_batch_capacity_exceeded:'本批火焰超过队列容量，任务中止并保持停车；超出目标未计为已处理',
    fixed_camera_landing_outside_measured_grid:'目标角度或距离超出水流实测范围',
    fixed_camera_landing_geometry_reference_mismatch:'落点表与当前双目、安装几何或零位不一致',
    fixed_camera_landing_active_profile_mismatch:'落点表与当前水泵设置、安装或启动参考不一致',
    fixed_camera_compensated_target_outside_limits:'水流补偿后的角度超出云台行程'
  });
  Object.assign(reasons, {
    ros_unavailable:'ROS 尚未连接',task_router_unavailable:'任务路由未就绪',
    real_fpga_feedback_required:'等待 FPGA 真实且新鲜的状态反馈',real_uart_not_enabled:'UART 尚未启用真实通信',
    real_actuator_feedback_required:'等待真实执行器状态反馈',real_actuator_not_enabled:'云台执行器尚未启用真实输出',
    real_pump_not_enabled:'真实水泵输出尚未启用',actuator_fault_or_estop:'执行器故障或急停中',
    fpga_pump_firmware_required:'需要支持 B13 水枪控制的 FPGA 固件和串口连接',
    fpga_pump_ready_feedback_required:'等待 FPGA B13 关闭且可开启的状态回报',
    fpga_pump_fresh_off_ready_required:'等待 FPGA B13 关闭且可开启的状态回报',
    fpga_pump_feedback_stale:'FPGA B13 状态已过期',
    fpga_pump_feedback_unavailable:'FPGA B13 输出状态未知',
    fpga_pump_awaiting_readback:'等待 FPGA B13 输出状态回读',
    fpga_pump_normal:'FPGA B13 状态正常',fpga_pump_pulse_complete:'本次喷水脉冲已到时',
    fpga_pump_link_lost:'FPGA 通信失联，B13 已按固件关闭',
    fpga_pump_scope_lost:'已退出 FPGA 控制页面',fpga_pump_vehicle_moving:'车辆移动，停止喷水',
    fpga_pump_stopped:'已发送停止指令',fpga_pump_mode_or_motion_changed:'任务切换，停止喷水',
    fpga_pump_reset:'FPGA 重启后等待关泵初始化',
    fpga_pump_idle_vehicle_required:'请先结束导航并确认车辆停稳',
    fpga_pump_request_expired_or_cancelled:'喷水请求已过期或取消，未重新执行',
    fpga_pump_ack_timeout_no_retry:'喷水应答超时，请核对 B13 状态；不会自动重发',
    waiting_for_fpga_pump_ready:'已停稳，等待 FPGA B13 关水与就绪回包后再授权本轮短喷',
    real_navigation_not_enabled:'FPGA 实际导航尚未启用，未下发行驶任务',real_preview_required:'请连接真实车载相机预览',
    mission_state_stale:'任务状态已过期',voice_subscriber_unavailable:'语音节点未就绪',
    microphone_disabled_or_voice_status_stale:'麦克风未启用或语音状态过期',
    no_receipt_no_automatic_retry:'未收到回执  不自动重发',
    listen_dispatched_observe_voice_status_for_outcome:'收音请求已发出，请看语音反馈',
    superseded_by_stop:'已由停止指令取消',queue_ttl_exceeded:'指令排队超时',command_queue_full:'指令队列已满',
    same_origin_required:'请求来源不匹配',host_not_allowed:'连接地址未获允许',
    idle:'待命',accepted:'已接受请求',navigating:'导航进行中',arrived:'已反馈到达',paused:'已暂停',cancelled:'已取消',
    waiting_fpga_acceptance:'等待 FPGA 接收目标',waiting_fpga_pause:'等待 FPGA 暂停',waiting_fpga_cancel:'等待 FPGA 取消',
    fpga_command_ack_timeout:'FPGA 指令回执超时',fpga_navigation_status_unavailable:'尚无有效导航反馈',
    destination_preview:'目的地预览  未驱动小车',fpga_accepted:'FPGA 已接收请求',fpga_navigating:'FPGA 正在导航',
    fpga_arrived:'FPGA 已反馈到达',fpga_paused:'FPGA 已反馈暂停',fpga_cancelled:'FPGA 已反馈取消',
    operator_hold:'人工保持停车',waiting_for_road_observer:'等待道路观测',navigation_not_ready:'导航尚未就绪',
    waiting_for_stable_fresh_gates:'等待稳定瞄准与执行条件',not_armed:'尚未授权喷水',operator_disarm:'已撤销喷水授权',
    minimum_off_interval:'喷射冷却中',burst_budget_exhausted:'本轮喷射次数已用完',arm_lease_expired:'喷水授权已到期',
    disabled:'未启用',uncalibrated:'尚未标定',calibration_not_verified:'标定尚未验收',
    physical_eye_mapping_verified_required:'左右镜头对应待核验',synchronization_verified_required:'双目同步待核验',
    metric_depth_verified_required:'实测距离待核验',verification_id_required:'缺少标定验收记录',
    no_reliable_disparity:'双目画面暂无可靠视差',
    insufficient_reliable_depth:'该区域有效测距点不足',
    mixed_or_unstable_depth:'该区域深度混杂或波动过大',
    insufficient_inlier_depth:'该区域稳定测距点不足',
    camera_geometry_unavailable:'等待相机几何参数',
    inactive_outside_fire_mode:'当前任务尚未开启双目测距',
    expired_before_depth_publication:'测距帧已过期，等待新帧',
    mode_or_geometry_changed_during_compute:'任务或相机参数已变化，等待重新测距',
    computed:'本帧测距计算完成',detector_unavailable:'火源识别暂不可用',no_fire_target:'当前没有火焰目标',
    exact_left_frame_missing:'等待检测对应的左眼原始帧',
    waiting_for_exact_stereo_pair:'等待同一帧双目图像到齐',
    processing_budget_exhausted:'本帧计算时间不足，等待下一帧',
    roi_outside_computed_region:'目标区域未完整计算',
    computed_delayed_observation:'延迟测距观测',observation_expired:'测距观测已过期',
    matching_right_frame_missing:'等待时间匹配的右眼原始帧',
    ambiguous_right_frame:'右眼有多个候选帧，暂不采用本次测距',
    stale_detection:'检测帧已过期，等待新画面',
    invalid_bbox:'目标框无效',clipped_bbox:'目标框超出画面边界',
    roi_outside_rectified_view:'目标区域超出双目有效重叠范围',
    invalid_target_ray:'目标方向无法计算',invalid_target_pixel:'目标像素坐标无效',
    stereo_result_geometry_mismatch:'深度结果与当前双目图像几何不一致',
    task_or_pose_not_ready:'任务或相机姿态未就绪',gimbal_not_ready:'云台姿态未就绪',
    explicit_fresh_selection_required:'请先选择有效火焰',observe_and_stop_before_selecting_fire:'请先观察并确认停稳',
    fresh_calibrated_fire_lock_required:'等待有效瞄准与落点标定',fire_mode_stopped_ready_required:'等待灭火模式、停稳和云台就绪',
    no_crosswalk_event:'等待交通任务',crosswalk_stop_requested:'已请求交通停车',traffic_request_started:'已请求红绿灯识别',
    confirming_green_same_fixture:'正在确认同一信号灯的绿灯',stable_green_same_fixture:'同一信号灯绿灯已确认',
    green_permission_expired:'绿灯许可已过期',camera_frame_timeout:'相机画面超时',unknown_lamp:'灯色尚未确认',
    overexposed:'灯珠曝光过强',target_lost_stop_before_scan:'信号灯丢失，先停车再搜索',
    ready:'已就绪',listening:'正在收音',speaking:'正在播报',spoken:'播报进程已完成',
    speech_preview:'文字播报预览',loading_asr:'正在加载语音模型',
    command_submitted:'任务请求已提交',command_rejected:'任务请求未接受',tts_unavailable:'音频播放不可用',
    audio_discarded:'本次音频已丢弃',speech_queue_full:'播报队列已满',
    listening_cancelled:'本次收音已取消',capture_cancelled_during_preparation:'已在准备阶段取消，未打开麦克风',
    no_speech:'没有识别到有效人声，请检查无线麦克风是否配对、开机且未静音',
    low_asr_confidence:'人声识别把握不足，本次未下发任务，请靠近麦克风清楚重说',
    playback_shutdown_or_audio_overrun:'音频被播报打断或采集溢出，本次未下发任务',
    playback_or_shutdown_during_transcription:'转写期间被打断，本次未下发任务',
    structured_rules:'本地事实规则',domain_reference:'本地技术参考'
  });
  const autoPhaseNames = {IDLE:'待命',SEARCH:'寻找目标',AIM:'瞄准检查',BURST:'短时喷射阶段',REVIEW:'喷后复查',REPORT:'场景报告',COMPLETE:'本轮已结束',CANCELLED:'已取消'};
  const labels = {traffic_preview:'红绿灯手动观察',mission:'任务状态机',traffic_observation:'信号灯视觉',traffic:'过街状态机',navigation:'FPGA 导航任务',fpga_link:'UART 链路',fpga_telemetry:'车辆状态',fpga_navigation:'导航反馈',actuators:'云台与水泵',fire_targets:'稳定火焰目标',firefighter_targets:'稳定消防员目标',fire_selection:'选定目标',fire_selection_ack:'目标选择回执',fire_proposal:'瞄准计算',fire_auto:'自动灭火',stereo:'双目深度',stereo_observation:'厂家参数测距观测',stereo_last_valid_observation:'最近有效测距（仅显示）',scene:'场景报告',ocr:'楼牌 OCR',scene_answer:'场景回答',voice:'语音状态',transcript:'语音转写',speech:'播报请求',interaction:'任务回执',extension_result:'执行节点回执',perception_diagnostics:'视觉性能',camera_diagnostics:'相机状态',road_diagnostics:'道路算法状态',ocr_diagnostics:'OCR 状态',detections:'YOLO 检测'};
  Object.assign(reasons, {
    audio_test_subscriber_unavailable:'收声测试节点尚未就绪，请检查 X5 语音服务',
    audio_test_dispatched_observe_test_status_for_outcome:'测试请求已发出，等待麦克风实际状态',
    microphone_busy:'麦克风正在收音或转写，请等待结束后再试',
    audio_test_active:'正在进行收声测试，暂不接收正式语音指令',
    no_audio_test_active:'当前没有正在采集的测试录音',
    expired_or_future_audio_test_request:'测试请求超时，未开始录音，请重新点击',
    duplicate_audio_test_request:'重复请求未执行',
    audio_overrun:'采集数据溢出，本次测试未完成，请重试',
    playback_or_shutdown:'测试被播报或服务停止打断',
    invalid_asr_confidence:'转写未提供有效置信度，结果仅供检查'
  });
  Object.assign(reasons,{
    transcription_timeout:'转写等待超过时限，本次未提交，请重试或手动提前结束收音',
    transcription_expired:'录音已过期，本次未提交',asr_decoder_busy:'上一轮语音识别尚未退出，请稍候',
    voice_command_disagreement:'两次识别的动作或参数不一致，本次未执行；请清楚重说或核对后用文字发送',
    voice_command_verification_failed:'动作语音未通过第二次识别核对，本次未执行',
    voice_verification_timeout:'动作语音核对超时，本次未执行',
    speech_evidence_required:'没有可靠的人声片段，本次未提交',
    voice_speech_not_verified:'没有检测到可靠的人声片段，请靠近麦克风重新说话',
    unscored_asr_requires_task_commands:'当前语音模式不支持这项动作，请使用任务交互入口',
    asr_decoder_still_stopping:'上一段语音仍在结束处理，请稍候再开始',
    asr_worker_timeout:'识别计算超时，本次未提交；请重新开始收音，系统会重新加载识别进程',
    asr_worker_cancelled:'本次识别已取消，可重新开始收音',
    asr_preparation_cancelled:'语音识别准备已取消，可重新开始收音',
    asr_worker_exited:'识别进程已退出，本次未提交；重新开始收音时将恢复',
    asr_session_expired:'上一段录音已失效，未提交指令；请重新开始收音',
    asr_worker_load_failed:'本地识别模型加载失败，请检查模型和语音服务',
    dialogue_busy:'本地模型正在回答上一条问题，请等回答结束再提问',
    yielding_to_dialogue:'正在让出算力给对话，识别稍后恢复',
    scene_snapshot_unavailable:'提问时没有新鲜有效画面，请开启识别后重新提问',
    ungrounded_scene_analysis:'分析回复含未经验证的现场结论，已改用可核对事实',
    answer_deadline_exceeded:'本地模型回答超时，本次使用可核对的事实回复',
    TimeoutError:'本地模型回答超时，请稍后重试',ValueError:'模型回复未通过校验，请换一种问法',
    ungrounded_generated_claim:'模型回复包含未经证实的现场或执行结论，已拦截',
    generation_truncated_or_incomplete:'模型回复未完整生成，请简化问题后重试',
    reply_limit_without_complete_sentence:'模型回复达到长度限制，未生成完整句子',
    invalid_generation_schema:'模型回复格式不合约定',invalid_knowledge_reply_schema:'模型回复格式不合约定',
    knowledge_requires_fixed_intent_and_hint:'模型回复超出本次知识问答范围',
    expired_command:'指令已过期，未执行',manual_motion_disabled:'短程运动尚未启用，需要接入 FPGA 并完成协议验收',
    real_manual_motion_not_enabled:'FPGA 短程运动协议尚未启用',manual_motion_not_enabled:'短程运动尚未启用',
    real_manual_motion_feedback_required:'请先接入 FPGA 并完成短程运动通信验收',
    manual_requires_confirmed_stop:'短程运动需要确认车辆已停稳',
    manual_blocked_by_traffic_estop_or_auto:'当前正在处理交通、急停或自动灭火，请先结束对应任务',
    manual_requires_fresh_stopped_telemetry:'等待 FPGA 上报有效的实际停车状态',
    cancel_navigation_and_wait_stopped_before_manual:'请先结束当前导航，并等待车辆停稳',
    manual_mission_unavailable_or_unsafe:'当前任务状态不允许短程运动',
    manual_telemetry_unavailable_or_estop:'车辆反馈不可用或处于急停',
    manual_actuator_status_unavailable:'等待云台与水泵状态反馈',
    stop_and_disarm_pump_before_manual:'请先停止喷水并撤销喷水授权',
    microphone_busy:'麦克风正在收音或转写，请等待结束后再试',
    semantic_navigation_pending:'本地 Qwen 正在理解导航，尚未下发运动',
    semantic_navigation_busy:'上一条导航仍在理解中，请等待结果或先停止',
    semantic_navigation_model_busy:'本地 Qwen 正在处理上一条请求，请等待结束后再提交',
    semantic_navigation_dialogue_busy:'本地 Qwen 正在回答问题，请等待回答结束后再导航',
    dialogue_model_busy:'本地 Qwen 正在回答上一条问题，本次导航未排队，请等待回答结束后重试',
    semantic_navigation_cancelled_by_manual_navigation:'已选择手动导航，本次语音导航理解已取消',
    semantic_navigation_timeout:'本地 Qwen 理解超时，本次导航未执行',
    semantic_navigation_model_disabled:'本地导航理解模型尚未启用',
    semantic_navigation_independent_asr_required:'导航语音尚未通过独立转写核对',
    semantic_navigation_context_changed:'理解期间起点参考发生变化，请重新下达导航',
    semantic_navigation_context_heading_changed:'理解期间车头方向发生变化，请停稳后重新下达导航',
    semantic_navigation_mode_changed:'任务模式已改变，本次导航理解已取消',
    semantic_navigation_cancelled_by_command:'已收到新的控制指令，本次导航理解已取消',
    semantic_navigation_shutdown:'节点停止，本次导航理解已取消'
  });
  labels.audio_test='收声测试';
  labels.dialogue_status='本地问答状态';
  labels.traffic_observation='信号灯 · 自动控制';
  labels.traffic_preview='红绿灯 · 同帧观测';
  labels.extension='拓展任务 · FPGA 实际反馈';
  Object.assign(reasons,{real_extension_uart_required:'需要接通真实 FPGA 拓展 UART',extension_firmware_or_feedback_unavailable:'等待配套 FPGA 固件与新鲜回包',extension_firmware_unavailable:'FPGA 拓展协议尚未就绪',extension_feedback_stale:'FPGA 状态已过期',extension_movement_in_progress:'当前运动尚未结束',fpga_physical_host_mode_required:'请在 FPGA 实体屏幕选择主机运动模式',measured_stop_required:'等待 FPGA 实际停稳反馈',confirm_physical_start_before_junction_route:'请先按实际摆放位置确认人工起点',physical_start_placement_confirmation_required:'需要明确确认车辆已摆放在指定坐标',reference_outside_map:'人工起点超出当前地图范围',verify_fixed_side_camera_before_seek:'请先完成固定侧视镜头的安装与运动方向验收',fresh_yolo_exposure_required_for_seek:'寻火前需要新鲜 YOLO 相机帧',request_accepted_waiting_hardware:'请求已接收，等待 FPGA 实际执行证据',stop_requested_no_measured_stop:'已请求停车，但未收到实际停稳反馈',fpga_distance_completed:'FPGA 已报告定距运动完成',fpga_capability_not_implemented_distance:'当前固件未提供定距运动',fpga_capability_not_implemented_velocity:'当前固件未提供速度控制',fpga_capability_not_implemented_route_turn:'当前固件未提供路口路线执行'});
  Object.assign(reasons,{extension_status_unavailable:'尚无新鲜 FPGA 拓展任务反馈',extension_protocol_not_ready:'FPGA 拓展协议尚未接通',extension_motion_not_ready:'车辆运动条件尚未就绪',extension_capability_unavailable_distance:'当前固件未提供定距运动',extension_capability_unavailable_velocity:'当前固件未提供寻火速度控制',extension_capability_unavailable_route_turn:'当前固件尚未提供路口计数与转向反馈'});
  Object.assign(reasons,{
    uart_reconnecting_wait_for_fresh_feedback:'FPGA 串口正在自动恢复，等待停车确认和新鲜回包。',
    continuous_route_or_pose_identity_lost:'行驶中原路线或当前位置回读失效',
    continuous_paused_route_or_pose_changed:'暂停时原路线或当前位置回读失效',
    uart_reconnected_waiting_fresh_stop_and_feedback:'FPGA 串口已重新连接，正在确认停车状态。',
    uart_reconnected_idle_new_command_required:'FPGA 通信已恢复，请重新确认实际起点并下发新任务。',
    uart_reconnected_new_command_required:'通信恢复后需要重新下发新指令，旧指令不会自动执行。',
    extension_not_ready:'拓展运动条件尚未就绪',extension_distance_parameters_mismatch:'距离或速度与已确认指令不一致，未执行',
    real_pump_required_for_auto_after_seek:'自动寻火灭火前需要接通并启用真实水泵',
    assign_separate_fpga_uart_motor_ports_are_reserved:'请为 FPGA 分配独立串口；现有两组串口已用于云台电机',
    pump_off_fresh_actuator_feedback_required:'需要新鲜的执行器反馈，确认水泵关闭且无故障',
    navigation_execution_receipt_missing:'导航指令已解析，但未收到执行节点回执；未确认下发运动，也未自动重发',
    retired_patrol_fire_use_region_search:'旧沿途灭火指令已停用，请改为“在 A 区域搜寻火源并执行灭火”',
    region_search_requires_explicit_region_and_fire:'请明确 A～D 区域，例如“去 C 区域搜索火焰”；要自动喷水请说“搜索火焰并执行灭火”',
    region_execute_fire_must_be_boolean:'区域任务模式无效，请重新选择仅搜索或搜索并灭火',
    fire_found_waiting_operator:'已发现火焰并停稳，等待你的后续指令',
    region_search_requires_explicit_text:'区域自动灭火需要完整口令',
    region_search_requires_extension2:'请先开启拓展任务 2，再下达区域任务',
    region_search_not_ready:'区域任务尚未就绪，请检查起点、朝向与 FPGA 回包',
    real_region_search_not_enabled:'真实区域搜索执行尚未启用',
    region_search_independent_asr_required:'区域与灭火指令需要两路语音识别一致',
    text_region_mismatch:'区域或自动灭火授权与原句不一致，未下发',
    region_start_pose_and_heading_required:'请在地图上设置实际起点与车头方向，并确认生效',
    region_search_already_active:'区域搜索正在进行，请先结束当前任务',
    following_search_waypoint:'正在沿规划路线经过区域途经点，全程识别火源',
    observing_waypoint_with_yolo:'持续观察火源',
    approaching_mapped_fire:'根据火源三维位置，沿道路前往可喷水位置',
    waiting_verified_fpga_stop:'已发现火源，等待 FPGA 实际停稳',
    automatic_bounded_fire_round:'停稳后执行自动瞄准和短时喷水',
    waiting_confirmed_output_low_and_disarmed:'等待实际关水与自动喷水解除，确认后继续搜索',
    fire_detected_waiting_verified_geometry:'看见火源，等待有效三维定位与标定',
    fire_geometry_or_calibration_unavailable:'未取得有效火源三维定位或标定，当前目标不执行喷水',
    planning_complete_patrol_route_once:'正在一次规划全部途经点与出口',
    all_waypoints_passed_following_original_exit_route:'全部途经点已通过，沿原路线驶向出口并继续检测火焰',
    confirmed_fire_waiting_verified_fpga_stop:'已连续确认真火焰，等待车辆实际停稳',
    fire_observed_waiting_two_second_stop:'车辆已停稳，观察火焰 2 秒',
    confirmed_fire_dwell_complete_waiting_geometry:'停稳观察完成，正在核对三维定位与喷水距离',
    resuming_same_patrol_route:'正在恢复同一条巡检路线',
    verifying_range_after_approach:'到达靠近位置，正在重新核对目标与喷水距离',
    no_fire_found:'搜索航点已全部检查，本次未发现有效火源',
    region_search_finished:'全部航点搜索完成，已请求停车与关水',
    region_yolo_feedback_missing:'缺少 YOLO 火源帧，区域任务已停止',
    region_yolo_feedback_lost:'YOLO 火源反馈中断，区域任务已停止',
    region_navigation_feedback_missing_or_mismatched:'导航回包缺失或任务不匹配，已请求停车',
    region_pump_off_not_confirmed:'未确认水泵实际关闭，保持停车',
    region_auto_feedback_missing_or_mismatched:'自动灭火回包缺失或任务不匹配，已请求停车与关水',
    region_stop_or_fire_geometry_not_confirmed:'未同时确认停稳和火源定位，已停止任务',
    region_verified_spray_range_unavailable:'靠近后尚未确认有效喷水距离，保持停车',
    checkpoint_fire_out_of_range_on_retained_route:'本点火焰不在有效喷水距离，已记录并继续巡检',
    processed_fire_departure_cooldown:'离开已处理火焰的冷却中，继续原路线',
    semantic_region_catalog_unavailable:'区域地图目录尚未加载',
    region_or_navigation_busy:'当前区域任务或导航尚未结束',
    real_extension_uart_and_fresh_feedback_required:'等待真实 FPGA 串口与新鲜状态',
    semantic_region_fpga_firmware_required:'请使用配套支持区域航点的 FPGA 固件',
    semantic_region_route_preference_firmware_required:'需烧录新版 FPGA 区域路线固件后，才能使用动态搜索与出口停车',
    semantic_region_continuous_route_firmware_required:'需更新 FPGA 连续巡检固件，才能一次规划全部途经点并按火焰暂停续行',
    semantic_region_bidirectional_centering_firmware_required:'FPGA 尚不支持保留路线的前后微调（0x3C/0x3D）；请烧录配套新版固件',
    centering_retained_jog_firmware_required:'FPGA 缺少前后微调协议（0x3C/0x3D）；请烧录配套新版固件',
    centering_straight_corridor_exhausted:'当前位置到弯道或路线边界已无可用直线微调距离，本目标不继续前后移动',
    centering_corridor_static_obstacle:'当前前后微调直线触及地图障碍或道路边界，未允许继续微调',
    centering_corridor_live_barrier:'当前前后微调直线与已回传的路障相交，未允许继续微调',
    centering_live_barrier_invalid:'实时路障数据无效，无法确认本段微调通道',
    centering_anchor_at_route_branch:'暂停位置处于路线分叉，尚不能唯一确认沿车头前后微调的通道',
    centering_anchor_outside_retained_route_corridor:'实际暂停位置不在原路线已验证的微调范围内',
    centering_retained_straight_span_ended:'已到原路线沿当前车头可直线移动的边界，本次不再向该方向微调',
    centering_local_distance_limit:'已到本次局部微调距离上限，需要返回原暂停位置',
    centering_traversability_invalid:'微调道路栅格数据不可用，请更新配套地图数据',
    centering_traversability_map_identity_mismatch:'微调道路栅格与当前 FPGA 地图不一致，未下发移动',
    centering_traversability_run_invalid:'微调道路栅格包含无效区段，请检查配套地图数据',
    centering_traversability_overlapping_runs:'微调道路栅格区段重叠，请检查配套地图数据',
    centering_traversability_hash_mismatch:'微调道路栅格校验不一致，请更新配套地图数据',
    centering_route_graph_identity_mismatch:'微调道路图与当前地图版本不一致，未下发移动',
    centering_route_graph_invalid:'微调道路连接关系无效，无法确认可移动范围',
    centering_route_only_corridor:'当前仅按原路线确认保守的直线微调范围',
    centering_actual_line_verified_on_retained_grid:'已结合实际车头方向、原路线和道路栅格确认直线微调范围',
    fire_centering_no_local_corridor:'当前路线没有可用的直线微调通道，本目标不继续前后移动',
    centering_distance_budget_reserved_for_return:'本批微调距离已接近上限，需要保留返回原暂停位置的距离',
    fire_centering_target_lost:'该火焰未能重新确认，已跳过此目标，未计为完成',
    target_not_processed:'该目标尚未取得处理完成证据',
    fire_centering_batch_timeout:'本批火焰处理超过 180 秒，任务已停止；请查看目标状态及 FPGA 回包后重新下达任务',
    fire_centering_stop_or_pump_off_missing:'未取得持续的新鲜停稳或关水反馈，居中任务已停止；请检查 FPGA 通信和 B13 状态',
    fire_centering_camera_direction_unset:'侧向相机的前方对应关系未设置，请先确认画面右侧或左侧对应车头前方',
    fire_centering_forward_image_side_required:'请配置侧向相机中与车头前方对应的画面方向',
    fire_centering_motion_proposal_missing:'未取得有效前后微调方案，任务已停止；请检查同路线位置回包',
    fire_centering_route_already_finished:'巡检路线已结束，当前目标不再驱动车辆前后微调',
    centering_begin_requires_paused_route_and_pump_off:'等待原导航路线暂停、车辆实际停稳及水泵确认关闭',
    centering_existing_session_requires_paused_pose:'当前居中流程需要原路线的暂停状态和新鲜停车位姿',
    centering_verified_paused_pose_required:'等待同一条路线的实际停稳位置反馈，尚未允许微调',
    centering_return_requires_fresh_pose:'返回原暂停位置需要新鲜惯导位姿；请检查 FPGA 回包',
    centering_end_requires_measured_original_anchor:'尚未确认回到原暂停位置并停稳，暂不恢复巡检',
    continuous_resume_requires_centering_anchor_restored:'本批居中流程尚未确认返回原暂停位置，暂不恢复原路线',
    centering_return_lateral_or_heading_error:'当前位置的横向偏差或车头方向尚未满足原路线恢复条件',
    centering_local_distance_exceeded:'实际前后位移超出本段直线微调范围，已停止运动',
    centering_local_lateral_limit_exceeded:'实际横向位置超出原路线允许偏差，已停止微调',
    centering_local_heading_changed:'微调期间实际车头方向变化超出允许范围，已停止运动',
    centering_navigation_feedback_lost:'微调期间 FPGA 导航反馈中断，已请求停车',
    centering_same_route_pose_lost:'未取得同一路线版本与进度的实时位姿，已停止微调',
    centering_session_not_active:'本次前后微调会话已经结束',
    centering_pose_generation_changed:'微调位置回包对应的路线版本发生变化',
    centering_paused_execution_index_changed:'暂停期间 FPGA 路线执行点发生变化',
    retained_route_not_active:'当前没有有效的保留导航路线',
    retained_navigation_feedback_stale:'原路线导航反馈已过期',
    retained_navigation_session_changed:'FPGA 导航会话或任务编号发生变化',
    retained_host_permission_lost:'FPGA 上位机控制页或控制链路已退出',
    retained_navigation_motion_status_changed:'FPGA 已退出本轮导航执行状态',
    retained_pose_generation_changed:'实时位置反馈与本轮路线版本不一致',
    retained_route_generation_changed:'保留路线无效或路线版本已改变',
    retained_paused_pose_moved:'当前位置或车头方向已偏离原路线暂停参考',
    retained_stop_not_confirmed:'尚未收到原路线暂停后的实际停稳确认',
    pose_not_valid:'惯导位置反馈暂时无效',
    pose_stale:'惯导位置回包已过期',
    pose_navigation_mode_or_status_changed:'惯导回包显示导航模式或状态已变化',
    pose_task_changed:'惯导回包属于另一个导航任务',
    pose_coordinates_invalid:'惯导坐标或方向超出有效范围',
    search_pump_recovery_existing_stop_preserved:'正在确认关水，保留原停车请求；等待原任务给出结果',
    search_pump_recovery_waiting_fresh_stop_and_low:'等待新鲜停车与关水回读；运动写入已锁住',
    search_pump_recovery_preparation_motion_detected:'准备阶段收到车辆正在运动的实测回报，已停止本轮任务',
    search_pump_recovery_preparation_stop_unconfirmed:'准备阶段尚未取得新鲜停稳回报，暂不放行运动',
    centering_pump_off_lost:'微调期间无法确认水泵关闭，已请求停车',
    centering_fpga_aborted_3:'FPGA 中止微调：惯导或路线参考失效（旧固件原因码 3，无法进一步区分）',
    centering_fpga_aborted_9:'FPGA 中止微调：地图位姿尚未提交或已失效（原因码 9）',
    centering_fpga_aborted_10:'FPGA 中止微调：惯导健康检查未通过，可能为 IMU 超时、重零或编码器异常（原因码 10）',
    centering_fpga_aborted_11:'FPGA 中止微调：本轮路线版本已改变（原因码 11）',
    centering_fpga_aborted_12:'FPGA 中止微调：暂停期间路线执行点发生改变（原因码 12）',
    centering_jog_direction_or_distance_mismatch:'惯导反馈的移动方向或距离与微调指令不符，已请求停车',
    centering_jog_no_forward_progress:'FPGA 报告微调完成，但惯导沿指令方向的位移不足或反向（不超过 0.5 mm）；本轮已停止，不再重复尝试',
    centering_jog_endpoint_not_measured:'FPGA 报告微调完成，但惯导尚未确认到达对应位置，已请求停车',
    centering_jog_measured_short_stop:'本次微调已实测停稳，但位移不足；重新核对当前目标和位置后再决定下一小步',
    centering_repeated_short_stop:'同向微调多次提前停下，未达到请求位移；本轮任务已停止，请检查底盘实际运动',
    centering_target_motion_exhausted:'当前火焰多次微调后仍未对中，已记录为未完成；继续其他目标，回锚确认后恢复原路线',
    centering_return_window_unreachable_on_verified_line:'当前方向无法在已验证直线内回到原暂停位置，原路线尚未恢复',
    centering_return_intermediate_step_limited:'本次回程小步受到直线边界限制，尚未确认能够返回原暂停位置',
    centering_jog_timeout:'本次前后微调超时，已请求停车；请检查底盘运动与串口反馈',
    centering_jog_ack_timeout:'FPGA 未确认接收本次微调指令，已请求停车；请检查串口连接',
    centering_stop_ack_timeout:'FPGA 未确认本次微调停车请求，请检查串口与实车停稳状态',
    centering_stop_rejected:'FPGA 拒绝本次微调停车请求，请检查固件及实车状态',
    centering_proposal_changed_or_invalid:'位置或路线在提交微调前发生变化，本次移动未获确认',
    centering_uart_sequences_exhausted:'当前串口请求尚未释放，请等待回包；未重复下发微调',
    centering_direction_requires_minus1_or1:'微调方向参数无效，未下发运动',
    centering_step_or_speed_out_of_range:'微调距离或速度超出协议范围，未下发运动',
    centering_signed_distance_required:'微调需要明确的正向或反向距离，未下发运动',
    centering_session_not_active:'当前没有正在执行的居中流程',
    centering_jog_not_active:'当前没有正在执行的前后微调',
    centering_anchor_captured:'已记录本批火焰观察的原暂停位置',
    centering_jog_waiting_acceptance:'前后微调指令已提交，等待 FPGA 确认接收',
    centering_retained_signed_jog_proposed:'已生成保留原路线的有限前后微调方案',
    centering_waiting_signed_jog_stop:'已请求结束本次微调，等待实际停稳回包',
    centering_jog_measured_stop:'本次微调已取得实际停稳与位置回包',
    centering_original_anchor_restored:'已确认返回原暂停位置，可继续保留的巡检路线',
    returning_to_anchor_to_reacquire_queued_flame:'正在返回原观察位置，重找暂时离开画面的队列火焰',
    collecting_post_stop_confirmations:'已停车，正在用新的连续画面确认候选；停车不表示已居中完成',
    waiting_post_stop_stable_flame_evidence:'等待停车后的同一火焰连续多帧，单帧高置信度还不能开始对齐',
    no_flame_visible_after_verified_stop:'停稳后未重新确认火焰，结束本批观察并恢复原路线',
    flame_batch_confirmation_timeout:'本批候选未在时限内稳定确认，跳过后恢复原路线',
    stable_flames_confirmed_after_stop:'停车后的连续多帧已确认本批火焰，开始逐个对齐',
    new_flame_confirmation_timeout:'新候选未在时限内稳定确认，本批不对它继续移动',
    waiting_new_batch_candidate_confirmation:'发现本批新候选，等待连续多帧确认后再对齐',
    rechecking_queued_flame_at_original_view:'已回原观察位置，等待新鲜多帧重新确认队列火焰',
    centered_stationary_one_second_required:'当前火焰已居中且车辆停稳，正在累计连续 1 秒观察',
    flame_centered_and_stopped_for_one_second:'本目标已完成停稳居中 1 秒观察，继续处理后续目标',
    centered_one_second_waiting_fire_geometry:'本目标已完成停稳居中 1 秒，正在核对三维位置与喷水条件',
    all_flames_processed_and_route_anchor_restored:'本批目标已处理并返回原暂停位置，继续原路线',
    collecting_all_visible_stable_flames:'正在用连续新鲜帧确认画面两侧的稳定火焰',
    all_visible_flames_queued_both_sides:'画面两侧的稳定火焰已加入本批队列',
    selecting_next_visible_flame:'正在选择本批下一团待处理火焰',
    moving_selected_flame_towards_centerline:'正在前后微调，让当前火焰进入垂直中线',
    local_move_stopped_rechecking_visual_center:'本次微调已停稳，正在重新核对当前火焰位置',
    waiting_measured_stop_after_visual_center:'火焰已进入居中带，等待车辆实际停稳',
    centerline_changed_restart_one_second_confirmation:'火焰离开居中带，重新对齐并累计连续 1 秒',
    centerline_requires_fresh_stationary_confirmation:'等待新鲜的目标居中与车辆停稳证据，再开始计时',
    centerline_changed_before_fire_round:'喷水前火焰位置发生变化，重新核对居中条件',
    returning_to_saved_route_anchor:'正在返回本批火焰开始前的原暂停位置',
    terminal_flame_batch_finished:'终点附近本批目标已处理，等待最终出口停稳确认',
    continuous_route_download_timeout:'完整巡检路线回读超时，车辆尚未启动；请检查 FPGA 串口连接',
    region_road_exit_unavailable:'当前区域没有可用道路出口，请检查地图道路与区域范围',
    region_waypoints_or_exit_unreachable_from_actual_heading:'按当前实际朝向无法连通全部航点及出口，请核对起点位置与车头方向',
    road_turns_require_outside_detour:'道路转向约束需要短暂绕出区域',
    region_exit_pose_still_inside:'已收到到达，但惯导位置仍在区域内，暂不确认驶出完成',
    region_search_exited_and_stopped:'全部航点搜索完毕，已确认驶出区域并停稳',
    region_exit_navigation_timeout:'前往出口超时，已请求停车',
    region_start_heading_has_no_directed_road_entry:'当前车头方向没有合法道路接入点，请核对实车朝向与地图起点',
    region_directed_road_graph_identity_invalid:'道路图与当前地图不匹配，请更新配套地图文件',
    region_directed_road_graph_all_blocked:'当前道路均被障碍标记阻断，无法安排区域搜索',
    semantic_region_map_identity_mismatch:'X5 区域地图与 FPGA 地图不一致',
    region_requires_extension2:'请先开启拓展任务 2',
    select_fpga_communication_navigation_page:'请在 FPGA 屏幕选择通信导航页',
    region_pump_off_and_disarmed_required:'等待确认关水及自动喷水解除',
    region_real_gimbal_reference_required:'双轴实际角度和安装参考尚未就绪',
    region_priority_state_not_ready:'当前有急停、交通任务或任务状态未更新，请先处理后重试',
    region_stop_or_pump_off_not_confirmed:'等待 FPGA 确认停稳与关水',
    gimbal_manual_subscriber_unavailable:'正式云台控制节点尚未连接',gimbal_manual_status_stale:'等待新鲜的云台控制状态',
    gimbal_manual_not_ready:'云台角度运动条件尚未就绪',gimbal_manual_actuator_feedback_required:'等待真实双轴角度回读',
    gimbal_reference_raw_feedback_required:'等待两台电机真实串口回读',
    gimbal_reference_stationary_readback_required:'等待两轴停止并取得稳定编码器读数',
    gimbal_reference_operator_confirmation_required:'请确认喷嘴处于机械零位、车辆与两轴均静止且俯仰有支撑',
    gimbal_reference_not_ready:'本次零位记录条件尚未就绪',
    gimbal_reference_dispatched_waiting_reference_feedback:'零位记录请求已下发，等待车载确认；未下发运动',
    gimbal_reference_capture_running:'正在读取并记录本次机械零位',
    gimbal_reference_already_confirmed:'本次机械零位已记录，无需重复确认',
    reference_already_confirmed:'本次机械零位已记录，无需重复确认',
    gimbal_reference_recorded_no_motion:'本次机械零位已记录，未转动电机',
    waiting_for_three_stationary_readbacks:'正在确认连续三次静止读数',
    raw_encoder_feedback_stale:'串口原始读数已过期，等待恢复',
    ready_for_operator_zero_confirmation:'等待确认喷嘴当前机械零位',
    installed_limits_gearing_or_supported_load_unconfirmed:'安装行程、传动比或俯仰支撑尚未确认',
    tilt_physical_direction_unconfirmed_or_inconsistent:'俯仰方向记录尚未确认或与当前配置不一致',
    pd42_retained_target_differs_no_automatic_rebase:'俯仰实际位置与驱动器保留目标不一致，请核对姿态',
    zdt_not_stationary_or_retained_targets_differ:'水平轴尚未稳定停下或保留目标与实际位置不一致',
    both_axes_must_be_measured_stationary:'等待双轴实际停止并取得编码器读数',
    end_automatic_task_before_reference:'请先结束自动任务，再记录机械零位',
    another_gimbal_owner_active:'云台正在执行其他动作，请等待结束',
    manual_vehicle_known_moving:'已知车辆正在运动，请停车后再调整云台或记录零位',
    gimbal_manual_axis_limits_unavailable:'车载角度范围尚未确认',gimbal_manual_motion_axes_unavailable:'车载轴运动权限尚未确认',
    gimbal_manual_pan_not_verified:'水平轴方向尚未验收',gimbal_manual_tilt_not_verified:'俯仰轴方向尚未验收',
    gimbal_manual_pan_outside_limits:'水平目标超出车载角度范围',gimbal_manual_tilt_outside_limits:'俯仰目标超出车载角度范围',
    gimbal_manual_dispatched_waiting_encoder_feedback:'已提交角度请求，等待实际角度反馈',manual_gimbal_running:'云台正在执行角度运动',
    manual_gimbal_already_running:'请等待当前角度运动结束',manual_gimbal_arrival_timeout:'到位超时，已停止角度运动',
    waiting_measured_arrival:'正在等待电机实测到位',measured_arrival:'已确认电机实测到位',not_requested:'尚未提交目标',
    pan_angle_outside_measured_limits:'水平目标超出实测范围',tilt_angle_outside_measured_limits:'俯仰目标超出实测范围',
    pan_direction_not_verified:'水平方向尚未验收',tilt_direction_not_verified:'俯仰方向尚未验收',
    region_pump_b13_not_ready:'FPGA B13 水泵控制尚未就绪',
    region_pump_b13_firmware_required:'FPGA 尚未回报 B13 水泵控制能力，请核对固件和串口连接',
    region_pump_output_disabled:'X5 自动喷水输出配置未启用；FPGA B13 通信正常也不会自动喷水',
    region_pump_feedback_stale:'等待新鲜的 FPGA B13 开关水反馈',
    region_pump_vehicle_moving:'车辆尚未停稳，FPGA 暂不允许开启水泵',
    region_pump_scope_required:'请在 FPGA 进入拓展2控制页，等待 B13 控制范围确认',
    region_pump_link_required:'FPGA 水泵控制链路未就绪，等待重新连接',
    region_pump_off_reset_required:'等待 FPGA 确认关水并解除水泵锁定',
    region_mount_or_priority_state_not_ready:'固定相机安装或任务优先级条件尚未就绪',
    region_auto_controller_not_idle:'上一轮自动灭火尚未结束',
    region_live_yolo_required:'等待新鲜 YOLO 火源数据',
    search_only_visual_unavailable_skip_observation:'视觉暂缺，取消本次对中并继续原路线',
    visual_unavailable_waiting_verified_local_stop:'视觉暂缺，等待本次微动停稳后恢复路线',
    visual_unavailable_returning_original_route_anchor:'视觉暂缺，正在回到原路线暂停位置',
    fire_centering_fresh_exposure_unavailable:'未收到新的有效对中画面',
    fire_stop_waiting_navigation_running:'等待车辆开始本段巡检后再确认停车目标',
    fire_stop_waiting_fresh_exposure:'等待新鲜画面中的稳定火焰',
    fire_stop_waiting_post_start_exposure:'等待本段启动后的新画面，不采用停车前的旧目标',
    fire_stop_waiting_suitable_position:'继续观察，等待火焰完整进入画面合适位置',
    fire_stop_confirming_stable_target:'正在确认同一火焰持续稳定，尚未触发停车',
    fire_stop_stable_target_ready:'火焰位置与稳定性已满足停车条件',
    search_pump_recovery_waiting_fresh_low:'正在核对关水回读，保留当前任务与原路线',
    search_pump_recovery_same_route_resumed:'已确认关水，继续原路线',
    search_pump_recovery_low_confirmed:'关水回读已恢复，继续当前任务',
    region_approach_calibration_required:'请先完成双目、相机到车身地图及喷嘴落点标定',
    region_approach_calibration_verified_required:'区域灭火定位标定尚未完成：需要双目三维坐标与车身、地图坐标对应，已填写的云台安装尺寸不能代替这项标定',
    region_approach_map_registration_verified_required:'火焰三维位置与 FPGA 地图的坐标对应尚未验证',
    region_approach_spray_range_verified_required:'尚未实测确认水枪的有效喷水距离范围',
    region_approach_body_envelope_verified_required:'小车车身尺寸与可通行空间尚未对应验证',
    region_approach_time_sync_verified_required:'相机图像与小车位姿的时间对应尚未验证',
    region_approach_camera_calibration_identity_required:'缺少与当前图像对应的双目标定编号或光学坐标系',
    region_approach_reference_frame_invalid:'区域定位使用的车身基准或左眼光学坐标系不一致',
    region_approach_measured_mount_translation_required:'缺少相机及喷嘴相对车身基准的实测位置',
    region_approach_measured_spray_range_required:'有效喷水距离尚未填写或超出支持范围',
    region_approach_measured_target_height_required:'火焰目标的有效高度范围尚未填写',
    region_approach_measured_body_clearance_required:'小车通行净距尚未填写或无效',
    mark_start_and_heading_on_fpga_or_hmi:'请先在 FPGA 屏幕或本页地图设置实际起点与车头方向',
    region_start_requires_measured_stop:'请等待 FPGA 实际停稳，再确认地图起点与方向',
    region_start_reference_query_required:'等待 FPGA 起点确认序号回读',
    region_start_not_in_clear_map_corridor:'起点不在已验证的可通行道路内，请按实际摆车位置重新确认',
    disarm_pump_before_vehicle_motion:'请先停止喷水并撤销喷水授权，再移动车辆',
    map_catalog_unavailable:'当前未加载与 FPGA 配套的地图配置',unsupported_semantic_map:'当前语义地图版本不兼容',
    initialized_nose_pose_required:'请先确认车头中点的地图坐标和朝向',start_outside_map:'起点超出地图边界',
    start_not_on_unique_straight_road:'起点或车头方向不在唯一的直线路段上，请核对摆放',
    start_too_close_to_junction_or_curve:'起点过于靠近路口或弯道，请留出阿克曼转弯准备距离',
    requested_turn_not_present_at_junction:'指定路口没有该方向的道路，请选择实际存在的分支',
    insufficient_straight_exit_for_ackermann_turn:'转弯出口的直线距离不足，无法完成阿克曼转弯',
    straight_road_ends_before_requested_junction:'直行道路在指定路口之前已结束，无法按该指令行驶',
    too_many_waypoint_constraints:'路线经过的约束点过多，请分成较短任务',
    route_constraint_outside_map:'路线约束点超出地图，未下发',junction_route_cycle:'路线重复绕行，未下发',
    invalid_junction_instruction:'路口序号或转向无效',junction_count_requires_1_to_8:'路口序号须为 1～8',
    distance_requires_5_to_1000_cm:'移动距离须为 5～1000 厘米',extension_distance_requires_5_to_1000_cm:'移动距离须为 5～1000 厘米',
    target_passed_center_stop_reposition_required:'火焰已越过中心，已请求停车；请重新摆放或改用云台瞄准',
    selected_fire_track_lost:'选定火焰跟踪丢失，已请求停车',fire_target_lost:'选定火焰跟踪丢失，已请求停车',
    fire_horizontal_center_confirmed:'已确认同一火焰水平居中并停车；尚不代表喷嘴命中',
    fpga_feedback_lost:'FPGA 状态回传中断，已请求停车',yolo_camera_exposure_lost:'相机或 YOLO 新帧中断，已请求停车',
    extension_deadline_expired:'任务超时，已请求停车',fpga_ack_timeout:'FPGA 未按时确认指令，已请求停车',
    fpga_mode_stop_not_confirmed:'切换任务后未确认 FPGA 停稳，未开始新运动',fpga_motion_fault:'FPGA 上报车辆运动故障，已请求停车',
    select_traffic_task_and_wait_ready_before_drive:'先开启交通识别，并等待 FPGA 确认交通模式与停稳后，再开始行驶',
    fpga_task_mode_transition_timeout:'FPGA 任务模式切换超时，未开始行驶',
    fpga_capability_not_implemented_traffic:'当前固件未提供独立交通行驶',extension_capability_unavailable_traffic:'当前固件未提供独立交通行驶',
    fpga_route_upload_timeout:'路线上传回执超时，已请求停车',fpga_route_planning_failed:'FPGA 路径规划失败，未开始行驶',
    fpga_route_planning_timeout:'FPGA 路径规划超时，未开始行驶',operator_stop:'已请求停止当前任务',
    pump_output_pending_or_not_confirmed_off:'水泵尚未确认关闭，导航已停止；等待关水回读后重新提交',
    observed_vehicle_movement:'车辆位置已变化，请重新确认起点与方向',
    vision_task_selected:'已切换到拓展一 · 观察与问答',dialogue_task_selected:'已切换到拓展二 · 语义导航与找火',traffic_task_selected:'已切换到独立交通任务',
    enable_fpga_zebra_detection_before_traffic_drive:'请先开启 FPGA 斑马线检测，并等待实际回读为开',
    fpga_capability_not_implemented_zebra_switch:'当前固件未提供斑马线检测开关',
    zebra_switch_pending:'斑马线设置正在确认，等待 FPGA 实际回读',
    zebra_switch_not_ready:'等待串口、固件能力与实际停稳反馈',
    measured_stop_required_for_zebra_switch:'切换斑马线检测前需确认车辆已停稳',
    extension_transition_in_progress:'任务模式或斑马线设置正在确认，请稍候',
    stop_current_movement_before_zebra_switch:'先结束当前运动并停稳，再修改斑马线检测',
    fpga_zebra_switch_readback_timeout:'未按时收到斑马线实际开关状态，设置尚未确认',
    select_observation_task_for_voice_and_dialogue:'请开启拓展一或拓展二后对话；交通任务期间暂停问答',
    select_extension2_for_numbered_navigation:'请先开启拓展二，再下达编号点导航',
    manual_spray_command_required_for_extension_tasks:'当前流程由你选定火焰，再明确说“开始喷水”；到达和识别不会自动喷水',
    point_reference_required:'请先确认实际起点；语音中的点号不会建立定位',
    numbered_start_heading_required:'请补充实际车头朝向，例如：我现在在8号点，车头朝左，请前往4号点。',
    numbered_heading_firmware_required:'FPGA需要更新支持车头朝向的导航固件',
    numbered_heading_requires_minus180_to180:'车头朝向应为-180到180度：0向右，90向上，180向左，-90向下',
    start_heading_requires_minus180_to180:'请明确车头朝向，角度范围-180到180度',
    numbered_navigation_needs_clear_points:'请说明明确的起点、终点和实际车头朝向，例如：我在8号点，朝左，前往4号点。',
    numbered_navigation_separate_actions:'请把导航和喷水等操作分开下达。',
    numbered_navigation_independent_asr_required:'导航语音尚未通过点号与方向核对，请核对转写或使用文字输入。',
    fpga_navigation_motion_failed_6:'起步车头朝向与规划首段不符，请核对实际方向后重新规划。',
    fpga_heading_ack_mismatch:'FPGA返回的车头朝向与请求不一致，未启动导航',
    uploading_actual_start_heading:'正在设置实际车头朝向',
    numbered_navigation_not_ready:'编号点导航尚未就绪，请等待真实 FPGA 反馈',
    confirm_physical_numbered_start:'请先按实际摆放位置确认编号起点',
    select_fpga_communication_navigation_page:'请从 FPGA 设置页返回首页，点击底部“拓展2对话控制”进入控制页；随后语音下达起点和目标',
    numbered_firmware_status_unavailable:'尚未收到新版 FPGA 编号点状态',
    numbered_navigation_busy:'上一项编号点任务尚未结束',
    spoken_start_does_not_match_confirmed_location:'说出的起点与已确认位置不符，请核对实际摆放',
    already_at_confirmed_point_no_navigation_started:'已确认位于该点，未启动重复导航',
    numbered_navigation_feedback_lost:'编号点导航反馈中断，已请求停车',
    fpga_selected_destination_changed:'FPGA 目标点发生变化，本次任务关联已失效',
    numbered_done_event_missing:'尚未收到与本任务匹配的到达事件',
    matching_done_and_measured_stop:'匹配的到达事件与实际停稳已确认',
    point_start_mismatch:'请求中的起点与已确认位置不一致，请检查实际摆放',
    point_navigation_protocol_unavailable:'当前 FPGA 未提供编号点导航反馈',
    fpga_zebra_state_verified:'FPGA 已实际回读并确认斑马线检测状态',
    motion_identity_exhausted_restart_while_stopped:'本次运动指令编号已用完；停稳后重启任务服务',
    duplicate_extension_request:'重复拓展指令未再次执行',expired_extension_request:'拓展指令已过期，未执行',
    unsupported_extension_action:'当前拓展任务不支持此动作',partial_uart_write:'串口帧未完整发送，通信已停止'
  });
  const reason = v => {
    if(reasons[v])return reasons[v];
    const raw=String(v||'');
    if(raw.startsWith('region_confirmed_fire_unprocessed:'))return '本批已确认火焰尚未处置，保持停车：'+reason(raw.slice('region_confirmed_fire_unprocessed:'.length));
    for(const [prefix,label] of [['depth_rejected:','双目测距未采用：'],['raw_frame_rejected:','双目原始帧未采用：'],['region_replan_failed:','重新规划失败：'],['region_leg_rejected:','区域航段未启动：'],['fire_centering_rejected:','火焰居中请求未执行：'],['fire_centering_anchor_return_unavailable:','无法恢复原暂停位置，任务已停止：']])if(raw.startsWith(prefix))return label+reason(raw.slice(prefix.length));
    const prefixNames={centering_fpga_aborted_:'FPGA 已中止本次前后微调，返回原因码 ',centering_fpga_jog_rejected_:'FPGA 拒绝本次前后微调，返回码 ',fpga_task_mode_rejected_:'FPGA 拒绝交通任务模式，返回码 ',fpga_mode_rejected_:'FPGA 拒绝任务模式，返回码 ',route_upload_rejected_:'FPGA 拒绝路线约束，返回码 ',fpga_command_rejected_:'FPGA 拒绝运动指令，返回码 ',fpga_fault_:'FPGA 上报运动故障，错误码 ',uart_open_failed: 'FPGA 串口打开失败：'};
    for(const [prefix,label] of Object.entries(prefixNames))if(raw.startsWith(prefix))return label+raw.slice(prefix.length).replace(/^:/,'');
    return v||'等待反馈';
  };
  const state = {data:null,received:0,connected:false,pending:false,view:'overview',stream:true,frameToken:'',frame:null,bitmap:null,frameAt:0,frameAge:null,frameUpdateAge:null,frameStale:3,frameStatus:'waiting',frameGeneration:null,frameEpoch:0,frameController:null,frameTimer:null,frameBusy:false,frameTimes:[],previewPerformance:{},pendingAction:null,connectionError:'',buildingsKey:'',localMessages:[],toastTimer:null};
  const canvas = $('visionCanvas'), context = canvas.getContext('2d'), decoder = new TextDecoder('utf-8',{fatal:true});
  const roadView={mode:'detection',frame:null,bitmap:null,at:0,age:null,status:'waiting',error:'',timer:null,controller:null,busy:false,epoch:0,frameTimes:[]};
  const wantsYoloFrames=()=>['overview','fire'].includes(state.view)||(state.view==='road'&&roadView.mode==='yolo');
  const wantsRoadFrames=()=>connected()&&(!fresh('extension')||data('extension').task==='traffic')&&state.view==='road'&&roadView.mode==='detection'&&state.stream&&!document.hidden;
  // Display-only inspection: retain the received image and same-frame boxes.
  function visionDisplayGeometry(mode, sourceWidth, sourceHeight, viewportWidth, viewportHeight, pixelRatio=1) {
    const ratio=Number.isFinite(pixelRatio)&&pixelRatio>0?pixelRatio:1;
    const width=Math.max(1,sourceWidth),height=Math.max(1,sourceHeight);
    const scale=mode==='native'?1/ratio:mode==='detail'?2/ratio:Math.min(viewportWidth/width,viewportHeight/height);
    return {width:width*scale,height:height*scale,scale};
  }
  function setupVisionInspection() {
    const panel=$('sharedVisionPanel'),viewport=$('cameraViewport'),plane=$('cameraImagePlane');
    const select=$('visionScale'),expand=$('visionExpand');
    const scalePreference='rescue.visionDisplayScale';
    let mode='native',lastGeometry=null,drag=null,pendingCenter=true,layoutFrame=null;
    try{const saved=localStorage.getItem(scalePreference);if(['fit','native','detail'].includes(saved))mode=saved;}catch{}
    select.value=mode;viewport.dataset.zoomed=String(mode!=='fit');
    const update=()=>{
      layoutFrame=null;
      const w=viewport.clientWidth,h=viewport.clientHeight;if(w<=0||h<=0)return;
      const ratio=window.devicePixelRatio||1;
      const current=visionDisplayGeometry(mode,canvas.width,canvas.height,w,h,ratio);
      // Preserve the viewed source position across live frames and window changes.
      const old=lastGeometry;
      if(!pendingCenter&&old&&old.mode===mode&&old.pixelRatio===ratio&&
          old.viewportWidth===w&&old.viewportHeight===h&&old.width===current.width&&old.height===current.height)return;
      const cx=old?(viewport.scrollLeft+old.viewportWidth/2)/Math.max(old.width,old.viewportWidth):.5;
      const cy=old?(viewport.scrollTop+old.viewportHeight/2)/Math.max(old.height,old.viewportHeight):.5;
      canvas.style.width=current.width+'px';canvas.style.height=current.height+'px';
      plane.style.width=Math.max(w,current.width)+'px';plane.style.height=Math.max(h,current.height)+'px';
      viewport.dataset.zoomed=String(mode!=='fit');
      viewport.scrollLeft=(pendingCenter ? .5 : cx)*Math.max(w,current.width)-w/2;
      viewport.scrollTop=(pendingCenter ? .5 : cy)*Math.max(h,current.height)-h/2;
      pendingCenter=false;lastGeometry={...current,mode,pixelRatio:ratio,viewportWidth:w,viewportHeight:h};
      text('visionScaleHint',mode==='fit'?'全幅缩放 · 识别全幅':mode==='native'?'1:1 查看 · 识别全幅':'2× 查看 · 识别全幅');
      $('visionScaleHint').title='这里只改变显示比例；YOLO始终识别完整单眼画面。1:1和2×模式可拖动查看边缘。';
    };
    // Do not mutate scrollable geometry inside ResizeObserver delivery: a
    // scrollbar change can resize its own viewport and repeatedly rescale fit.
    const scheduleLayout=()=>{if(layoutFrame===null)layoutFrame=requestAnimationFrame(update);};
    select.addEventListener('change',()=>{mode=select.value;pendingCenter=true;viewport.dataset.zoomed=String(mode!=='fit');
      try{localStorage.setItem(scalePreference,mode);}catch{}scheduleLayout();});
    const expanded=()=>document.fullscreenElement===panel||panel.classList.contains('vision-focus');
    const updateExpanded=()=>{expand.textContent=expanded()?'退出全屏':'全屏查看';expand.setAttribute('aria-pressed',String(expanded()));scheduleLayout();};
    expand.addEventListener('click',async()=>{
      if(document.fullscreenElement===panel)await document.exitFullscreen();
      else if(panel.classList.contains('vision-focus'))panel.classList.remove('vision-focus');
      else{
        try{if(!panel.requestFullscreen)throw new Error('Fullscreen unavailable');await panel.requestFullscreen();}
        catch{panel.classList.add('vision-focus');}
      }
      updateExpanded();
    });
    document.addEventListener('fullscreenchange',updateExpanded);
    document.addEventListener('keydown',event=>{
      if(event.key==='Escape'&&panel.classList.contains('vision-focus')){panel.classList.remove('vision-focus');updateExpanded();}
    });
    viewport.addEventListener('pointerdown',event=>{
      if(mode==='fit'||event.button!==0)return;
      drag={id:event.pointerId,x:event.clientX,y:event.clientY,left:viewport.scrollLeft,top:viewport.scrollTop};
      viewport.setPointerCapture(event.pointerId);viewport.classList.add('dragging');event.preventDefault();
    });
    viewport.addEventListener('pointermove',event=>{
      if(!drag||event.pointerId!==drag.id)return;
      viewport.scrollLeft=drag.left+drag.x-event.clientX;viewport.scrollTop=drag.top+drag.y-event.clientY;
    });
    const endDrag=()=>{drag=null;viewport.classList.remove('dragging');};
    viewport.addEventListener('pointerup',endDrag);viewport.addEventListener('pointercancel',endDrag);
    viewport.addEventListener('lostpointercapture',endDrag);
    new ResizeObserver(scheduleLayout).observe(viewport);
    new MutationObserver(scheduleLayout).observe(canvas,{attributes:true,attributeFilter:['width','height']});
    window.addEventListener('resize',scheduleLayout);scheduleLayout();
  }
  setupVisionInspection();
  const topic = key => state.data?.topics?.[key];
  function fresh(key) { const t=topic(key); return Boolean(connected() && t?.synthetic!==true && t?.data?.synthetic!==true && t?.fresh===true && finite(t.age_s) && t.age_s+(performance.now()-state.received)/1000<= (finite(t.stale_after_s)?t.stale_after_s:3)); }
  const data = key => fresh(key) && topic(key)?.data && typeof topic(key).data === 'object' ? topic(key).data : {};
  const seen = key => topic(key)?.synthetic===true||topic(key)?.data?.synthetic===true?null:topic(key)?.data;
  const liveSource = () => state.data?.mode==='live' && state.data?.synthetic===false;
  const connected = () => state.connected && liveSource() && performance.now()-state.received<3000;
  const liveEvents = () => (state.data?.events||[]).filter(e=>e.synthetic!==true&&e.data?.synthetic!==true);
  const canControl = () => Boolean(connected() && state.data?.control?.command_ready && !state.pending);
  const navigationReady = () => fresh('fpga_link') && data('fpga_link').dry_run===false && data('fpga_link').status_fresh===true && !data('fpga_link').error;
  const pumpReady = () => fresh('actuators') && data('actuators').dry_run===false && data('actuators').enabled===true && data('actuators').pump_enabled===true && (data('actuators').pump_backend!=='fpga_uart'||data('actuators').pump_ready===true) && !data('actuators').error && !data('actuators').estop;
  function gimbalReadbackReason(value) {
    const detail=typeof value==='string'?value:'';
    if(/ZDT.*(?:0x1F|timeout|timed out)/i.test(detail))return '水平 ZDT 电机串口无反馈，尚未取得本次角度；请检查水平电机供电与 PIN 3/5 串口连接，再点击恢复连接。';
    if(/PD42.*(?:timeout|timed out)/i.test(detail))return '俯仰 PD42 电机串口无反馈，尚未取得本次角度；请检查俯仰电机供电与 PIN 8/10 串口连接，再点击恢复连接。';
    return detail?reason(detail):'双轴编码器反馈未就绪';
  }
  function aimingBlockReason() {
    const m=data('mission'),a=data('actuators'),problems=[];
    if(!fresh('mission'))problems.push('任务状态未更新');
    else if(m.vehicle_stopped!==true)problems.push('尚无实际停稳反馈');
    if(!fresh('actuators'))problems.push('执行器状态未更新');
    else {
      if(a.error)problems.push('执行器：'+reason(a.error));
      if(a.estop)problems.push('执行器处于急停');
      if(a.dry_run!==false)problems.push('云台仍为 dry-run');
      if(a.enabled!==true)problems.push('真实云台输出未启用');
      if(a.feedback_required===true&&(a.status_fresh!==true||a.feedback_fresh!==true))problems.push(gimbalReadbackReason(a.readback_error));
    }
    return problems.join('；');
  }
  const aimingReady = () => !aimingBlockReason();
  function targetSelectionReason() {
    const m=data('mission');
    if(!fresh('fire_targets'))return '火焰目标已过期，等待新画面';
    if(!fresh('mission')||!['inspect','fire'].includes(m.mode))return '请先开启火场识别';
    if(m.vehicle_stopped!==true)return '请先停车，等待实际停稳反馈';
    if(m.emergency_stop||m.traffic_active||data('fire_auto').active===true)return '请先结束当前优先任务';
    return '';
  }
  function selectedRoundExpectation() {
    const s=data('fire_selection');
    if(s.valid!==true||!Number.isInteger(s.target_id)||!Number.isInteger(s.selection_generation)||typeof s.registry_session!=='string')return null;
    return {target_id:s.target_id,registry_session:s.registry_session,selection_generation:s.selection_generation};
  }
  function selectedRoundReason() {
    if(!selectedRoundExpectation())return '先点击一个稳定火焰编号';
    if(data('fire_auto').active===true)return '当前自动灭火尚未结束';
    if(fresh('extension')&&regionSearchState().active===true)return '区域任务正在运行，请先停止区域任务';
    const block=aimingBlockReason();if(block)return block;
    if(!pumpReady())return '真实水泵输出尚未启用或未收到就绪反馈';
    const p=data('fire_proposal');
    if(p.calibration_ready===false)return '请先完成固定相机与喷嘴标定';
    if(p.valid!==true)return reason(p.reason||'等待该目标的距离与瞄准计算');
    return '';
  }
  function manualPumpReason(value) {
    const linkReasons=['manual_pump_vehicle_not_stopped','manual_pump_fpga_idle_required',
      'manual_pump_fpga_reconnecting','manual_pump_fpga_feedback_unavailable','manual_pump_fpga_not_ready',
      'manual_pump_fresh_off_required','manual_pump_physical_control_page_required','manual_pump_fpga_link_required'];
    if(linkReasons.includes(value)){
      const e=data('extension'),p=data('pump_manual'),error=p.fpga_error||e.error||'';
      const hostClosed=(p.fpga_feedback_fresh===true&&p.fpga_physical_host_ready===false)||
        (fresh('extension')&&e.status_fresh===true&&e.fpga_host?.fresh===true&&
          (e.fpga_host.selected===false||e.fpga_host.scope===false));
      if(hostClosed)return 'FPGA 未进入“拓展2对话控制”页，B13 未收到试喷开启命令；请在 FPGA 首页进入该控制页，不用启动行驶';
      const missing=p.fpga_feedback_fresh===false||(fresh('extension')&&e.status_fresh===false);
      if(missing)return (p.fpga_recovering===true||e.uart_recovering===true?
        'FPGA 串口无反馈，正在重连；当前无法控制 B13':'FPGA 串口无反馈，无法控制 B13')+
        (error?'。'+reason(error):'。请检查 FPGA 供电和串口连接。');
    }
    if(value!=='manual_pump_fault_or_estop')return reason(value);
    const p=data('pump_manual'),a=data('actuators'),pumpFresh=fresh('pump_manual'),actuatorFresh=fresh('actuators');
    if(!pumpFresh&&!actuatorFresh)return '执行器故障状态待更新；当前故障原因与急停状态尚不能确认';
    if(p.estop===true||a.estop===true)return '执行器处于急停，不能开始新的试喷；关水仍可提交';
    const estopKnown=typeof p.estop==='boolean'||typeof a.estop==='boolean';
    const estopDetail=estopKnown?'':' 急停状态待更新，当前不能确认是否已解除。';
    // The pump node aggregates current faults and retires recovered history.
    // An explicit empty aggregate must not revive an older actuator diagnostic.
    const hasCurrentAggregate=pumpFresh&&typeof p.actuator_error==='string';
    const detail=[...new Set((hasCurrentAggregate?[p.actuator_error]:[a.error,a.readback_error])
      .filter(x=>typeof x==='string'&&x.trim()).map(x=>x.trim()))].join('；');
    if(/FD8.*(?:overspeed|following error)/i.test(detail)){
      const d=a.pan_feedback_diagnostic?.last_fault_detail;
      const measured=d?.kind==='overspeed'&&finite(d.value)&&finite(d.limit)?
        '曾测得转速 '+num(d.value,1)+' RPM（限制 '+num(d.limit,1)+'）':
        d?.kind==='following_error'&&finite(d.value)&&finite(d.limit)?
        '曾测得跟随误差 '+num(d.value,2)+'°（限制 '+num(d.limit,2)+'°）':'曾报告速度或跟随误差';
      return '水平电机'+measured+'。手动试喷需要先确认两轴静止且驱动器当前无报警；'+
        (p.gimbal_stationary_reason?reason(p.gimbal_stationary_reason)+'。':'正在等待新的电机状态。')+
        '当前故障：'+detail+'。自动瞄准仍需恢复云台后才能使用。'+estopDetail;
    }
    if(/timeout|timed out/i.test(detail)){
      const axes=[/ZDT/i.test(detail)?'水平 ZDT':null,/PD42/i.test(detail)?'俯仰 PD42':null].filter(Boolean);
      return (axes.length?axes.join('、')+' 电机':'电机')+'串口回读超时，新的试喷暂不可用；请先恢复云台连接。 详情：'+detail+estopDetail;
    }
    if(detail)return '执行器当前故障：'+detail+'；新的试喷暂不可用。'+estopDetail;
    if(pumpFresh&&p.gimbal_recovery_history_retired===true&&typeof p.gimbal_recovery_history==='string'&&p.gimbal_recovery_history.trim())
      return '之前的云台恢复故障已由新反馈确认恢复；本次拦截原因待更新，请以上方当前试喷条件为准。'+estopDetail;
    return '本次试喷被执行器条件拦截，但尚无具体的当前故障信息；等待状态更新。'+estopDetail;
  }
  function manualPumpReadiness() {
    if(!connected())return '未连接车载服务';
    if(!fresh('pump_manual'))return '等待手动试喷节点的新鲜状态';
    const p=data('pump_manual');
    if(p.can_start===true)return '';
    if($('pumpManualFixedNozzle').checked===true){
      if(p.fixed_nozzle_readiness===true)return '';
      return manualPumpReason(p.fixed_nozzle_ready_reason||p.ready_reason||p.reason||'手动试喷尚未就绪');
    }
    if(p.can_start!==true)return manualPumpReason(p.ready_reason||p.reason||'手动试喷尚未就绪');
    return '';
  }
  function renderManualPump() {
    const p=data('pump_manual'),live=fresh('pump_manual');
    if(!connected()||!live)$('pumpManualFixedNozzle').checked=false;
    const block=manualPumpReadiness(),fixedOnly=$('pumpManualFixedNozzle').checked===true&&p.can_start!==true&&p.fixed_nozzle_readiness===true;
    const phases={idle:'等待人工试喷',preparing_off:'试喷准备 · 先确认关水与就绪',waiting_on:'已提交 · 等待开启反馈',spraying:'单次试喷中',waiting_off:'等待新的关水反馈',cooldown:'已关水 · 冷却中',completed:'本次输出脉冲已完成',cancelled:'已停止试喷',failed:'试喷异常或过程未完整确认'};
    text('pumpManualPhase',live?(phases[p.phase]||reason(p.phase||'等待任务反馈')):'等待车载状态');
    text('pumpManualGate',block||(fixedOnly?
      '本次固定喷嘴确认可用于单次 B13 试水；仍需车端确认停车、控制页和关水状态。':p.gimbal_fault_ignored_for_manual_pump===true?
      '云台动作仍需恢复；已确认两轴静止，可按当前喷嘴方向单次试喷。':
      '可按当前喷嘴方向进行单次试喷，不要求火焰目标或落点标定。'));
    const actual=p.actual_pump_on??p.pump_on;
    text('pumpManualActual',live&&p.feedback_fresh===true&&typeof actual==='boolean'?(actual?'B13：高电平':'B13：低电平'):'B13：反馈未知');
    const outputResult=!live?'等待新鲜的试喷结果；当前不能确认输出状态。':
      p.stop_confirmed===true?'最近一次停止结果：关水已由 FPGA 低电平回包确认。':
      p.confirmed_pump_off===true&&p.blocking!==true?'最近一轮输出结果：已确认关水'+(p.phase==='failed'?'；试喷过程曾中止或未完整确认。':'。'):
      p.blocking===true?'本轮输出结果：仍在等待实际关水确认。':'本轮尚无关水确认记录。';
    text('pumpManualOutcome',outputResult+(live&&p.feedback_fresh!==true?' 最新 B13 反馈已过期，历史关水结果不代表当前电平。':'')+
      (live&&p.confirmed_pump_off===true&&block?' 是否能再次试喷，以上方当前条件为准。':''));
    $('pumpManualStart').disabled=!connected()||state.pumpManualPending===true;
    $('pumpManualStart').title=block?'点击查看当前不能试喷的原因：'+block:'按当前喷嘴方向执行一次所选时长的短喷';
    $('pumpManualDuration').disabled=state.pumpManualPending===true||p.active===true;
    $('pumpManualFixedNozzle').disabled=!connected()||!live||state.pumpManualPending===true||p.active===true;
    text('pumpManualFixedNozzleHint',live&&p.gimbal_communication_exempt===true&&p.active===true?
      '本次正在按固定喷嘴确认执行单次试水，云台未执行运动；不代表电机连接已恢复。':
      live&&p.can_start!==true&&p.fixed_nozzle_readiness===true?
      '本次可在现场确认喷嘴固定后单独试水。请逐次勾选，点击提交即清空；不启用自动瞄准。':
      live&&p.can_start!==true&&p.fixed_nozzle_ready_reason?
      '当前固定喷嘴试水条件：'+manualPumpReason(p.fixed_nozzle_ready_reason)+'。勾选不能解除这些条件。':
      '云台仅缺少通信反馈时，可按现场实际情况作本次确认。每次提交后清空；不改变自动瞄准或正式灭火条件。');
    $('pumpManualStop').disabled=!connected();
    if(state.pumpManualLocalMessage){text('pumpManualReceipt',state.pumpManualLocalMessage);return;}
    const ids=[state.pumpManualRequestId,state.pumpManualClientRequestId].filter(Boolean);
    if(live&&p.stop_confirmed===true&&ids.includes(p.stop_confirmed_request_id)){
      text('pumpManualReceipt','停止已完成：已收到本次停止请求对应的 B13 低电平反馈。');
    }else if(live&&ids.includes(p.last_request_id)){
      const outcome=p.confirmed_pump_off===true&&p.blocking!==true?'；本轮关水已确认':'';
      text('pumpManualReceipt',(p.last_result==='rejected'?'未执行：':'车载回执：')+manualPumpReason(p.last_request_reason||p.reason||p.last_result||'等待执行反馈')+outcome);
    }else if(state.pumpManualClientRequestId){
      const record=(state.data?.commands||[]).find(item=>item.request_id===state.pumpManualClientRequestId);
      if(record&&['rejected','expired'].includes(record.status))text('pumpManualReceipt','请求未执行：'+manualPumpReason(record.reason||record.status)+'；本页不会自动重发。');
    }
  }
  async function submitManualPump(action) {
    const stop=action==='stop';
    let fixedNozzleConfirmed=false;
    if(!stop){
      if(state.pumpManualPending)return;
      const block=manualPumpReadiness();
      fixedNozzleConfirmed=$('pumpManualFixedNozzle').checked===true;
      // One local acknowledgement authorizes only this start attempt. It is
      // never persisted, restored after errors, or copied into stop commands.
      $('pumpManualFixedNozzle').checked=false;
      if(block){
        state.pumpManualLocalMessage='未发送试喷：'+block+'。条件恢复后请重新点击。';
        renderManualPump();toast(block,true);return;
      }
    }
    const duration=Number($('pumpManualDuration').value);
    if(!stop&&(!Number.isInteger(duration)||duration<100||duration>1000)){
      state.pumpManualLocalMessage='未发送试喷：单次喷时必须是100～1000毫秒的整数。';
      renderManualPump();toast(state.pumpManualLocalMessage,true);return;
    }
    const payload={action,request_id:uuid()};
    if(!stop){payload.duration_ms=duration;payload.operator_confirmed=true;}
    if(!stop&&fixedNozzleConfirmed)payload.fixed_nozzle_confirmed=true;
    if(stop)$('pumpManualFixedNozzle').checked=false;
    const generation=(state.pumpManualGeneration||0)+1;state.pumpManualGeneration=generation;
    state.pumpManualPending=true;state.pumpManualRequestId=payload.request_id;state.pumpManualClientRequestId=payload.request_id;
    state.pumpManualLocalMessage='';renderManualPump();
    try{
      const result=await request('/api/pump-manual',payload);
      if(state.pumpManualGeneration===generation){
        state.pumpManualRequestId=result.command_id||result.request_id||payload.request_id;
        text('pumpManualReceipt',stop?'停止请求已提交，等待实际低电平反馈。':'单次试喷请求已提交，等待实际输出反馈；不会自动重试。');
      }
      return result;
    }catch(error){if(state.pumpManualGeneration===generation)text('pumpManualReceipt',(error.definitiveRejection?'未执行：':'提交未确认：')+error.message+'；不会自动重试。');}
    finally{if(state.pumpManualGeneration===generation)state.pumpManualPending=false;renderManualPump();}
  }
  function toast(message, error=false) { text('toast',message);$('toast').className='toast'+(error?' error':'');clearTimeout(state.toastTimer);state.toastTimer=setTimeout(()=>$('toast').classList.add('hidden'),5500); }
  const uuid = () => globalThis.crypto?.randomUUID?.() || ('web-'+Date.now().toString(16)+'-'+Array.from(crypto.getRandomValues(new Uint8Array(12)),x=>x.toString(16).padStart(2,'0')).join(''));
  function badge(id, content, kind='') { text(id,content);$(id).className='badge'+(kind?' '+kind:''); }
  async function request(path, payload) {
    if (!connected()) throw new Error('未连接实时车载服务，指令未发送');
    const headers={'Content-Type':'application/json'};
    const response = await fetch(path,{method:'POST',cache:'no-store',headers,body:JSON.stringify(payload),signal:AbortSignal.timeout(5000)});
    let result;try{result=await response.json();}catch{throw new Error('服务器未返回有效结果：HTTP '+response.status);}
    if (!response.ok) {
      const error=new Error(reason(result?.reason||result?.error||result?.message)||('HTTP '+response.status));
      error.definitiveRejection=response.status>=400&&response.status<500&&result?.synthetic===false&&typeof result?.error==='string';
      throw error;
    }
    return result;
  }
  function selectView(name) {
    if (!['overview','road','fire','voice','evidence','motor','extension'].includes(name)) name='overview';
    const requested=name;
    if(['voice','extension'].includes(name))name='overview';
    if(name==='evidence')name='motor';
    if(state.view==='motor'&&name!=='motor')endMotorRuns('已切换页面');
    state.view=name;const mainView=name==='road'?'fire':name;
    document.querySelectorAll('.view').forEach(el=>el.classList.toggle('active',el.id==='view-'+mainView));document.querySelectorAll('[data-view]').forEach(el=>el.classList.toggle('active',el.dataset.view===mainView));
    const pane=name==='road'?'traffic':'aim';document.querySelectorAll('[data-pane]').forEach(el=>el.classList.toggle('active',el.dataset.pane===pane));document.querySelectorAll('[data-vision-tab]').forEach(el=>el.classList.toggle('active',el.dataset.visionTab===pane));
    if(name==='motor')selectDevicePane(requested==='evidence'?'evidence':initialDevicePane());
    const cameraSlot=$('cameraSlot-'+name);if(cameraSlot)cameraSlot.appendChild($('sharedVisionPanel'));history.replaceState(null,'','#'+name);if(wantsYoloFrames())scheduleFrame(0);else{state.frameEpoch++;stopFrame(false);}stopRoadFrames();scheduleRoadFrame(0);
    if(name==='overview')requestAnimationFrame(()=>{if(state.view!=='overview')return;const box=$('dialogueMessages');if(state.dialogueScrollPending&&state.dialogueFollowEnd!==false)box.scrollTop=box.scrollHeight;state.dialogueScrollPending=false;});
  }
  function initialDevicePane(){
    if(state.devicePane)return state.devicePane;
    const a=data('actuators');
    return connected()&&fresh('actuators')&&a.dry_run===false&&a.enabled===true?'gimbal':'motors';
  }
  function selectDevicePane(name){
    if(!['motors','gimbal','advanced','evidence'].includes(name))name='motors';
    if(state.devicePane==='motors'&&name!=='motors')endMotorRuns('已离开电机调试面板');
    state.devicePane=name;document.querySelectorAll('[data-device-pane]').forEach(el=>el.classList.toggle('active',el.dataset.devicePane===name));document.querySelectorAll('[data-device-tab]').forEach(el=>el.classList.toggle('active',el.dataset.deviceTab===name));
  }
  const mobileTabs = document.createElement('nav');mobileTabs.className='mobile-tabs';mobileTabs.setAttribute('aria-label','移动端主导航');mobileTabs.innerHTML='<button data-view="overview">任务驾驶台</button><button data-view="fire">视觉与瞄准</button><button data-view="motor">设备调试</button>';$('modeBanner').before(mobileTabs);
  document.querySelectorAll('[data-vision-tab]').forEach(btn=>btn.addEventListener('click',()=>selectView(btn.dataset.visionTab==='traffic'?'road':'fire')));
  document.querySelectorAll('[data-device-tab]').forEach(btn=>btn.addEventListener('click',()=>selectDevicePane(btn.dataset.deviceTab)));
  $('audioTestDeviceOpen').addEventListener('click',()=>$('audioTestOpen').click());
  document.querySelectorAll('[data-view]').forEach(btn=>btn.addEventListener('click',()=>selectView(btn.dataset.view)));
  document.querySelector('.brand').addEventListener('click',event=>{event.preventDefault();selectView('overview');});

  async function sendCommand(command, {stop=false,confirmed=false,targetExpectation=null}={}) {
    const clean=String(command||'').trim();if(!clean)return;
    if(!stop&&!canControl()){toast('当前不能提交任务：'+reason(state.data?.control?.read_only_reason||'请等待新鲜的任务节点状态'),true);return;}
    const dangerAction=dangerKind(clean)!==null;
    if(dangerAction&&!confirmed){confirmAction(clean);return;}
    state.pending=true;renderControls();
    try {
      const payload={text:clean,request_id:uuid()};if(targetExpectation)payload.target_expectation=targetExpectation;
      const result=await request('/api/command',payload);
      state.localMessages.push({text:clean,at:result.queued_at||Date.now()/1000,id:result.command_id||result.request_id||uuid(),action:result.action,status:result.status||'queued'});state.localMessages=state.localMessages.slice(-25);
      toast('实车指令已提交，等待任务回执；不代表动作完成。');
      if($('commandText').value.trim()===clean)$('commandText').value='';renderDialogue();
      return result;
    } catch(error) { toast((error.definitiveRejection?'未执行：':'提交未确认：')+error.message+'。本页不会自动重发。',true); }
    finally {state.pending=false;renderControls();}
  }
  function normalizedCommand(value){let clean=String(value).replace(/[\s，。！？、,.!?：:；;]/g,'').trim();for(const prefix of ['你好消防小车','消防小车','你好小车','小车','机器人'])if(clean.startsWith(prefix)){clean=clean.slice(prefix.length);break;}for(const prefix of ['请你','请'])if(clean.startsWith(prefix)){clean=clean.slice(prefix.length);break;}return clean;}
  function dangerKind(value){const clean=normalizedCommand(value);if(clean==='前进直到火焰居中然后自动灭火')return'auto';if(/^(不|别|禁止)/.test(clean)||['如果','假如','能否','能不能','是否','吗','么','或者','还是','然后'].some(word=>clean.includes(word)))return null;if(/^手动试喷(?:[0-9零〇一二两三四五六七八九十百千]+毫秒)?$/.test(clean))return'manual-pump';if(['对选定火焰自动瞄准喷水并关水','对选定火焰自动瞄准喷水关水','一键瞄准喷水关水','一键瞄准喷水并关水','对选定火焰执行一轮灭火'].includes(clean))return'selected-round';if(['开始喷水','开启喷水','开启喷水灭火','开始喷水灭火','启动水泵','开启水泵'].includes(clean))return'spray';if(['自动找火灭火','开始自动找火灭火','开启自动灭火','开始自动灭火','切换自动模式','到达后自动灭火','到达后自动找火灭火','到达火场后自动灭火','设置自动灭火模式'].includes(clean)||/^(去|前往|到|导航到).+(自动找火灭火|自动灭火)$/.test(clean))return'auto';return null;}
  function selectionIdentity(){const s=data('fire_selection');return s.valid===true&&Number.isInteger(s.target_id)?[s.target_id,s.selection_generation,s.registry_session].join(':'):null;}
  function confirmAction(command) {
    const manualTest=dangerKind(command)==='manual-pump';
    const selectedRound=dangerKind(command)==='selected-round';
    const spray=selectedRound||dangerKind(command)==='spray';
    const selected=data('fire_selection');
    if(spray&&!selectionIdentity()){toast('尚无新鲜且有效的选定火焰。请先观察并选择目标。',true);return;}
    state.pendingAction={command,identity:spray?selectionIdentity():null,spray,selectedRound,targetExpectation:selectedRound?selectedRoundExpectation():null};
    text('actionTitle',manualTest?'确认按当前喷嘴方向短喷':selectedRound?'确认瞄准、短喷并关水':spray?'确认对选定火焰提交喷水':'确认自动灭火模式');
    text('actionDescription',manualTest?'执行一次100～1000毫秒的输出脉冲，默认400毫秒。不会自动移动云台；请确认当前喷嘴方向。':selectedRound?'对当前选定火焰执行一轮：双轴瞄准、确认角度到位、有限短喷、确认关水。目标变化或反馈失效会中止。':spray?'这会请求对当前选定目标短时喷水。请核对目标编号；选择变化或失效时，本次确认会被取消。':'这条指令可能在条件满足后自动选择一个火焰、瞄准并短时喷水。每轮处理一个目标，随后复查。');
    text('actionTarget',(spray?'当前选定：火焰 #'+selected.target_id+'\n':'')+'将提交：'+command+'\n'+(manualTest?'独立人工试喷入口：不要求火焰标定；仍需停车、水泵就绪与真实开关水反馈。':data('actuators').dry_run===true?'执行器当前为 dry-run；不代表真实喷水。':'实际执行仍取决于停车、标定、瞄准与硬件条件。'));
    $('actionCheck').checked=false;$('actionConfirm').disabled=true;$('actionDialog').showModal();
  }
  async function handleCommandButton(btn) {
    const result=await sendCommand(btn.dataset.command,{stop:btn.dataset.stop==='true'});
    if(result&&btn.dataset.afterView)selectView(btn.dataset.afterView);
  }
  document.querySelectorAll('[data-command]').forEach(btn=>btn.addEventListener('click',()=>handleCommandButton(btn)));
  $('actionCheck').addEventListener('change',()=>$('actionConfirm').disabled=!$('actionCheck').checked);
  $('actionCancel').addEventListener('click',()=>$('actionDialog').close());
  $('actionConfirm').addEventListener('click',()=>{const action=state.pendingAction;if(!action||!$('actionCheck').checked)return;if(action.spray&&action.identity!==selectionIdentity()){toast('目标已变化或过期，请重新选择并确认。',true);$('actionDialog').close();return;}if(!canControl()){toast('当前连接或控制状态已变化，未提交。',true);return;}$('actionDialog').close();sendCommand(action.command,{confirmed:true,targetExpectation:action.targetExpectation});});
  $('actionDialog').addEventListener('close',()=>state.pendingAction=null);
  $('sprayBtn').addEventListener('click',()=>sendCommand('开始喷水'));
  $('extinguishSelectedBtn').addEventListener('click',()=>sendCommand('对选定火焰自动瞄准喷水并关水'));
  $('fireSetupOpen').addEventListener('click',()=>{$('fireSetupDialog').showModal();renderFireSetup();});
  $('fireSetupClose').addEventListener('click',()=>$('fireSetupDialog').close());
  $('pumpManualFixedNozzle').checked=false;
  $('pumpManualOpen').addEventListener('click',()=>{state.pumpManualLocalMessage='';$('pumpManualFixedNozzle').checked=false;$('pumpManualDialog').showModal();renderManualPump();});
  $('pumpManualClose').addEventListener('click',()=>$('pumpManualDialog').close());
  $('pumpManualDialog').addEventListener('close',()=>{$('pumpManualFixedNozzle').checked=false;});
  $('pumpManualDialog').addEventListener('cancel',()=>{$('pumpManualFixedNozzle').checked=false;});
  $('pumpManualFixedNozzle').addEventListener('change',()=>{state.pumpManualLocalMessage='';renderManualPump();});
  window.addEventListener('pageshow',()=>{$('pumpManualFixedNozzle').checked=false;});
  $('pumpManualStart').addEventListener('click',()=>submitManualPump('start'));
  $('pumpManualStop').addEventListener('click',()=>submitManualPump('stop'));
  $('fireSetupManual').addEventListener('click',()=>{$('fireSetupDialog').close();selectView('motor');selectDevicePane('gimbal');});
  $('fireSetupExport').addEventListener('click',()=>{
    const keys=['mission','actuators','stereo','stereo_observation','stereo_last_valid_observation','fire_selection','fire_proposal','gimbal_manual','extension'];
    const report={captured_at:new Date().toISOString(),connected:connected(),topics:Object.fromEntries(keys.map(key=>[key,{fresh:fresh(key),data:seen(key)||null}]))};
    const link=document.createElement('a'),url=URL.createObjectURL(new Blob([JSON.stringify(report,null,2)],{type:'application/json'}));
    link.href=url;link.download='fire-readiness-'+Date.now()+'.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  });
  $('navigateBtn').addEventListener('click',()=>{const number=$('buildingSelect').value,policy=$('arrivalSelect').value;if(!number||!connected()){toast('请等待车载楼栋配置并选择目的地。',true);return;}sendCommand('去'+number+'号楼'+(policy==='auto'?'自动灭火':'等待语音指令'));});
  function extensionCommand(kind, value, option) {
    if(kind==='reference'){
      const [x,y,heading]=value.map(Number);
      if(![x,y,heading].every(Number.isFinite)||x<0||x>=352||y<0||y>=320||heading< -180||heading>360)throw new Error('参考坐标超出地图范围：X 0～352、Y 0～320 cm，朝向 -180～360°');
      return '确认起点X'+x+'厘米Y'+y+'厘米朝向'+heading+'度';
    }
    if(kind==='distance'){
      const n=Number(value);
      if(!Number.isInteger(n)||n<5||n>1000||!['forward','reverse'].includes(option))throw new Error('距离请输入 5～1000 cm 的整数');
      return (option==='forward'?'前进':'后退')+n+'厘米';
    }
    if(kind==='turn'){
      const n=Number(value);
      if(!Number.isInteger(n)||n<1||n>8||!['left','right'].includes(option))throw new Error('路口序号请输入 1～8 的整数');
      return '一直前进在第'+n+'个路口'+(option==='left'?'左转':'右转');
    }
    if(kind==='firefighter'&&value==='observe')return '前进直到消防员居中';
    throw new Error('未知拓展任务参数');
  }
  function submitExtension(kind,value,option){
    try{const command=extensionCommand(kind,value,option);sendCommand(command);}catch(error){toast(error.message,true);}
  }
  $('extensionDistanceForm').addEventListener('submit',event=>{event.preventDefault();submitExtension('distance',$('extensionDistance').value,$('extensionDirection').value);});
  $('extensionTurnForm').addEventListener('submit',event=>{event.preventDefault();submitExtension('turn',$('extensionJunction').value,$('extensionTurn').value);});
  $('extensionFireForm').addEventListener('submit',event=>{event.preventDefault();submitExtension('firefighter',$('extensionAfter').value);});
  $('extensionReferenceOpen').addEventListener('click',()=>$('extensionReferenceDialog').showModal());
  $('extensionReferenceForm').addEventListener('submit',event=>{event.preventDefault();try{const command=extensionCommand('reference',[$('extensionReferenceX').value,$('extensionReferenceY').value,$('extensionReferenceHeading').value]);$('extensionReferenceDialog').close();sendCommand(command);}catch(error){toast(error.message,true);}});
  $('commandForm').addEventListener('submit',event=>{event.preventDefault();const value=$('commandText').value.trim(),stop=['停止喷水','停止试喷','紧急停止','停车','停止'].includes(normalizedCommand(value));if(!stop&&!dialogueInputReady()){toast('交通任务期间暂停问答；可切回拓展1或拓展2。',true);return;}sendCommand(value,{stop});});
  $('dialogueMessages').addEventListener('scroll',()=>{const box=$('dialogueMessages');if(box.clientHeight>0)state.dialogueFollowEnd=box.scrollHeight-box.scrollTop-box.clientHeight<70;},{passive:true});
  $('commandText').addEventListener('keydown',event=>{if(event.key==='Enter'&&(event.ctrlKey||event.metaKey)){event.preventDefault();$('commandForm').requestSubmit();}});
  document.querySelectorAll('[data-fill]').forEach(btn=>btn.addEventListener('click',()=>{$('commandText').value=btn.dataset.fill;$('commandText').focus();}));
  $('questionLibraryOpen').addEventListener('click',()=>$('questionLibraryDialog').showModal());
  document.querySelectorAll('[data-example]').forEach(btn=>btn.addEventListener('click',()=>{
    $('commandText').value=btn.dataset.example;$('questionLibraryDialog').close();$('commandText').focus();
  }));
  $('targetList').addEventListener('click',event=>{const btn=event.target.closest('[data-target]');if(btn&&!btn.disabled)sendCommand('选择'+btn.dataset.target+'号火焰');});
  function voiceStartBlock() {
    if(state.voicePower?.pending||data('voice').voice_control?.status==='pending')return '语音开关正在切换，等待车载确认后再开始收音';
    return data('voice').voice_enabled===false?'语音已关闭（收音、识别、播报）；请先开启语音，文字指令继续可用':'';
  }
  function renderVoicePower() {
    const v=data('voice'),ui=state.voicePower,known=typeof v.voice_enabled==='boolean';
    const configured=(name,enabled)=>name+(enabled===true?'已启用':enabled===false?'未启用':'待确认');
    const effective=[configured('收音/识别',v.microphone_enabled),configured('播报',v.tts_enabled)].join('，');
    const record=ui&&(state.data?.commands||[]).find(item=>item.request_id===ui.requestId);
    // Only this request's applied feedback confirms its outcome. Neither HTTP
    // success nor an unrelated change of the effective switch acknowledges it.
    const feedback=ui&&[v.voice_control,record?.voice_control_status,ui.receipt?.voice_control_status]
      .find(item=>item?.request_id===ui.requestId&&['applied','rejected'].includes(item.status));
    if(ui&&feedback?.status==='applied'&&feedback.enabled===ui.enabled&&v.voice_enabled===ui.enabled){
      if(ui.outcome!=='applied')state.voiceControl={};
      ui.pending=false;ui.outcome='applied';ui.error='';
    }else if(ui&&(feedback?.status==='rejected'||['rejected','expired'].includes(record?.status))){
      ui.pending=false;ui.outcome='rejected';ui.error=reason(feedback?.reason||record?.reason||'语音切换未执行');
    }else if(ui?.pending&&performance.now()-ui.startedAt>=12000){
      ui.pending=false;ui.outcome='unknown';ui.error='尚未收到本次生效确认；请核对当前状态，本页不会自动重发';
    }
    const busy=Boolean(ui?.pending||v.voice_control?.status==='pending');
    let note=!connected()?'连接中断，无法确认语音开关状态':!known?'等待车载语音状态':
      v.voice_enabled?'总开关已开启；'+effective+'；不会自动录音':'语音已关闭（收音、识别、播报）；文字指令与 Qwen 仍可用';
    if(ui?.pending)note=(ui.enabled?'正在开启':'正在关闭')+'语音，等待本次车载生效确认';
    else if(ui?.error)note=(ui.outcome==='rejected'?'未执行：':'提交未确认：')+ui.error;
    else if(v.voice_control?.status==='pending')note='车载语音正在切换，等待采集、识别与播放状态确认';
    else if(ui?.outcome==='applied'&&known&&v.voice_enabled===ui.enabled)note=(ui.enabled?'总开关开启已确认；'+effective+'；不会自动录音':'本次关闭已确认（收音、识别、播报）')+'；文字指令与 Qwen 仍可用';
    text('voicePowerStatus',note);
    const button=$('voicePowerToggle');button.setAttribute('aria-checked',String(v.voice_enabled===true));
    button.disabled=!connected()||state.data?.control?.voice_control_ready!==true||!known||busy;
    button.textContent=busy?'切换中…':!known?'等待状态':v.voice_enabled?'关闭语音':'开启语音';
    button.title=!connected()?'请连接车载服务':state.data?.control?.voice_control_ready!==true?'等待语音开关控制节点':!known?'等待有效语音开关状态':v.voice_enabled?'停止并关闭车载收音、识别与播报；保留文字输入和 Qwen':'按车载配置恢复语音功能；未配置的麦克风或播报不会被自动启用，也不会自动录音';
    button.closest('.voice-power-bar').dataset.state=busy?'pending':ui?.error?'error':known&&v.voice_enabled?'on':'off';
  }
  async function submitVoicePower() {
    const v=data('voice');
    if(state.voicePower?.pending||v.voice_control?.status==='pending'||!connected()||
        state.data?.control?.voice_control_ready!==true||typeof v.voice_enabled!=='boolean')return;
    const ui={requestId:uuid(),enabled:!v.voice_enabled,pending:true,startedAt:performance.now(),outcome:'pending',error:''};
    state.voicePower=ui;renderVoicePower();renderControls();
    try{
      const result=await request('/api/voice-control',{enabled:ui.enabled,request_id:ui.requestId});
      if(state.voicePower!==ui)return;
      ui.receipt=result;ui.commandId=result.command_id;
      if(['rejected','expired'].includes(result.status)){
        ui.pending=false;ui.outcome='rejected';ui.error=reason(result.reason||'语音开关请求被拒绝');
      }
    }catch(error){
      if(state.voicePower===ui){ui.pending=false;ui.outcome=error.definitiveRejection?'rejected':'unknown';ui.error=error.message+'；未确认生效，不会自动重发';}
    }finally{
      if(state.voicePower===ui){renderVoicePower();renderControls();renderVoiceCapture();}
    }
  }
  $('voicePowerToggle').addEventListener('click',()=>submitVoicePower());
  async function submitListen(enabled) {
    if(enabled&&voiceStartBlock()){toast(voiceStartBlock(),true);return;}
    const ui=state.voiceControl||(state.voiceControl={});
    if(enabled?ui.pendingStart:ui.pendingStop)return;
    const requestId=uuid(),payload={enabled,request_id:requestId};
    if(enabled){ui.pendingStart=requestId;ui.startRequestId=requestId;ui.stop=null;}
    else {
      ui.pendingStop=requestId;
      if(ui.startRequestId)payload.cancel_start_request_id=ui.startRequestId;
      const voice=data('voice');
      ui.stop={requestId,baselineStopAt:voice.stop_requested_at||0,baselineStopSeq:voice.stop_request_seq||0,error:''};
    }
    renderControls();renderVoiceCapture();
    try {
      const result=await request('/api/listen',payload);
      if(!enabled&&ui.stop?.requestId===requestId)ui.stop.commandId=result.command_id;
      // A slow Start response must not overwrite a newer End click's feedback.
      if(!enabled||!ui.stop)toast(enabled?'开始请求已提交，等“正在车载收音”后再说话。':'结束请求已提交，正在等待麦克风停止确认。');
    } catch(error) {
      if(!enabled&&ui.stop?.requestId===requestId)ui.stop.error=error.message;
      if(!enabled||!ui.stop)toast('语音请求未确认：'+error.message,true);
    } finally {
      if(enabled&&ui.pendingStart===requestId)ui.pendingStart=null;
      if(!enabled&&ui.pendingStop===requestId)ui.pendingStop=null;
      renderControls();renderVoiceCapture();
    }
  }
  for(const [id,enabled] of [['listenStart',true],['listenStop',false]])$(id).addEventListener('click',()=>submitListen(enabled));
  const audioTestUI={pending:false,pendingEnabled:null,requestId:null,sessionId:null,submittedAt:0,closeAfterSubmit:false,feedbackError:''};
  const testActive = () => data('voice').audio_test_active===true || (fresh('audio_test') && ['preparing','listening','transcribing'].includes(data('audio_test').state));
  async function submitAudioTest(enabled) {
    if(enabled&&voiceStartBlock()){toast(voiceStartBlock(),true);return;}
    if(audioTestUI.pending)return;
    audioTestUI.pending=true;audioTestUI.pendingEnabled=enabled;audioTestUI.feedbackError='';renderAudioTest();
    let submitted=false;
    try {
      const result=await request('/api/audio-test',{enabled,request_id:uuid()});
      audioTestUI.requestId=result.command_id;audioTestUI.submittedAt=performance.now();submitted=true;
      if(enabled)audioTestUI.sessionId=result.command_id;
      toast(enabled?'测试请求已提交，等“正在收声”后再说话。':'结束测试请求已提交，等待本地转写结果。');
    } catch(error) {toast('收声测试请求未确认：'+error.message,true);}
    finally {
      audioTestUI.pending=false;renderAudioTest();
      if(audioTestUI.closeAfterSubmit){audioTestUI.closeAfterSubmit=false;if(enabled&&submitted)await submitAudioTest(false);$('audioTestDialog').close();}
    }
  }
  function closeAudioTest() {
    if(audioTestUI.pending){audioTestUI.closeAfterSubmit=true;return;}
    const last=seen('audio_test')||{};
    const ownSession=audioTestUI.sessionId&&[last.command_id,last.session_id].includes(audioTestUI.sessionId);
    if((ownSession&&testActive()) || (audioTestUI.requestId&&audioTestUI.pendingEnabled===true&&performance.now()-audioTestUI.submittedAt<8000))submitAudioTest(false);
    $('audioTestDialog').close();
  }
  $('audioTestOpen').addEventListener('click',()=>{renderAudioTest();$('audioTestDialog').showModal();});
  $('audioTestStart').addEventListener('click',()=>submitAudioTest(true));
  $('audioTestStop').addEventListener('click',()=>submitAudioTest(false));
  $('audioTestClose').addEventListener('click',closeAudioTest);
  $('audioTestDialog').addEventListener('cancel',event=>{event.preventDefault();closeAudioTest();});
  function renderAudioTest() {
    const v=data('voice'),last=seen('audio_test')||{},live=fresh('audio_test');
    const matches=!audioTestUI.sessionId||[last.command_id,last.session_id].includes(audioTestUI.sessionId);
    if(audioTestUI.requestId&&last.request_rejected===true&&last.request_id===audioTestUI.requestId){
      audioTestUI.feedbackError=reason(last.request_reason||last.reason);audioTestUI.requestId=null;
    } else if(audioTestUI.requestId&&((last.request_accepted===true&&last.request_id===audioTestUI.requestId)||(audioTestUI.pendingEnabled===true&&[last.command_id,last.session_id].includes(audioTestUI.requestId))))audioTestUI.requestId=null;
    const queued=audioTestUI.requestId&&(state.data?.commands||[]).find(c=>c.command_id===audioTestUI.requestId);
    if(['expired','rejected'].includes(queued?.status))audioTestUI.feedbackError=reason(queued.reason||'测试请求未接受');
    const waiting=Boolean(audioTestUI.requestId&&!['expired','rejected'].includes(queued?.status)&&performance.now()-audioTestUI.submittedAt<8000);
    const active=testActive(),states={preparing:'准备麦克风与模型',listening:'正在收声 · 请说话',transcribing:'收音结束 · 正在转写',completed:'测试完成',cancelled:'测试已取消',error:'测试未完成'};
    let hint='点击开始，等出现“正在收声”后，说“今天进行语音测试”。';
    if(!connected())hint='当前连接中断，无法确认麦克风状态；已开始的测试最多采集 10 秒。';
    else if(waiting||audioTestUI.pending)hint='请求已提交；收到车载“正在收声”反馈后再开始说话。';
    else if(!matches&&audioTestUI.sessionId)hint=reason(queued?.reason||'尚未收到本次测试反馈。可以重新测试，或点击结束请求停止。');
    else if(last.state==='preparing'&&live)hint='正在打开设备或加载本地中文模型，请稍候。';
    else if(last.state==='listening'&&live)hint='现在请对无线麦克风说“今天进行语音测试”。说完可以点“结束并识别”。';
    else if(last.state==='transcribing'&&live)hint='麦克风已关闭，正在 X5 本地识别本次录音。';
    else if(last.reason)hint=last.reason==='low_asr_confidence'?'识别把握偏低，已保留文字供检查；请靠近麦克风清楚重说。':reason(last.reason);
    else if(last.state==='completed')hint='已收到本次本地识别结果。请核对文字是否与你实际说的一致。';
    if(!live&&['preparing','listening','transcribing'].includes(last.state)&&!waiting)hint='测试状态已过期，无法确认当前进度；已开始的测试最多采集 10 秒。';
    if(audioTestUI.feedbackError&&!waiting)hint='本次请求未接受：'+audioTestUI.feedbackError;
    if(voiceStartBlock()&&!active)hint=voiceStartBlock();
    text('audioTestState',waiting||audioTestUI.pending?'等待车载反馈':!matches?'等待本次测试结果':last.state?(live?'':'历史 · ')+(states[last.state]||last.state):'等待开始');
    text('audioTestHint',hint);
    text('audioTestTime',matches&&last.state?num(last.elapsed_s,1)+' / '+num(last.duration_s||10,0)+' s':'— / 10 s');
    const serverNow=(state.data?.server_time||0)+(performance.now()-state.received)/1000;
    const meterFresh=live&&matches&&last.capture_active===true&&last.state==='listening'&&finite(last.level_stamp)&&serverNow-last.level_stamp>=0&&serverNow-last.level_stamp<1.6;
    const level=meterFresh&&finite(last.level_dbfs)?Math.max(-96,Math.min(0,last.level_dbfs)):null;
    $('audioTestMeterFill').style.width=level===null?'0%':(100*(level+96)/96)+'%';
    $('audioTestMeter').setAttribute('aria-valuenow',level===null?-96:level);
    $('audioTestMeter').setAttribute('aria-valuetext',level===null?'没有实时采样':num(level,1)+' dBFS');
    $('audioTestMeter').dataset.overload=String(meterFresh&&last.peak_dbfs>=-.5);
    text('audioTestLevel',level===null?(last.state?'没有实时采样':'尚无采样'):num(level,1)+' dBFS');
    text('audioTestPeak',meterFresh&&finite(last.peak_dbfs)?num(last.peak_dbfs,1)+' dBFS':'—');
    text('audioTestText',matches?(last.text||(['preparing','listening','transcribing'].includes(last.state)?'等待本次识别文字':last.state==='completed'?'未识别到有效文字':'尚无测试结果')):'等待本次识别文字');
    const resultTime=last.result_stamp||last.completed_at;
    text('audioTestResultMeta',matches&&last.state?['本次采样 '+num(last.sampled_s,1)+' s',finite(last.confidence)?'解码评分 '+num(last.confidence*100,0)+'%（不是准确率）':last.confidence_available===false?'本模型不提供置信度':'暂无识别评分',last.engine||'',finite(last.decode_s)?'推理 '+num(last.decode_s,2)+' s':'',resultTime?'结果 '+clock(resultTime):'', '未下发任务'].filter(Boolean).join(' · '):'录音最多 10 秒；仅在 X5 本地转写，不保存录音文件。');
    text('audioTestDevice','输入：'+(last.input_device||'X5 上的 USB 接收器')+'；不使用电脑麦克风。');
    $('audioTestStart').disabled=!(connected()&&state.data?.control?.audio_test_ready)||!!voiceStartBlock()||audioTestUI.pending||waiting||active||v.listening===true;
    $('audioTestStart').title=voiceStartBlock()||'采集测试录音，仅转写，不执行任务';
    // Stop remains available if status is stale, as long as the ROS subscriber exists.
    $('audioTestStop').disabled=!(connected()&&state.data?.ros?.available&&state.data?.ros?.audio_test_ready)||audioTestUI.pending;
  }
  function extensionMotionReadiness(capability) {
    const e=data('extension');
    if(!connected()||!fresh('extension')||e.status_fresh!==true)return {ready:false,reason:'extension_status_unavailable'};
    if(e.real_uart!==true||e.protocol_ready!==true)return {ready:false,reason:'extension_protocol_not_ready'};
    if(e.capabilities?.[capability]!==true)return {ready:false,reason:'extension_capability_unavailable_'+capability};
    if(e.can_move!==true)return {ready:false,reason:e.reason||'extension_motion_not_ready'};
    return {ready:true,reason:''};
  }
  function trafficDriveReadiness() {
    const gate=extensionMotionReadiness('traffic'),e=data('extension');
    if(!gate.ready)return gate;
    if(e.task!=='traffic'||e.fpga_task_mode!==3||e.mode_transition)return {ready:false,reason:'select_traffic_task_and_wait_ready_before_drive'};
    if(e.zebra_detection_enabled!==true||e.zebra_transition)return {ready:false,reason:'enable_fpga_zebra_detection_before_traffic_drive'};
    return gate;
  }
  function zebraSwitchReadiness() {
    const e=data('extension');
    if(!connected()||!fresh('extension')||e.real_uart!==true||e.status_fresh!==true)return {ready:false,reason:'extension_status_unavailable'};
    if(e.protocol_ready!==true||e.capabilities?.zebra_switch!==true)return {ready:false,reason:'fpga_capability_not_implemented_zebra_switch'};
    if(e.zebra_transition)return {ready:false,reason:'zebra_switch_pending'};
    // The core allows a stopped red-light wait even though movement is busy.
    if(e.zebra_can_change!==true)return {ready:false,reason:e.zebra_change_reason||'zebra_switch_not_ready'};
    return {ready:true,reason:''};
  }
  function zebraDisplayState() {
    const e=data('extension');
    if(!connected()||!fresh('extension')||e.real_uart!==true)return '未连接';
    if(e.status_fresh!==true||e.protocol_ready!==true||e.capabilities?.zebra_switch!==true||typeof e.zebra_detection_enabled!=='boolean')return '未知';
    return e.zebra_detection_enabled?'开':'关';
  }
  function modeButtonActive(mode, mission) {
    if(mode==='extension_one')return fresh('extension')&&data('extension').task==='vision';
    if(mode==='extension_two')return fresh('extension')&&['dialogue','semantic'].includes(data('extension').task);
    return mode===mission.mode;
  }
  function renderExtension() {
    const e=data('extension'),m=data('mission'),v=data('voice'),d=data('dialogue_status');
    const rawActive=e.active&&typeof e.active==='object'?e.active:{};
    const rawResult=e.last_result&&typeof e.last_result==='object'?e.last_result:{};
    const active=rawActive.command_id&&(['completed','fault'].includes(rawActive.phase)&&rawResult.command_id&&rawResult.command_id!==rawActive.command_id)?{}:rawActive;
    const result=active.command_id&&rawResult.command_id&&active.command_id!==rawResult.command_id?{}:rawResult;
    const phases={planning:'生成路口约束路线',uploading_route:'向 FPGA 上传本次路线',waiting_stop:'切换任务前等待停稳',waiting_idle:'等待 FPGA 空闲与停稳确认',waiting_mode:'等待 FPGA 切换任务',waiting_mode_stop:'任务模式已切换，等待 FPGA 停稳',waiting_ack:'等待 FPGA 接收指令',waiting_traffic_speed:'等待 FPGA 确认交通限速',traffic_driving:'交通任务行驶中',moving:active.action==='traffic_drive'?'交通任务行驶中 · 等待斑马线停车':'等待车辆实际运动反馈',seeking:'前进并寻找消防员',centering:'确认同一消防员居中',stopping:'已请求停车，等待停稳',completed:'已收到任务完成证据',fault:'任务中止 / 故障'};
    const reference=e.reference||{};
    text('extensionReferenceState',reference.confirmed===true?'已确认：('+num(reference.x_cm,2)+', '+num(reference.y_cm,2)+') cm / '+num(reference.heading_deg,1)+'°':'路口任务前，按地图摆放并确认车头位置。');
    text('extensionLink',fresh('extension')?(e.real_uart===true&&e.protocol_ready===true?'拓展 UART 协议已连接':'X5 在线 · 等待 FPGA 拓展协议'):'等待拓展节点');
    text('extensionVision',fresh('mission')?(m.yolo_requested===true?'请求识别：火焰 / 人物 / 消防员':'YOLO 按任务关闭'):'等待识别状态');
    text('extensionVoice',fresh('voice')?(v.microphone_enabled===true?'USB 麦克风已启用':'USB 麦克风未启用'):'等待收音节点');
    text('extensionDialogue',e.dialogue_enabled!==true?'对话模式未开启':fresh('dialogue_status')?(d.state==='running'||d.state==='thinking'?'正在本地推理':reason(d.state||'ready')):'等待本地模型反馈');
    text('extensionZebraState',zebraDisplayState());
    text('extensionZebraDetail',e.zebra_transition?(e.zebra_transition.phase==='waiting_readback'?'指令已确认，等待实际开关回读':'请求已发送，等待 FPGA 回执'):(['开','关'].includes(zebraDisplayState())?'以上为 FPGA 实际状态回读。':'需要实际串口回读；发出请求不会改变显示状态。'));
    text('extensionTrafficState',fresh('extension')?(e.task==='traffic'?(e.mode_transition?'交通模式切换中':active.action==='traffic_drive'&&!['completed','fault'].includes(active.phase)?'交通行驶任务':'识别已开 · 未自动行驶'):'交通任务关闭'):'等待任务反馈');
    const phase=e.mode_transition?.phase||active.phase;
    text('extensionPhase',!fresh('extension')?'等待车载任务状态':phase?(phases[phase]||phase):result.completed===true?'上一任务已反馈完成':e.busy===true?'任务执行中':e.task==='vision'?'拓展一 · 观察与问答':['dialogue','semantic'].includes(e.task)?'拓展二 · 语义导航与搜索消防员':e.task==='traffic'?'独立交通任务 · 等待灯色与 FPGA 反馈':'等待明确运动指令');
    text('extensionReason',fresh('extension')?(active.action==='seek_firefighter'?regionReason(result.reason||e.reason):reason(result.reason||e.reason)):'没有新鲜车载证据；不会显示为执行成功。');
    text('extensionCommandId',active.command_id||result.command_id||'—');
    text('extensionAck',!fresh('extension')?'—':result.accepted===false?'请求未接受':result.completed===true?'完成已确认':result.accepted===true?'请求已接收 · 不代表动作完成':phase==='waiting_ack'?'等待回执':'等待本次回执');
    const progress=[];
    if(finite(e.fpga_status?.remaining_mm)&&active.action==='manual_move')progress.push('剩余 '+num(e.fpga_status.remaining_mm/10,1)+' cm');
    if(Number.isInteger(e.junction_count))progress.push('已过 '+e.junction_count+' 个路口');
    if(e.real_uart===true)progress.push(measuredVehicleMotion(e,fresh('extension'),m,fresh('mission')).text);
    text('extensionProgress',progress.join(' / ')||'等待实际里程 / 路口证据');
    text('extensionTarget',active.action==='seek_firefighter'?(Number.isInteger(e.target_id)?'消防员 #'+e.target_id:'等待稳定消防员')+(Array.isArray(e.center_error_norm)&&finite(e.center_error_norm[0])?' / 水平偏差 '+num(e.center_error_norm[0]*100,1)+'%':''):'仅寻找消防员时记录本次目标');
    if(fresh('extension')&&isCurrentRegionTask(e)){
      const r=e.region_search,view=regionTaskPresentation(r),target=r.centering?.current_target_id;
      text('extensionPhase',(r.region_id||'指定')+' 区 · '+view.title);
      text('extensionReason',view.detail||'以区域任务阶段及 FPGA 实际反馈为准');
      text('extensionCommandId',r.command_id);
      const outcome=regionTargetOutcome(r);
      text('extensionAck',view.failed?'任务中止 · 未完成':view.phase==='CANCELLED'?'任务已取消':['COMPLETE','COMPLETED'].includes(view.phase)?
        (outcome.unresolved.length||outcome.pending.length?'路线已结束 · 仍有消防员未处置':'任务完成反馈已收到'):'区域任务执行中');
      text('extensionTarget',view.active&&Number.isInteger(target)?'本轮当前消防员 #'+target:view.terminal?'本轮已结束，不再处理队列目标':'等待稳定消防员');
    }
  }
  function renderControls() {
    const ready=canControl();document.querySelectorAll('[data-command],[data-control]').forEach(btn=>{btn.disabled=btn.dataset.stop==='true'?!(connected()):!ready;});
    text('controlBadge',ready?'实车控制就绪':connected()?'等待任务链路':'等待连接');$('controlBadge').dataset.state=ready?'ok':'idle';
    $('buildingSelect').disabled=!connected()||!$('buildingSelect').value;$('navigateBtn').disabled=!ready||!$('buildingSelect').value||!navigationReady();$('navigateBtn').title=navigationReady()?'提交实际导航任务':'FPGA 尚无有效实车通信反馈';
    const dialogueReady=dialogueInputReady();$('sendCommand').disabled=!ready||!dialogueReady;
    $('sendCommand').title=dialogueReady?'向 X5 提交文字问题或明确任务':'待机可说“开启任务1/2”；交通任务期间暂停问答';
    const listen=connected()&&state.data?.control?.listen_ready&&dialogueReady;$('listenStart').disabled=!listen||!!voiceStartBlock()||data('voice').listening===true||Boolean(state.voiceControl?.pendingStart||state.voiceControl?.pendingStop);
    $('listenStart').title=voiceStartBlock()||'使用 X5 上的 USB 接收器开始收音';
    $('listenStop').disabled=!(connected()&&state.data?.ros?.available===true&&state.data?.ros?.listen_ready===true);
    document.querySelectorAll('[data-target]').forEach(btn=>{const issue=targetSelectionReason();btn.disabled=!ready||btn.dataset.selectable!=='true'||!!issue;btn.title=issue||(btn.dataset.selectable!=='true'?'目标尚未稳定':'选择此火焰；条件满足时自动瞄准，喷水需另外下令');});
    const roundIssue=selectedRoundReason();$('extinguishSelectedBtn').disabled=!ready||!!roundIssue;$('extinguishSelectedBtn').title=roundIssue||'执行一轮自动瞄准、短喷、关水';text('selectedRoundHint',roundIssue||'已具备提交条件，点击后确认选定目标。');
    $('sprayBtn').disabled=!ready||!selectionIdentity()||!pumpReady();$('sprayBtn').title=pumpReady()?'提交短时喷水请求':'水泵与云台尚未启用真实输出';
    document.querySelectorAll('[data-command="自动找火灭火"]').forEach(btn=>{btn.disabled=!ready||!pumpReady();btn.title=pumpReady()?'请求自动灭火':'先连接并验收云台、水泵及停稳反馈';});
    $('streamToggle').disabled=!connected();
    for(const [form,capability] of [['extensionDistanceForm','distance'],['extensionTurnForm','route_turn'],['extensionFireForm','velocity']]){
      const gate=extensionMotionReadiness(capability),button=$(form).querySelector('button[type="submit"]');
      button.disabled=!ready||!gate.ready;button.title=gate.ready?'提交真实 FPGA 运动请求':reason(gate.reason);
      if(capability==='route_turn'&&data('extension').reference?.confirmed!==true){button.disabled=true;button.title='请先按实际位置设置人工起点；上一段移动后的旧参考不能复用';}
    }
    const referenceReady=extensionMotionReadiness('route_turn');$('extensionReferenceForm').querySelector('button[type="submit"]').disabled=!ready||!referenceReady.ready||data('extension').vehicle_stopped!==true;
    const trafficGate=trafficDriveReadiness();document.querySelectorAll('[data-traffic-drive]').forEach(button=>{button.disabled=!ready||!trafficGate.ready;button.title=trafficGate.ready?'开始真实交通行驶，限速由 FPGA 确认':reason(trafficGate.reason);});
    const zebraGate=zebraSwitchReadiness();document.querySelectorAll('[data-zebra]').forEach(button=>{button.disabled=!ready||!zebraGate.ready||data('extension').zebra_detection_enabled===(button.dataset.zebra==='on');button.title=zebraGate.ready?'提交设置并等待 FPGA 实际回读':reason(zebraGate.reason);});
    text('commandHint',!connected()?'尚未连接实时车载服务，指令不会发送。':!ready?'控制暂不可用：'+reason(state.data?.control?.read_only_reason):'指令由 X5 任务节点执行；任务进度与完成状态来自车载反馈。');
    if(data('voice').audio_test_active===true){text('micState','正在收声测试');text('micNote','测试结果在“收声测试”窗口中显示，不会下发小车任务。');}
    renderAudioTest();renderPointControls();
  }

  function dialogueInputReady() {
    const e=data('extension');
    return connected()&&fresh('extension')&&(['idle','standby'].includes(e.task)||(['vision','dialogue','semantic'].includes(e.task)&&e.dialogue_enabled===true));
  }
  function validPointId(value) { return Number.isInteger(value)&&value>=1&&value<=9; }
  function pointStartReference() {
    const e=data('extension');
    if(!connected()||!fresh('extension')||e.real_uart!==true||e.protocol_ready!==true||e.status_fresh!==true)return null;
    const reference=e.point_start_reference;
    if(reference&&typeof reference==='object'){
      if(reference.valid!==true)return null;
      if(reference.kind==='numbered'&&validPointId(reference.point_id))return reference;
      if(reference.kind==='map'&&Array.isArray(reference.cell)&&reference.cell.length===2&&reference.cell.every(Number.isInteger)&&reference.cell[0]>=0&&reference.cell[0]<176&&reference.cell[1]>=0&&reference.cell[1]<160&&Number.isFinite(reference.heading_deg)&&reference.heading_deg>=-180&&reference.heading_deg<=180)return reference;
      return null;
    }
    const old=e.point_reference;
    return old?.confirmed===true&&validPointId(old.point_id)?{...old,valid:true,kind:'numbered'}:null;
  }
  function pointNavigationReadiness(confirm=false) {
    const e=data('extension');
    if(!connected()||!fresh('extension')||e.real_uart!==true||e.protocol_ready!==true||e.status_fresh!==true)
      return {ready:false,reason:'extension_protocol_not_ready'};
    if(confirm){
      if(e.can_confirm_point_start!==true||e.vehicle_stopped!==true)return {ready:false,reason:e.point_navigation_reason||'measured_stop_required'};
      return {ready:true,reason:''};
    }
    if(!['dialogue','semantic'].includes(e.task))return {ready:false,reason:'select_extension2_for_numbered_navigation'};
    if(!pointStartReference())return {ready:false,reason:'point_reference_required'};
    if(e.can_navigate_point!==true)return {ready:false,reason:e.point_navigation_reason||'numbered_navigation_not_ready'};
    return {ready:true,reason:''};
  }
  function numberedTaskState() {
    const n=data('navigation');
    if(n.motion_kind==='numbered_point')return n;
    const value=data('extension').numbered_navigation;
    return value&&typeof value==='object'?value:{};
  }
  function pointRequestPayload(action) {
    if(action==='confirm_point_start'){
      const point=Number($('pointStartSelect').value);
      if(!validPointId(point))throw new Error('请选择车头当前实际所在的 1～9 号点。');
      if(!$('pointPlacementCheck').checked)throw new Error('请先核对车辆实际位置和 FPGA 起始朝向，再勾选确认。');
      const raw=$('pointStartHeading').value.trim(),heading=Number(raw);
      if(!raw||!Number.isFinite(heading)||heading< -180||heading>180)throw new Error('请填写实际车头方向：0°向右、90°向上、180°向左、-90°向下。');
      return {action,point_id:point,confirmed:true,start_heading_deg:heading};
    }
    if(action==='navigate_point'){
      if(!validPointId(state.pointTarget))throw new Error('请先选择 1～9 号目标点。');
      const ref=pointStartReference();
      if(!ref)throw new Error('没有新鲜且已确认的实际起点。');
      return ref.kind==='numbered'?{action,target_point:state.pointTarget,start_point:ref.point_id}:
        {action,target_point:state.pointTarget};
    }
    throw new Error('未知编号点操作。');
  }
  async function sendPointAction(action) {
    if(!canControl()){toast('尚未连接可用的实车任务服务，未下发。',true);return;}
    const gate=pointNavigationReadiness(action==='confirm_point_start');
    if(!gate.ready){toast(reason(gate.reason),true);return;}
    let payload;
    try{payload=pointRequestPayload(action);}catch(error){toast(error.message,true);return;}
    state.pending=true;renderControls();
    try{
      const result=await request('/api/point-navigation',{...payload,request_id:uuid()});
      const sentence=action==='confirm_point_start'?'确认实际起点为 '+payload.point_id+' 号点':validPointId(payload.start_point)?'从 '+payload.start_point+' 号点到 '+payload.target_point+' 号点':'从已确认当前位置前往 '+payload.target_point+' 号点';
      state.localMessages.push({text:result.text||sentence,at:result.queued_at||Date.now()/1000,
        id:result.command_id||result.request_id||uuid(),action,status:result.status||'queued'});
      state.localMessages=state.localMessages.slice(-25);
      if(action==='confirm_point_start')$('pointPlacementCheck').checked=false;
      toast(action==='confirm_point_start'?'起点确认请求已提交，等待车载回执。':'编号点导航已提交；到达并停稳后开启 YOLO，以车载反馈为准。');
      renderDialogue();
    }catch(error){toast('请求未确认：'+error.message+'；不会自动重发。',true);}
    finally{state.pending=false;renderPointNavigation();renderControls();}
  }
  function renderPointControls() {
    const confirm=pointNavigationReadiness(true),nav=pointNavigationReadiness(false),ready=canControl();
    if(!confirm.ready)$('pointPlacementCheck').checked=false;
    $('pointConfirmStart').disabled=!ready||!confirm.ready||!validPointId(Number($('pointStartSelect').value))||!$('pointPlacementCheck').checked;
    $('pointConfirmStart').title=confirm.ready?'提交人工摆放确认；等待车载状态确认后才能导航':reason(confirm.reason);
    $('pointNavigate').disabled=!ready||!nav.ready||!validPointId(state.pointTarget);
    $('pointNavigate').title=nav.ready?'请求 FPGA 规划与执行，到达后 X5 开启 YOLO 观察':reason(nav.reason);
    text('pointNavigationGate',!confirm.ready?reason(confirm.reason):!nav.ready?reason(nav.reason):'起点已确认；选择目标后下达真实导航任务。');
    const region=regionSearchReadiness();
    $('regionSearchStart').disabled=!ready||!region.ready||!['A','B','C','D'].includes(state.regionTarget);
    $('regionSearchStart').textContent='搜索消防员';
    $('regionSearchStart').title=region.ready?'稳定确认消防员后缓慢前后对中，停稳观察后继续原路线；不触发喷水':regionReason(region.reason);
    renderRegionFireReadiness(region);
  }
  function regionSearchReadiness() {
    if(!connected()||!fresh('extension'))return {ready:false,reason:'extension_status_unavailable'};
    const e=data('extension');
    return {ready:e.region_observe_ready===true,
      reason:e.region_observe_reason||'region_search_not_ready'};
  }
  function regionSearchCommand() {
    if(!['A','B','C','D'].includes(state.regionTarget))return '';
    return '在'+state.regionTarget+'区域搜索消防员';
  }
  function regionSearchState() {
    const value=data('extension').region_search;
    return value&&typeof value==='object'?value:{};
  }
  function regionSprayConfirmed(row) {
    const attempt=row?.spray_attempt||{},proof=row?.spray_evidence||{};
    return row?.spray_confirmed===true&&row.status==='completed'&&
      typeof attempt.run_id==='string'&&attempt.run_id.length>0&&proof.run_id===attempt.run_id&&
      proof.registry_session===row.registry_session&&proof.target_id===row.target_id&&
      attempt.registry_session===row.registry_session&&attempt.target_id===row.target_id&&
      proof.saw_pump_on===true&&finite(attempt.started_at)&&finite(proof.confirmed_pump_off_at)&&
      finite(proof.auto_completion_state_stamp)&&finite(proof.confirmed_disarmed_stamp)&&
      attempt.started_at<=proof.confirmed_pump_off_at&&
      proof.confirmed_pump_off_at<=proof.auto_completion_state_stamp&&
      proof.auto_completion_state_stamp<=proof.confirmed_disarmed_stamp;
  }
  function regionTargetOutcome(r) {
    const queue=(Array.isArray(r.centering?.queue)?r.centering.queue:[]).filter(row=>Number.isInteger(row?.target_id)&&row.target_id>0);
    const key=row=>JSON.stringify([row.registry_session,row.target_id]),unresolved=new Map();
    for(const row of Array.isArray(r.skipped_observations)?r.skipped_observations:[])
      if(Number.isInteger(row?.target_id)&&row.target_id>0)unresolved.set(key(row),row);
    for(const row of queue){
      if(!['skipped','lost','failed'].includes(row.status))continue;
      for(const alias of [...(Array.isArray(row.observed_id_aliases)?row.observed_id_aliases:[]),row.original_identity])
        if(Array.isArray(alias)&&alias.length===2)unresolved.delete(JSON.stringify(alias));
      unresolved.set(key(row),row);
    }
    const pending=queue.filter(row=>!['completed','skipped','lost','failed'].includes(row.status));
    const complete=queue.filter(row=>row.status==='completed').length,missing=[...unresolved.values()],parts=[];
    if(queue.length)parts.push('本批消防员居中观察 '+complete+'/'+queue.length);
    if(missing.length)parts.push('未完成观察 '+missing.length+' 项');
    if(pending.length)parts.push('待观察 '+pending.length+' 项');
    const details=missing.slice(-3).map(row=>'消防员 #'+row.target_id+'：'+regionReason(row.reason||'target_not_processed'));
    return {queue,pending,unresolved:missing,complete,summary:parts.join(' · '),detail:details.join('；')};
  }
  function regionReason(code) {
    return reason(code).replaceAll('火焰','消防员').replaceAll('火源','消防员').replaceAll('每团','每位').replaceAll('下一团','下一位');
  }
  function regionTaskPresentation(r) {
    const phase=String(r.phase||'IDLE').toUpperCase();
    const terminal=['COMPLETE','COMPLETED','FAILED','FAULT','CANCELLED'].includes(phase);
    const failed=['FAILED','FAULT'].includes(phase);
    const generic=new Set(['region_navigation_failed','region_task_failed','navigation_failed','failed','fault']);
    const failure=terminal&&r.failure_detail&&r.failure_detail.command_id===r.command_id?r.failure_detail:{};
    const candidates=[failure.reason,failure.navigation_reason,r.reason,r.navigation?.reason,r.navigation?.centering?.reason,r.centering?.reason];
    const code=candidates.find(value=>typeof value==='string'&&value&&!generic.has(value))||r.reason||'';
    const labels={IDLE:'等待区域任务',ENTERING:'前往搜索区域',NAVIGATING:'沿路线搜索消防员',DRIVING:'沿路线搜索消防员',EXITING:'驶向出口 · 继续检测',
      WAIT_STOP:'发现消防员 · 等待实际停稳',PAUSING:'发现消防员 · 等待实际停稳',STOPPING:'等待实际停稳',
      CENTER_COLLECT:'停稳后确认本批消防员',CHECKPOINT_COLLECT:'停稳后确认本批消防员',CENTER_SELECT:'选择下一位消防员',ALIGN_SELECT:'选择下一位消防员',
      CENTER_MOVING:'前后微调对中',ALIGNING:'微调对中',CENTER_STOP:'对中后等待实际停稳',ALIGN_STOP:'对中后等待实际停稳',
      CENTER_DWELL:'已居中 · 停稳观察',ALIGN_DWELL:'已居中 · 停稳观察',CENTER_REACQUIRE:'返回原观察位重找目标',CENTER_RETURN:'返回原路线暂停位置',
      WAIT_GEOMETRY:'等待三维定位',WAIT_SPRAY_RANGE:'核对定位与喷水距离',APPROACHING:'沿道路靠近目标',
      EXTINGUISHING:'云台瞄准与短喷',FIRING:'云台瞄准与短喷',ALIGN_FIRE:'云台瞄准与短喷',WAIT_PUMP_OFF:'等待实际关水',RESUMING:'恢复原路线行驶',
      SEARCHING:'观察消防员',OBSERVING:'观察消防员',COMPLETE:'区域任务已完成',COMPLETED:'区域任务已完成',FAILED:'区域任务已中止',FAULT:'区域任务已中止',CANCELLED:'区域任务已取消'};
    if(r.execute_fire===true&&r.centering?.gimbal_only===true)Object.assign(labels,{
      CENTER_DWELL:'停车确认目标 · 准备云台瞄准',CENTER_SELECT:'车辆保持停稳 · 选择下一消防员',
      EXTINGUISHING:'车辆保持停稳 · 云台瞄准与短喷',WAIT_PUMP_OFF:'确认本目标关水 · 本批尚未结束'});
    const hold=terminal?'':searchPumpRecoveryText(r);
    const identityFailure=failure.navigation_identity_failure||r.navigation?.identity_failure||r.navigation?.centering?.pose_failure;
    const poseFailure=['centering_same_route_pose_lost','continuous_route_or_pose_identity_lost','continuous_paused_route_or_pose_changed'].includes(code)&&(!failure.leg_id||failure.leg_id===r.navigation?.command_id)?identityFailure?.reason:'';
    const noFireStop=failed&&Object.prototype.hasOwnProperty.call(r,'last_fire_trigger')&&!r.last_fire_trigger;
    const detail=regionReason(code)+(typeof poseFailure==='string'&&poseFailure&&poseFailure!==code?'；具体原因：'+regionReason(poseFailure):'')+(noFireStop?'；本轮未触发消防员停车':'');
    const outcome=regionTargetOutcome(r),unfinished=outcome.unresolved.length+outcome.pending.length;
    const completed=['COMPLETE','COMPLETED'].includes(phase);
    return {phase,terminal,failed,active:r.active===true&&!terminal,
      title:hold?'核对停车与关水 · 任务保留':completed&&unfinished?'路线已结束 · 仍有目标未处置':labels[phase]||reason(phase),
      detail:hold||detail+(terminal&&outcome.summary?'；'+outcome.summary:''),reason:code,
      state:failed||completed&&unfinished?'warn':r.active===true&&!terminal?'sel':completed?'ok':'idle'};
  }
  function isCurrentRegionTask(e) {
    const r=e.region_search||{};
    if(!r.command_id||!['dialogue','semantic'].includes(e.task))return false;
    if(r.active===true)return true;
    const active=e.active||{};
    if(active.command_id&&active.command_id!==r.command_id&&!['completed','fault','failed','cancelled'].includes(active.phase))return false;
    return !(e.last_result?.command_id&&e.last_result.command_id!==r.command_id&&e.last_result.accepted===true);
  }
  function measuredVehicleMotion(e,extensionFresh,m,missionFresh) {
    // Navigation mode is a permission; wheel feedback is the measured fact.
    // Never replace a fresh FPGA stop/motion report with the inspect-mode policy.
    if(e.real_uart===true){
      if(extensionFresh&&e.status_fresh===true&&typeof e.vehicle_stopped==='boolean')
        return {text:e.vehicle_stopped?'实测已停稳':'实测未停稳',state:e.vehicle_stopped?'ok':'sel',source:'FPGA 车轮反馈'};
      return {text:'停车状态未知',state:'warn',source:'等待新鲜 FPGA 车轮反馈'};
    }
    if(missionFresh&&typeof m.vehicle_stopped==='boolean')
      return {text:m.vehicle_stopped?'已反馈停稳':'等待停稳反馈',state:m.vehicle_stopped?'ok':'idle',source:'任务节点反馈'};
    return {text:'停车状态未知',state:'idle',source:'等待实际运动反馈'};
  }
  function regionRouteSummary(r,physical) {
    if(!r.command_id)return '一次规划全部途经点，全程检测消防员；稳定确认后缓慢前后对中、逐个停稳观察，再返回原路线继续，出口停车。';
    if(!physical)return '等待新鲜区域规划反馈；不显示旧访问顺序。';
    const ids=(Array.isArray(r.waypoints)?r.waypoints:[]).map(p=>p?.id).filter(id=>typeof id==='string'&&/^[ABCD][1-4]$/.test(id));
    const visited=new Set(Array.isArray(r.passed_waypoint_ids)?r.passed_waypoint_ids:Array.isArray(r.visited_waypoint_ids)?r.visited_waypoint_ids:[]);
    const order=ids.map(id=>(visited.has(id)?'✓ ':'')+id);
    if(r.exit?.cell)order.push(r.exit.confirmed_stopped===true?'出口已停稳':'出口');
    const policy=r.route_policy||{},details=[];
    const download=r.navigation?.route_download;
    if(r.continuous_route===true&&download?.active===true&&Number.isInteger(download.received)&&
        Number.isInteger(download.total)&&download.total>0&&download.total<=2048&&download.received>=0&&download.received<=download.total)
      details.push('完整路线回读 '+download.received+'/'+download.total+' 点，完成后出发');
    if(policy.continuous_route===true)details.push('整条路线一次规划');
    if(policy.continuous_route===true&&r.navigation?.retained_route===true)details.push('途经点不停 · 仅确认消防员后暂停，再沿原路线续行');
    if(['directed_graph_region_first','directed_graph_region_preferred'].includes(policy.strategy))details.push('按道路与朝向规划 · 区域内优先');
    if(Number.isFinite(policy.estimated_distance_cm))details.push('预计 '+Math.round(policy.estimated_distance_cm)+' cm');
    if(policy.fallback_reason)details.push('需借道：'+reason(policy.fallback_reason));
    return (order.length?'途经顺序：'+order.join(' → '):'等待实际路线排序')+(details.length?' · '+details.join(' · '):'');
  }
  function searchPumpRecoveryText(r) {
    const hold=r?.pump_readback_recovery;
    return r?.active===true&&!['COMPLETE','COMPLETED','FAILED','FAULT','CANCELLED'].includes(String(r.phase||'').toUpperCase())&&r.execute_fire===false&&hold?.active===true?
      (hold.reason==='search_pump_recovery_waiting_fresh_stop_and_low'?'核对停车与关水回读':'核对关水回读')+
      '，暂缓运动；任务与原路线保留，确认后自动继续':'';
  }
  function checkpointAlignmentView(r,physical) {
    const view=regionTaskPresentation(r),outcome=regionTargetOutcome(r);
    if(!physical)return {rule:'等待车载消防员检测配置与新鲜反馈',
      progress:r.command_id?'上次任务记录：'+view.title+(view.detail?' · '+view.detail:''):'等待实时任务状态。',
      queue:r.command_id?[outcome.summary,outcome.detail,'实时停车、目标位置和运动状态尚未确认。'].filter(Boolean).join('；'):'',state:'idle'};
    const a=r.checkpoint_alignment||{},c=r.centering||{},trigger=r.firefighter_stop_trigger||r.person_stop_trigger||r.fire_stop_trigger||{};
    const score=(r.firefighter_statistics||r.person_statistics||r.fire_statistics)?.min_confidence??a.min_confidence;
    const confidence=finite(score)&&score>=.9&&score<=1?'置信度 ≥'+Math.round(score*100)+'% · ':'';
    const frames=Number.isInteger(trigger.confirm_frames)?trigger.confirm_frames:null,duration=trigger.stable_duration_s;
    const stopRule=frames&&finite(duration)?'稳定 '+frames+' 帧 / '+duration.toFixed(2)+' 秒才停车':'稳定消防员才停车';
    const phases={IDLE:'等待区域任务：途经点不停',NAVIGATING:'沿路线搜索消防员',ENTERING:'前往区域，全程检测消防员',EXITING:'驶向出口，继续检测消防员',
      WAIT_STOP:'已确认消防员，等待实际停稳',CENTER_COLLECT:'多帧确认画面两侧的消防员',CENTER_SELECT:'选择下一位消防员',
      CENTER_MOVING:'缓慢前后微调，让当前消防员对齐中线',CENTER_STOP:'等待车辆实际停稳',CENTER_DWELL:'已停稳居中，连续观察 1 秒',
      CENTER_REACQUIRE:'返回原观察位置重找队列消防员',CENTER_RETURN:'返回原路线暂停位置',COMPLETE:'路线已完成，出口已停稳',FAILED:'任务已停止，请查看原因',CANCELLED:'任务已取消',
      CHECKPOINT_COLLECT:'检测点多帧确认消防员',ALIGN_SELECT:'选择下一位消防员',ALIGNING:'缓慢前进对齐',ALIGN_STOP:'等待实际停稳',ALIGN_DWELL:'停稳观察',DONE:'本检测点已处理'};
    const holdText=searchPumpRecoveryText(r),details=[holdText||phases[r.phase]||phases[a.phase]||'等待区域巡检状态'];
    if(r.navigation?.centering?.map_limits_enabled===false)details.push('微调地图限制已关闭 · 回原暂停位置后续行');
    if(!holdText&&r.active===true&&['NAVIGATING','ENTERING','EXITING'].includes(r.phase)&&typeof trigger.reason==='string'){
      details.push(regionReason(trigger.reason));
      if(Number.isInteger(trigger.hits)&&frames)details.push('稳定帧 '+Math.min(trigger.hits,frames)+'/'+frames);
    }
    if(r.active===true&&Number.isInteger(c.current_target_id))details.push('当前消防员 #'+c.current_target_id);
    if(!holdText&&r.active===true&&r.phase==='CENTER_MOVING'&&['forward','reverse'].includes(c.move_direction))details.push(c.move_direction==='forward'?'缓慢前进对齐':'缓慢后退对齐');
    if(!holdText&&r.active===true&&r.phase==='CENTER_DWELL'&&finite(c.dwell_remaining_s)&&c.dwell_remaining_s>=0&&c.dwell_remaining_s<=1)details.push('停留剩余 '+c.dwell_remaining_s.toFixed(1)+' 秒');
    const remaining=(r.firefighter_retrigger||r.person_retrigger||r.fire_retrigger)?.cooldown_remaining_s;
    if(r.active===true&&finite(remaining)&&remaining>0&&remaining<=30)details.push('新目标停车冷却 '+remaining.toFixed(1)+' 秒');
    const statuses={candidate:'待多帧确认',queued:'待对齐',aligning:'对齐中',centered:'停稳观察',completed:'居中观察完成',lost:'已丢失 · 未完成',skipped:'已跳过 · 未完成',failed:'未完成'};
    const queue=(Array.isArray(c.queue)?c.queue:Array.isArray(a.queue)?a.queue:[]).filter(item=>Number.isInteger(item?.target_id)&&item.target_id>0);
    const queueText=queue.length?(view.terminal?'本轮记录：':'本批：')+queue.slice(0,12).map(item=>'消防员 #'+item.target_id+' '+(view.terminal&&!['completed','skipped','lost'].includes(item.status)?'未完成（任务已结束）':statuses[item.status]||'待回报')).join(' → ')+'。':'';
    const tolerance=c.center_tolerance_norm??a.center_tolerance_norm,band=finite(tolerance)&&tolerance>=.01&&tolerance<=.08?'±'+Number((tolerance*100).toFixed(1))+'%':'等待车端容差';
    return {rule:'全幅消防员检测 · '+confidence+stopRule+' · 缓慢前后对中 '+band+' · 居中停 1 秒',
      progress:view.terminal?view.title+(view.detail?' · '+view.detail:''):details.join(' · '),
      queue:[queueText,outcome.summary,outcome.detail,'本批观察后返回原路线；消防员任务不控制云台或喷水。'].filter(Boolean).join(' '),state:view.state};
  }
  function renderRegionSearch() {
    const e=data('extension'),r=regionSearchState(),physical=fresh('extension')&&e.real_uart===true&&e.status_fresh===true;
    const view=regionTaskPresentation(r);
    text('regionPhase',!r.command_id?'等待消防员区域任务':view.terminal&&fresh('extension')?view.title:!physical?'反馈过期 · 等待车载状态':view.title);
    const visited=Number.isInteger(r.passed_waypoint_count)?r.passed_waypoint_count:Array.isArray(r.visited_waypoint_ids)?r.visited_waypoint_ids.length:null;
    const pieces=[];
    if(physical&&r.command_id){
      if(['A','B','C','D'].includes(r.region_id))pieces.push(r.region_id+' 区 · 消防员搜索');
      if(Number.isInteger(visited)&&Number.isInteger(r.waypoints_total))pieces.push('途经点 '+visited+'/'+r.waypoints_total);
      if(view.active&&r.current_waypoint_id)pieces.push('下一途经点 '+String(r.current_waypoint_id));
      if(view.detail)pieces.push(view.detail);
      if(r.visual_degraded===true)pieces.push('视觉暂缺，地图导航继续；恢复新帧后重新确认消防员');
      if(r.visual_coverage_incomplete===true)pieces.push('部分路段视觉曾中断，路线完成不代表全程无人');
    }
    const gate=regionSearchReadiness();
    text('regionFireStatistics',regionFireStatisticsText(r,physical));text('regionRoutePlan',regionRouteSummary(r,physical));
    const alignment=checkpointAlignmentView(r,physical);
    text('regionAlignmentRule',alignment.rule);text('regionAlignmentProgress',alignment.progress);text('regionAlignmentQueue',alignment.queue);
    $('regionAlignment').dataset.state=alignment.state;
    if(physical&&r.command_id&&['CANCELLED','FAILED','FAULT'].includes(String(r.phase||'').toUpperCase()))pieces.push(!gate.ready?'下一步：'+regionReason(gate.reason):'处理原因后重新下达区域指令；不会自动重发');
    text('regionProgress',!physical?'等待 FPGA 新鲜反馈，暂不确认任务进度。':pieces.join(' · ')||(!gate.ready?regionReason(gate.reason):'设置起点与车头方向，再下达“去 C 区域搜索消防员”；确认消防员后缓慢对中观察，再返回原路线继续。'));
    document.querySelectorAll('[data-region]').forEach(el=>el.setAttribute('aria-pressed',String(el.dataset.region===state.regionTarget)));
  }
  function regionFireStatisticsText(r,physical) {
    const s=r.firefighter_statistics||r.person_statistics||r.fire_statistics;
    if(!r.command_id||!s)return '消防员统计：等待区域搜索任务';
    const count=value=>Number.isInteger(value)&&value>=0?String(value):'—';
    const complete=(Array.isArray(r.centering?.queue)?r.centering.queue:[]).filter(row=>row?.status==='completed').length;
    return (physical?'':'历史统计 · ')+'当前可见消防员 '+(physical&&s.current_fresh===true?count(s.current_visible):'—')+
      ' · 累计跟踪记录 '+count(s.tracked_observations_count)+' · 本批已观察 '+complete+(s.truncated?' · 记录已达上限':'')+
      '。跟踪记录可能包含重复身份，不等于真实人数。';
  }
  function fireSetupItems() {
    const a=data('actuators'),m=data('mission'),p=data('fire_proposal'),s=data('stereo');
    const angles=a.actual_angles_deg,feedback=fresh('actuators')&&a.status_fresh===true&&a.feedback_fresh===true&&Array.isArray(angles)&&angles.length===2&&angles.every(finite);
    return [
      ['车辆停稳',fresh('mission')&&m.vehicle_stopped===true,'等待 FPGA 的实际停稳反馈'],
      ['真实云台程序',fresh('actuators')&&a.dry_run===false&&a.enabled===true,'等待真实云台控制与串口连接；dry-run 不会转动电机'],
      ['双轴编码器与零位',feedback&&a.reference_confirmed_this_start===true,'需要有效角度与零位参考；恢复编码器坐标不等于已经机械回零'],
      ['双目米制测距',fresh('stereo')&&s.gate==='ready'&&s.control_eligible===true&&s.observation_only!==true,'确认左右眼、曝光同步，并用已知距离验收双目标定；厂家参数观测不作为自动瞄准距离'],
      ['固定相机到喷嘴标定',fresh('fire_proposal')&&fresh('mission')&&p.calibration_ready===true&&m.mounting_verified===true,'安装尺寸只是其中一项，还需要光学方向、双目数据与电机参考对应'],
      ['目标水流补偿',fresh('fire_proposal')&&p.water_impact_calibration_verified===true&&p.landing_compensated===true&&Boolean(p.landing_calibration_id),'选定目标后计算实测水流补偿；未选目标时本项不代表已有落点表无效'],
      ['FPGA 水泵输出',pumpReady(),pumpReadinessDetail()]
    ];
  }
  function pumpReadinessDetail() {
    if(!fresh('actuators'))return '等待云台与水泵节点的新鲜状态';
    const a=data('actuators'),e=data('extension');
    if(a.pump_backend!=='fpga_uart'||a.pump_pin!=='B13')return '当前水泵后端未对应 FPGA UART / B13';
    if(fresh('extension')&&e.capabilities?.pump_b13!==true)return reason('region_pump_b13_firmware_required');
    const output=a.pump_enabled===true?'自动喷水配置已启用':'自动喷水配置未启用';
    const feedback=a.pump_feedback_fresh===true?(a.pump_on===true?'B13 高电平':a.pump_on===false?'B13 低电平':'B13 电平未知'):'尚无新鲜 B13 反馈';
    return output+'；'+feedback+(a.pump_reason?'；'+reason(a.pump_reason):'');
  }
  function renderRegionFireReadiness(gate) {
    const panel=$('regionFireReadiness');
    if(!panel)return;
    panel.hidden=true;
    if(panel.hidden)return;
    const active=fresh('extension')&&regionSearchState().active===true;
    text('regionFireReadinessSummary',active?'搜索并灭火执行状态':gate.ready?'搜索并灭火：任务入口已就绪':'搜索并灭火：待完成条件');
    text('regionFireReadinessReason',active?'任务逐个处理火焰；实际关水确认后才进入下一动作。':gate.ready?'任务可提交；停车后的每次瞄准与喷水仍以实时反馈为准。':'当前拦截原因：'+reason(gate.reason));
    const items=fireSetupItems();
    html('regionFireReadinessItems',items.map(([label,ready,hint])=>`<div class="${ready?'ready':'pending'}"><strong>${ready?'已确认':'待确认'} · ${esc(label)}</strong><span>${esc(ready?'已收到新鲜状态':hint)}</span></div>`).join(''));
  }
  function renderFireSetup() {
    const items=fireSetupItems();
    $('fireSetupChecks').replaceChildren(...items.map(([label,ready,hint])=>{
      const row=document.createElement('div'),title=document.createElement('strong'),detail=document.createElement('span');
      row.className=ready?'ready':'pending';title.textContent=(ready?'已就绪 · ':'待完成 · ')+label;detail.textContent=ready?'已收到当前状态确认':hint;row.append(title,detail);return row;
    }));
    const pending=items.find(item=>!item[1]);
    text('fireSetupNext',!connected()?'当前未连接小车。连接后显示实际检查结果。':pending?'下一步：'+pending[2]:'准备项已满足，返回选择火焰后执行一轮。');
  }
  function fireRoundProgress(value) {
    if(!value?.command_id)return '';
    const phases={SEARCH:'确认选定火焰',WAIT_READY:'等待观察条件',AIM:'双轴瞄准 · 等待实际角度到位',
      BURST:'执行有限短喷 · 等待真实开关水反馈',REVIEW:'关水后复查目标',REPORT:'汇总本轮结果',
      COMPLETE:'本轮已结束',CANCELLED:'本轮已取消',WAIT_SELECTION:'等待选择火焰'};
    return (phases[value.phase]||reason(value.phase))+(value.reason?' · '+reason(value.reason):'');
  }
  function renderPointNavigation() {
    const e=data('extension'),ref=pointStartReference(),n=numberedTaskState(),confirmed=Boolean(ref);
    const mapReference=ref?.kind==='map';
    text('pointReferenceBadge',confirmed?(mapReference?'地图起点':ref.point_id+' 号点'):'未确认');
    $('pointReferenceBadge').dataset.state=confirmed?'ok':'idle';
    text('pointReferenceState',mapReference?(ref.source==='fpga_encoder_imu_pose'?'由停稳后的惯导位置提供起点':'地图位置与方向已获车载确认')+'：图上格中心 X '+(ref.cell[0]*2+1)+' / Y '+(ref.cell[1]*2+1)+' cm · 朝向 '+ref.heading_deg+'°。':confirmed?(ref.source==='confirmed_arrival'?'由本轮 FPGA 到达反馈确认':'人工位置已获车载确认')+'：'+ref.point_id+' 号点'+(Number.isFinite(ref.start_heading_deg)?' · 朝向 '+ref.start_heading_deg+'°':' · 朝向以新鲜惯导反馈为准')+'。':'尚未确认起点。可在地图上设置实际位置与方向，或在这里确认编号起点；选择目标不等于定位。');
    text('pointTargetSummary',validPointId(state.pointTarget)?'待提交目标：'+state.pointTarget+' 号点':'尚未选择目标');
    document.querySelectorAll('[data-point-target]').forEach(button=>button.setAttribute('aria-pressed',String(Number(button.dataset.pointTarget)===state.pointTarget)));
    const phases={setting_heading:'设置实际车头朝向',idle:'等待任务',setting_start:'设置已确认起点',running:'FPGA 正在导航',confirming_arrival:'收到到达事件，等待实际停稳',waiting_stop:'等待实际停稳',waiting_idle:'等待空闲与停稳',waiting_mode:'切换拓展二',waiting_mode_stop:'确认模式与停稳',waiting_ack:'等待指令回执',selecting_point:'选择编号目标',waiting_point_ack:'等待选点回执',planning:'FPGA 路径规划中',waiting_point_ready:'等待规划就绪',waiting_route_ready:'等待规划就绪',ready:'路线就绪',starting:'请求开始行驶',waiting_start_ack:'等待启动回执',navigating:'FPGA 正在导航',moving:'FPGA 正在导航',arrived:'已反馈到达',scanning:'到点开启 YOLO 观察',arrival_scan:'到点开启 YOLO 观察',finding_fire:'到点开启 YOLO 观察',completed:'本次任务完成',pausing:'发现火焰，正在暂停路线',paused:'路线已暂停 · 处理火焰',held:'已停车等待指令',resuming:'正在恢复原路线',stopping:'等待实际停稳',cancelled:'任务已取消',fault:'任务异常 / 已请求停车',failed:'任务失败'};
    const hasTask=Boolean(n.command_id)&&validPointId(n.target_point),phase=String(n.phase||n.state||'');
    text('pointRouteSummary',hasTask?(validPointId(n.start_point)?n.start_point+' 号点 → ':Array.isArray(n.start_cell)?'地图起点 → ':'目标 ')+n.target_point+' 号点':'等待编号点任务');
    const physicalFresh=fresh('extension')&&e.real_uart===true&&e.protocol_ready===true&&e.status_fresh===true;
    const verifiedArrival=physicalFresh&&n.valid===true&&n.arrived===true&&n.vehicle_stopped===true;
    const successLike=['arrived','completed'].includes(phase);
    text('pointPhase',!hasTask?'尚未下发':!physicalFresh?'反馈过期 · 等待车载状态':successLike&&!verifiedArrival?'等待到达与停稳证据':phases[phase]||'等待任务反馈');
    $('pointPhase').dataset.state=!hasTask?'idle':['failed','fault'].includes(phase)?'warn':verifiedArrival?'ok':'sel';
    text('pointReason',!hasTask?'编号点导航到达后等待单独指令；搜索消防员请使用 A～D 区域任务。':!physicalFresh?'没有新鲜的 FPGA 状态，不能确认当前位置或到达结果。':successLike&&!verifiedArrival?'等待 FPGA 到达事件与实际停稳确认。':n.reason?reason(n.reason):verifiedArrival?'FPGA 到达与停稳已确认；继续观察找火，喷水需另行指令。':'请求已提交；接受或启动回执不表示已经到达。');
    text('pointCommandId',hasTask?'任务 ID：'+n.command_id:'完成状态以 FPGA 回传为准。');
    renderRegionSearch();
  }
  document.querySelectorAll('[data-point-target]').forEach(button=>button.addEventListener('click',()=>{
    state.pointTarget=Number(button.dataset.pointTarget);renderPointNavigation();renderPointControls();
  }));
  $('pointStartSelect').addEventListener('change',()=>{$('pointPlacementCheck').checked=false;renderPointControls();});
  $('pointStartHeading').addEventListener('input',()=>{$('pointPlacementCheck').checked=false;renderPointControls();});
  $('pointPlacementCheck').addEventListener('change',renderPointControls);
  $('pointReferenceForm').addEventListener('submit',event=>{event.preventDefault();sendPointAction('confirm_point_start');});
  $('pointNavigate').addEventListener('click',()=>sendPointAction('navigate_point'));
  document.querySelectorAll('[data-region]').forEach(button=>button.addEventListener('click',()=>{
    state.regionTarget=button.dataset.region;
    $('commandText').value=regionSearchCommand();
    renderRegionSearch();renderPointControls();renderFpgaMap();
  }));
  $('regionSearchMode').addEventListener('change',()=>{
    state.regionMode='observe';
    if(state.regionTarget)$('commandText').value=regionSearchCommand();
    renderRegionSearch();renderPointControls();
  });
  $('regionSearchStart').addEventListener('click',()=>{
    if(!canControl()||!regionSearchReadiness().ready||!['A','B','C','D'].includes(state.regionTarget))return;
    sendCommand(regionSearchCommand());
  });

  // Read-only FPGA map telemetry. A selected point is never a measured pose.
  let mapData=null,mapReceived=0,mapStaticKey='';
  function mapSnapshot(payload,online,ageMs){
    const live=payload?.live||{},current=online&&ageMs>=0&&ageMs<2500&&live.supported===true&&live.fresh===true;
    const validPair=p=>Array.isArray(p)&&p.length===2&&p.every(finite);
    const pose=live.pose;
    return {pose:current&&pose?.valid===true&&pose.fresh===true&&[pose.x_cm,pose.y_cm,pose.heading_deg].every(finite)?pose:null,
      route:current&&live.route?.valid===true&&live.route.fresh===true&&Array.isArray(live.route.points_cm)&&live.route.points_cm.every(validPair)?live.route:null,
      barriers:current&&live.barriers?.valid===true&&live.barriers.fresh===true&&Array.isArray(live.barriers.segments_cm)&&live.barriers.segments_cm.every(p=>Array.isArray(p)&&p.length===4&&p.every(finite))?live.barriers:null};
  }
  function selectMapPoint(point){
    if(!validPointId(point))return;
    if(state.mapPickMode==='start'){
      const p=(mapData?.points||[]).find(p=>p.id===point);
      if(p&&finite(p.raster_cm?.x)&&finite(p.raster_cm?.y))selectMapStart(p.raster_cm.x,p.raster_cm.y);
      return;
    }
    state.pointTarget=point;$('commandText').value='去'+point+'号点';renderPointNavigation();renderPointControls();
    $('commandText').focus();toast('已选 '+point+' 号点。可直接说出指令，或发送输入框中的文字。');
  }
  function normalizedMapHeading(value){return ((value+180)%360+360)%360-180;}
  function validMapHeadingSample(sample){
    return sample&&Number.isInteger(sample.node)&&sample.node>=0&&Array.isArray(sample.cell)&&sample.cell.length===2&&sample.cell.every(Number.isInteger)&&sample.cell[0]>=0&&sample.cell[0]<176&&sample.cell[1]>=0&&sample.cell[1]<160&&Array.isArray(sample.xy_q8)&&sample.xy_q8.length===2&&sample.xy_q8.every(Number.isInteger)&&sample.xy_q8[0]>=0&&sample.xy_q8[0]<352*256&&sample.xy_q8[1]>=0&&sample.xy_q8[1]<320*256&&finite(sample.heading_deg)&&sample.heading_deg>=-180&&sample.heading_deg<=180;
  }
  function sameMapHeadingCatalog(previous,current){
    return previous&&Number.isInteger(current?.map_signature)&&previous.map_signature===current.map_signature&&
      (!previous.start_heading_samples_sha256||!current.start_heading_samples_sha256||previous.start_heading_samples_sha256===current.start_heading_samples_sha256);
  }
  function retainMapHeadingCatalog(previous,current){
    if(sameMapHeadingCatalog(previous,current)&&!Object.prototype.hasOwnProperty.call(current,'start_heading_samples')){
      return {...current,start_heading_samples:previous.start_heading_samples,
        start_heading_samples_sha256:current.start_heading_samples_sha256||previous.start_heading_samples_sha256};
    }
    return current;
  }
  function nearestMapStartHeading(x,y,samples){
    let best=null,distance=Infinity;
    const qx=Math.floor(x/2)*512+256,qy=Math.floor(y/2)*512+256;
    for(const sample of Array.isArray(samples)?samples:[]){
      if(!validMapHeadingSample(sample))continue;
      const candidate=Math.abs(sample.xy_q8[0]-qx)+Math.abs(sample.xy_q8[1]-qy);
      // Preserve ROM order when two directed-road samples are equally near.
      if(candidate<distance){distance=candidate;best=normalizedMapHeading(sample.heading_deg);}
    }
    return best;
  }
  function nextMapStart(previous,x,y,samples){
    const repeat=previous&&finite(previous.heading)&&Math.hypot(x-previous.x,y-previous.y)<=10;
    return repeat?{...previous,heading:normalizedMapHeading(previous.heading+90)}:
      {x:Math.round(x*100)/100,y:Math.round(y*100)/100,heading:nearestMapStartHeading(x,y,samples)};
  }
  function selectMapStart(x,y){
    const g=mapData?.geometry;
    if(!g||!finite(x)||!finite(y)||x<0||y<0||x>=g.width_cm||y>=g.height_cm)return;
    state.mapStart=nextMapStart(state.mapStart,x,y,mapData.start_heading_samples);
    renderFpgaMap();
  }
  function mapReferenceCommand(draft,height){
    if(!draft||![draft.x,draft.y,draft.heading,height].every(finite)||draft.x<0||draft.x>=352||draft.y<=0||draft.y>=height||draft.heading< -180||draft.heading>180)throw Error('请先在地图上选择实际起点和车头方向。');
    return '确认起点X'+draft.x.toFixed(2)+'厘米Y'+(height-draft.y).toFixed(2)+'厘米朝向'+draft.heading+'度';
  }
  function regionMapSvg(catalog){
    return ['A','B','C','D'].map(id=>{
      const r=catalog?.regions?.[id],b=r?.bounds_image_cm;
      if(!Array.isArray(b)||b.length!==4||!b.every(finite)||b[2]<=b[0]||b[3]<=b[1])return '';
      const points=(r.waypoints||[]).filter(p=>Array.isArray(p.image_cm)&&p.image_cm.length===2&&p.image_cm.every(finite));
      return `<g class="semantic-map-region" data-map-region="${id}"><rect x="${b[0]}" y="${b[1]}" width="${b[2]-b[0]}" height="${b[3]-b[1]}"/><text x="${b[0]+8}" y="${b[1]+14}">${id}</text>${points.map((p,i)=>`<g data-map-waypoint="${id}${i+1}" transform="translate(${p.image_cm[0]} ${p.image_cm[1]})"><circle r="3.5"/><text x="5" y="-5">${id}${i+1}</text></g>`).join('')}</g>`;
    }).join('');
  }
  function regionExitSvg(region,physical) {
    const cell=region?.exit?.cell;
    if(!physical||!region.command_id||!Array.isArray(cell)||cell.length!==2||!cell.every(Number.isInteger)||cell[0]<0||cell[0]>=176||cell[1]<0||cell[1]>=160)return '';
    return `<g class="region-exit ${region.exit.confirmed_stopped===true?'confirmed':''}" transform="translate(${cell[0]*2+1} ${cell[1]*2+1})"><path d="M0 -5 L5 0 L0 5 L-5 0 Z"/><text x="7" y="3">出口</text></g>`;
  }
  function mapStartReadiness(){
    const e=data('extension');
    if(!connected())return {ready:false,reason:'尚未连接 X5，请先恢复上位机连接。'};
    if(!fresh('extension'))return {ready:false,reason:'extension_status_unavailable'};
    // The extension node owns the real FPGA UART. The isolated legacy bridge
    // and an as-yet uninitialized map pose are not confirmation prerequisites.
    if(e.real_uart!==true||e.protocol_ready!==true||e.status_fresh!==true){
      const error=typeof e.error==='string'?e.error.trim():'';
      if(error)return {ready:false,reason:/write\s+timeout/i.test(error)?'FPGA 串口写入超时（Write timeout），尚未取得有效回包；请恢复串口通信后再确认起点。':'FPGA 串口通信异常：'+error};
      return {ready:false,reason:e.real_uart!==true?'real_extension_uart_required':'extension_firmware_or_feedback_unavailable'};
    }
    if(e.capabilities?.route_turn!==true)return {ready:false,reason:'fpga_capability_not_implemented_route_turn'};
    if(e.can_move!==true)return {ready:false,reason:e.reason&&e.reason!=='ready'?e.reason:e.busy?'extension_movement_in_progress':e.fpga_status?.mode_selected!==true?'fpga_physical_host_mode_required':'measured_stop_required'};
    if(e.vehicle_stopped!==true)return {ready:false,reason:'measured_stop_required'};
    if(!canControl())return {ready:false,reason:state.pending?'上一条指令正在提交，请稍候。':state.data?.control?.read_only_reason||'任务节点尚未就绪，请等待连接恢复。'};
    if(!Array.isArray(mapData?.start_heading_samples)||!mapData.start_heading_samples.some(validMapHeadingSample))return {ready:false,reason:'缺少道路朝向目录，请更新配套固件与地图目录后设置起点。'};
    if(state.mapPickMode!=='start'||!state.mapStart)return {ready:false,reason:'请点击“设置起点与方向”，再在地图上选择实际起点。'};
    try{mapReferenceCommand(state.mapStart,mapData?.geometry?.height_cm);}
    catch(error){return {ready:false,reason:error.message};}
    return {ready:true,reason:''};
  }
  function renderFpgaMap(){
    const value=mapData,g=value?.geometry;
    if(g&&finite(g.width_cm)&&finite(g.height_cm)&&g.width_cm>0&&g.height_cm>0&&g.width_cm<=2000&&g.height_cm<=2000){
      $('fpgaMap').setAttribute('viewBox',`0 0 ${g.width_cm} ${g.height_cm}`);
      const catalog=value.semantic_regions||data('extension').semantic_regions;
      const key=JSON.stringify([g,value.points,catalog]);
      if(key!==mapStaticKey){
        const runs=Array.isArray(g.road_runs)?g.road_runs:[];
        html('mapGeometry',runs.filter(r=>Array.isArray(r)&&r.length===3&&r.every(finite)&&r[0]>=0&&r[0]<g.height_cm&&r[1]>=0&&r[2]>0&&r[1]+r[2]<=g.width_cm).map(([y,x,n])=>`<rect x="${x}" y="${y}" width="${n}" height="1" fill="#363e47"/>`).join(''));
        html('mapRegions',regionMapSvg(catalog));
        const points=Array.isArray(value.points)?value.points:[];
        html('mapPoints',points.filter(p=>validPointId(p.id)&&finite(p.raster_cm?.x)&&finite(p.raster_cm?.y)).map(p=>`<g class="map-point" data-map-point="${p.id}" role="button" tabindex="0" aria-label="地图选择 ${p.id} 号点" transform="translate(${p.raster_cm.x} ${p.raster_cm.y})"><circle r="9"/><text>${p.id}</text></g>`).join(''));
        $('mapPoints').querySelectorAll('[data-map-point]').forEach(el=>{el.addEventListener('click',event=>{event.stopPropagation();selectMapPoint(Number(el.dataset.mapPoint));});el.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();selectMapPoint(Number(el.dataset.mapPoint));}});});
        mapStaticKey=key;
      }
      $('mapEmpty').classList.add('hidden');
    }else{$('mapEmpty').classList.remove('hidden');}
    const live=mapSnapshot(value,connected(),performance.now()-mapReceived);
    html('mapRoute',(live.route?`<polyline points="${live.route.points_cm.map(p=>p.join(',')).join(' ')}"/>`:'')+(live.barriers?live.barriers.segments_cm.map(p=>`<path d="M${p[0]} ${p[1]}L${p[2]} ${p[3]}" stroke="#FFB224" stroke-width="3"/>`).join(''):''));
    html('mapPose',live.pose?`<g transform="translate(${live.pose.x_cm} ${live.pose.y_cm}) rotate(${-live.pose.heading_deg})"><path d="M0 0 L-12 -5 L-9 0 L-12 5 Z"/></g>`:'');
    const draft=state.mapStart,editing=state.mapPickMode==='start';
    const headingCatalogReady=Array.isArray(value?.start_heading_samples)&&value.start_heading_samples.some(validMapHeadingSample);
    html('mapStart',editing&&draft?`<g transform="translate(${draft.x} ${draft.y})"><circle r="11"/>${finite(draft.heading)?`<path transform="rotate(${-draft.heading})" d="M8 0 L-7 -5 L-4 0 L-7 5 Z"/>`:''}</g>`:'');
    $('mapSetStart').setAttribute('aria-pressed',String(editing));
    const startGate=mapStartReadiness();
    $('mapConfirmStart').disabled=!startGate.ready;
    $('mapConfirmStart').title=startGate.ready?'提交实际起点与方向，等待车载确认；不会启动车辆。':reason(startGate.reason);
    const regionReference=data('extension').region_reference;
    const draftAccepted=editing&&draft&&fresh('extension')&&data('extension').status_fresh===true&&regionReference?.valid===true&&regionReference.source==='operator_hmi'&&regionReference.cell?.[0]===Math.floor(draft.x/2)&&regionReference.cell?.[1]===Math.floor(draft.y/2)&&finite(regionReference.heading_deg)&&Math.abs(normalizedMapHeading(regionReference.heading_deg-draft.heading))<.1;
    const startHint=!headingCatalogReady?'缺少道路朝向目录，请更新配套固件与地图目录后设置起点。':editing&&draft?`${draftAccepted?'起点已获车载确认':'待确认起点'} · 朝向 ${finite(draft.heading)?draft.heading+'°':'待重新选点'}；请按实车核对，再点箭头逆时针转 90°。`:'点击“设置起点”，按最近道路切线给出初始箭头；请按实车核对，再点起点逆时针转 90°。';
    text('mapStartHint',startGate.ready?startHint:reason(startGate.reason)+(reason(startGate.reason)===startHint?'':' '+startHint));
    $('mapRegions').querySelectorAll('[data-map-region]').forEach(el=>el.classList.toggle('selected',el.dataset.mapRegion===(regionSearchState().active?regionSearchState().region_id:state.regionTarget)));
    const searched=fresh('extension')&&data('extension').status_fresh===true?regionSearchState():{};
    html('mapExit',regionExitSvg(searched,connected()&&fresh('extension')&&data('extension').real_uart===true&&data('extension').status_fresh===true));
    $('mapRegions').querySelectorAll('[data-map-waypoint]').forEach(el=>{
      el.classList.toggle('visited',(searched.visited_waypoint_ids||[]).includes(el.dataset.mapWaypoint));
      el.classList.toggle('current',searched.active===true&&searched.current_waypoint_id===el.dataset.mapWaypoint);
    });
    const ref=pointStartReference(),refValid=Boolean(ref),isNumbered=ref?.kind==='numbered';
    $('mapPoints').querySelectorAll('[data-map-point]').forEach(el=>{el.classList.toggle('selected',Number(el.dataset.mapPoint)===state.pointTarget);el.classList.toggle('origin',isNumbered&&Number(el.dataset.mapPoint)===ref.point_id);});
    text('mapOriginSummary',refValid?(isNumbered?'已确认 '+ref.point_id+' 号点':'已确认地图起点 · '+num(ref.heading_deg,1)+'°'):regionReference?.valid===true&&fresh('extension')&&data('extension').status_fresh===true?'已确认地图起点 · '+num(regionReference.heading_deg,1)+'°':'请设置实际起点与朝向');
    text('mapLiveState',live.pose?'里程计车头位置':live.route?'收到真实路线':value?'底图就绪 · 位姿未知':'等待地图');$('mapLiveState').dataset.state=live.pose?'ok':'idle';
    text('mapTelemetry',live.pose?`车头 X ${num(live.pose.x_cm,1)} / Y ${num(live.pose.y_cm,1)} cm · ${num(live.pose.heading_deg,1)}°`:'没有新鲜 FPGA 位姿回包；不显示预测位置。');
  }
  $('mapSetStart').addEventListener('click',()=>{state.mapPickMode=state.mapPickMode==='start'?'target':'start';renderFpgaMap();});
  $('fpgaMap').addEventListener('click',event=>{
    if(state.mapPickMode!=='start')return;
    const svg=$('fpgaMap'),matrix=svg.getScreenCTM();if(!matrix)return;
    const point=svg.createSVGPoint();point.x=event.clientX;point.y=event.clientY;
    const local=point.matrixTransform(matrix.inverse());selectMapStart(local.x,local.y);
  });
  $('mapConfirmStart').addEventListener('click',()=>{
    const gate=mapStartReadiness();
    if(!gate.ready){toast(reason(gate.reason),true);return;}
    try{sendCommand(mapReferenceCommand(state.mapStart,mapData?.geometry?.height_cm));}
    catch(error){toast(error.message,true);}
  });
  async function pollFpgaMap(){
    try{
      const needsHeadings=!Array.isArray(mapData?.start_heading_samples)||!mapData.start_heading_samples.some(validMapHeadingSample);
      const response=await fetch('/api/fpga-map'+(needsHeadings?'?include_headings=1':''),{cache:'no-store',signal:AbortSignal.timeout(2000)});
      if(!response.ok)throw Error('map unavailable');
      const value=await response.json();if(value.schema_version!==1)throw Error('map schema');
      if(mapData&&!sameMapHeadingCatalog(mapData,value))state.mapStart=null;
      mapData=retainMapHeadingCatalog(mapData,value);mapReceived=performance.now();
    }
    catch{mapReceived=0;}
    finally{renderFpgaMap();setTimeout(pollFpgaMap,document.hidden?4000:1000);}
  }

  function renderGimbalFeedback(){
    const a=data('actuators'),t=data('gimbal_self_test'),live=fresh('actuators')&&a.dry_run===false&&a.status_fresh===true&&a.feedback_fresh===true;
    const actual=Array.isArray(a.actual_angles_deg)?a.actual_angles_deg:[],target=Array.isArray(a.target_deg)?a.target_deg:[];
    text('aimPanActual',live&&finite(actual[0])?num(actual[0],1)+'°':'—');text('aimTiltActual',live&&finite(actual[1])?num(actual[1],1)+'°':'—');
    text('aimPanTarget',live&&finite(target[0])?'目标 '+num(target[0],1)+'°':'目标待反馈');text('aimTiltTarget',live&&finite(target[1])?'目标 '+num(target[1],1)+'°':'目标待反馈');
    text('aimFeedbackReason',live&&actual.length===2?'实际角度来自电机串口回读。':'等待双轴真实角度反馈；指令角度不代表已到位。');
    const current=fresh('gimbal_self_test');$('gimbalTestStart').disabled=!canControl()||!current||t.can_start!==true;$('gimbalTestStop').disabled=!connected();
    text('gimbalTestPhase',current?(t.hardware_verified===true?'本轮硬件自检通过':reason(t.phase||'等待自检')+' · 实机未验收'):'实机未验收');text('gimbalTestReason',current?reason(t.can_start===true?t.reason||'小幅往返并回原位；不会自动归零。':t.ready_reason||t.reason||'条件未就绪'):'等待自检节点和实际停稳；不自动归零。');
  }
  async function submitGimbalTest(action){
    if(!connected())return;const status=data('gimbal_self_test');if(action==='start'&&(!canControl()||!fresh('gimbal_self_test')||status.can_start!==true)){toast('当前双轴自检条件未就绪。',true);return;}
    const payload={action,request_id:uuid()};if(action==='start')Object.assign(payload,{axes:$('gimbalTestAxes').value,amplitude_deg:1,speed_deg_s:6});
    try{await request('/api/gimbal-self-test',payload);toast('自检请求已提交，完成以实测回执为准。');}catch(error){toast(error.message,true);}
  }
  $('gimbalTestStart').addEventListener('click',()=>submitGimbalTest('start'));$('gimbalTestStop').addEventListener('click',()=>submitGimbalTest('stop'));

  function manualGimbalView(){
    const a=data('actuators'),m=data('gimbal_manual'),actual=a.actual_angles_deg;
    const ros=connected()?state.data?.ros||{}:{};
    const controlConnected=ros.available===true&&ros.gimbal_manual_ready===true;
    const actuatorCurrent=connected()&&fresh('actuators');
    const fault=actuatorCurrent&&typeof a.error==='string'?a.error.trim():'';
    const live=actuatorCurrent&&!fault&&a.dry_run===false&&a.status_fresh===true&&a.feedback_fresh===true;
    const rawLive=actuatorCurrent&&a.dry_run===false&&a.raw_feedback_fresh===true;
    const current=connected()&&fresh('gimbal_manual');
    const limits=Array.isArray(a.axis_limits_deg)?a.axis_limits_deg:[];
    const bounds=index=>live&&Array.isArray(limits[index])&&limits[index].length===2&&limits[index].every(finite)&&limits[index][0]<limits[index][1]?limits[index]:null;
    const reachable=[0,1].map(index=>{
      const range=m.axis_reachable_limits_deg?.[index],mechanical=bounds(index);
      return current&&mechanical&&Array.isArray(range)&&range.length===2&&range.every(finite)&&range[0]<=range[1]&&range[0]>=mechanical[0]&&range[1]<=mechanical[1]?range:null;
    });
    const steps=[0,1].map(index=>current&&finite(m.axis_step_deg?.[index])&&m.axis_step_deg[index]>0?m.axis_step_deg[index]:null);
    const diagnostic=actuatorCurrent&&a.diagnostic_feedback_fresh===true&&Array.isArray(a.diagnostic_angles_deg)?a.diagnostic_angles_deg:[];
    return {state:m,actuators:a,ros,controlConnected,actuatorCurrent,current,live,rawLive,fault,pan:bounds(0),tilt:bounds(1),reachable,steps,actual:live&&Array.isArray(actual)?actual:diagnostic,
      axes:current&&Array.isArray(m.motion_axes)?m.motion_axes:[],ready:controlConnected&&current&&live&&a.enabled===true&&a.reference_confirmed_this_start===true&&m.can_start===true};
  }
  function manualGimbalTopicStatus(key){
    const envelope=topic(key);
    if(!envelope||envelope.received_at==null)return '尚未收到';
    return fresh(key)?'正常更新':'已过期，等待更新';
  }
  function manualGimbalAngle(value){return String(Number(value.toFixed(3)));}
  function manualGimbalReason(value,view){
    const referenceReasons={
      gimbal_reference_raw_feedback_required:'等待两台电机真实串口回读',
      gimbal_reference_stationary_readback_required:'等待两轴停止并取得稳定编码器读数',
      gimbal_reference_operator_confirmation_required:'请先确认喷嘴处于机械零位、车辆与两轴均静止且俯仰有支撑',
      gimbal_reference_not_ready:'本次零位记录条件尚未就绪',
      gimbal_reference_confirmed:'本次零位已记录，可以输入目标角度',
      reference_required:'真实串口已连接，等待确认本次机械零位',
      operator_reference_required:'真实串口已连接，等待确认本次机械零位',
      gimbal_reconnecting_readback:'正在重连两台电机并读取位置',
      gimbal_recovered_no_motion:'连接与编码器参考已恢复，未执行机械回零；请重新提交目标角度',
      gimbal_recovery_dispatched_waiting_feedback:'恢复请求已提交，等待真实双轴反馈',
      gimbal_recovery_waiting_pump_off:'等待 FPGA 确认已关水',
      gimbal_recovery_waiting_stationary_readback:'等待两轴静止并取得有效位置',
      gimbal_recovery_waiting_saved_reference:'等待已保存的编码器参考通过核对',
      gimbal_recovery_running:'正在恢复连接，请等待车载回执',
      gimbal_recovery_cancelled:'恢复已停止，可重新点击恢复连接'
    };
    if(referenceReasons[value])return referenceReasons[value];
    if(value==='fd8_no_reachable_lattice_point')return '当前机械范围内没有可达的 FD8 角度点，请核对本次安装参考与机械限位。';
    if(typeof value==='string'&&value.includes('FD8')&&value.includes('not reachable')){
      const range=view.reachable[0];
      return 'FD8 的离散步距使这个端点无法在允许误差内到达；'+(range?`请参考本次可达端点 ${manualGimbalAngle(range[0])}° ～ ${manualGimbalAngle(range[1])}°，重新输入目标。`:'请等待本次可达端点反馈后重新输入目标。');
    }
    return reason(value);
  }
  function manualGimbalBlockReason(view){
    if(!connected())return '与 X5 的整车连接已失效，当前无法确认电机状态。已输入角度会保留；恢复实时连接后再检查，页面不会自动重发。';
    if(view.fault&&/(?:ZDT|PD42).*(?:timeout|timed out)/i.test(view.fault))return gimbalReadbackReason(view.fault);
    if(view.fault)return '上次运动已中止：'+view.fault+'。可点击“重置连接 / 恢复控制”重新读取双轴状态；保留固定零位，恢复后重新提交目标角度。';
    if(view.ros.available!==true)return '网页服务已连接，但车载 ROS 通信尚未就绪，无法送达云台指令。请检查车载交互服务启动日志；可以先输入角度。';
    if(!view.controlConnected)return '网页服务已连接，但未发现正式云台控制节点。请检查车载云台启动日志和串口占用；仅打开网页或重新输入角度不能启动电机。';
    if(!view.actuatorCurrent)return '云台控制节点在线，但执行器状态'+manualGimbalTopicStatus('actuators')+'，暂时无法确认双轴角度和零位。可以先输入角度，状态恢复后再提交。';
    if(view.actuators.readback_error&&view.actuators.raw_feedback_fresh!==true)return gimbalReadbackReason(view.actuators.readback_error);
    if(view.actuatorCurrent&&view.actuators.reference_confirmed_this_start!==true)return '本次开机零位尚未记录。真实串口回读就绪后，在本页确认喷嘴水平居中、俯仰水平（0° / 0°），即可记录零位并调整角度。';
    if(view.actuatorCurrent&&(view.actuators.dry_run!==false||view.actuators.enabled!==true))return '云台真实输出尚未启用。角度输入只保存草稿；启用正式云台后再点击运动。';
    if(!view.live)return '等待双轴真实角度回读；已输入的目标会保留，反馈恢复后再点击运动。';
    if(!view.current)return '云台控制节点在线，但角度控制状态'+manualGimbalTopicStatus('gimbal_manual')+'。请检查车载状态发布；已输入角度会保留，状态恢复后再提交。';
    return manualGimbalReason(view.state.ready_reason||view.state.reason||'条件未就绪',view);
  }
  function manualGimbalDraft(view,preset=null){
    if(!view.ready)throw Error(manualGimbalBlockReason(view));
    if(!view.pan||!view.axes.includes('pan'))throw Error('等待水平轴真实范围与运动权限。');
    const values={};
    for(const axis of ['pan','tilt']){
      const raw=(preset?preset[axis]:$('gimbalManual'+(axis==='pan'?'Pan':'Tilt')).value).trim();
      if(axis==='tilt'&&!raw)continue;
      if(!view.axes.includes(axis))throw Error((axis==='pan'?'水平':'俯仰')+'轴尚未开放运动；请先完成该轴验收，或清空俯仰目标保持当前角度。');
      const value=Number(raw),range=view[axis];
      if(!raw||!finite(value)||!range||value<range[0]||value>range[1])throw Error((axis==='pan'?'水平':'俯仰')+'角度必须在当前车载范围内。');
      values[axis+'_deg']=value;
    }
    return values;
  }
  function manualGimbalReferenceReason(view){
    if(!connected()||!view.controlConnected||!view.actuatorCurrent)return manualGimbalBlockReason(view);
    if(!view.current)return '等待新鲜的云台控制状态。';
    const a=view.actuators,recovery=a.reference_capture_supported===true&&a.reference_recovery_permitted===true&&Boolean(a.error)&&a.reference_recovery_error===a.error;
    if(a.estop)return '云台处于急停，暂不能记录零位。';
    if(view.fault&&!recovery)return manualGimbalBlockReason(view);
    if(!view.rawLive)return view.actuators.readback_error?gimbalReadbackReason(view.actuators.readback_error):'等待两台电机真实串口回读；不会把旧读数作为本次零位。';
    const counts=view.actuators.motor_counts,speeds=view.actuators.motor_rpm;
    if(!Array.isArray(counts)||counts.length!==2||!counts.every(Number.isInteger)||
       !Array.isArray(speeds)||speeds.length!==2||!speeds.every(value=>finite(value)&&value===0))return '等待两轴停止并取得稳定编码器读数。';
    if(view.state.reference_capture_ready!==true)return manualGimbalReason(view.state.reference_capture_reason||'gimbal_reference_not_ready',view);
    return '';
  }
  function renderManualGimbal(){
    const v=manualGimbalView(),m=v.state;
    const online=connected();
    text('gimbalManualConnection',online?'网页服务已连接':'未连接 X5');
    text('gimbalManualNode',!online?'等待连接':v.ros.available!==true?'ROS 未就绪':v.controlConnected?'控制节点在线':'未发现控制节点');
    text('gimbalManualStatus',online?manualGimbalTopicStatus('gimbal_manual'):'等待连接');
    text('gimbalManualFeedback',!online?'等待连接':!v.actuatorCurrent?manualGimbalTopicStatus('actuators'):v.live?'双轴回读正常':v.rawLive?(v.fault?'真实串口回读正常 · 等待恢复控制':'真实串口回读正常'):v.actuators.diagnostic_feedback_fresh===true?'诊断回读正常 · 等待恢复控制':'未取得有效回读 · 可重置连接');
    const absoluteState=v.actuators.absolute_reference_restore_state;
    text('gimbalManualReference',!v.actuatorCurrent?'等待状态':absoluteState==='restored'?'已恢复编码器参考':absoluteState==='failed'?'编码器参考恢复失败':v.actuators.absolute_reference_saved===true?(v.actuators.reference_confirmed_this_start===true?'零位参考已保存':'正在核对编码器参考'):v.actuators.reference_confirmed_this_start===true?'本次已记录':'本次未记录');
    const rawCounts=v.actuators.motor_counts,rawSpeeds=v.actuators.motor_rpm;
    text('gimbalManualRaw',v.rawLive&&Array.isArray(rawCounts)&&rawCounts.length===2&&rawCounts.every(Number.isInteger)?
      `编码器原始计数：水平 ${rawCounts[0]} / 俯仰 ${rawCounts[1]}`+
      (Array.isArray(rawSpeeds)&&rawSpeeds.length===2&&rawSpeeds.every(finite)?` · 转速 ${num(rawSpeeds[0],1)} / ${num(rawSpeeds[1],1)} RPM`:''):'等待双轴串口原始读数');
    const referenceReason=manualGimbalReferenceReason(v),referenceCheckbox=$('gimbalReferenceConfirm');
    referenceCheckbox.disabled=Boolean(referenceReason)||Boolean(state.gimbalManualPending);
    if(referenceReason)referenceCheckbox.checked=false;
    $('gimbalReferenceCapture').disabled=Boolean(referenceReason)||!referenceCheckbox.checked||Boolean(state.gimbalManualPending);
    text('gimbalReferenceCapture',v.actuators.reference_confirmed_this_start===true?'重新记录当前位置为机械零位':'确认当前位置为机械零位');
    $('gimbalReferenceCapture').title=referenceReason||'记录当前两轴读数为机械 0° / 0°，不转动、不使能、不清除驱动器编码器。';
    text('gimbalReferenceReason',referenceReason||(v.actuators.absolute_reference_saved===true?'已保存编码器零位参考；恢复此参考不会让喷嘴机械回零。若相同物理姿态的角度变化，请核对安装与驱动器编码器基准；不要用改待机目标角度掩盖偏移。':'将喷嘴摆到约定的机械 0° / 0°，保持静止并勾选后记录。待机 0° / 30° 与机械零位不同；记录不会转动电机。'));
    for(const [index,axis] of ['Pan','Tilt'].entries()){
      const key=axis.toLowerCase(),range=v[key],enabled=v.axes.includes(key);
      const input=$('gimbalManual'+axis);input.disabled=false;
      if(range){input.min=String(range[0]);input.max=String(range[1]);}else{input.removeAttribute('min');input.removeAttribute('max');}
      text('gimbalManual'+axis+'Actual',finite(v.actual[index])?num(v.actual[index],1)+'°':'—');
      const reachable=v.reachable[index],step=v.steps[index];
      const rangeText=range?`机械范围 ${range[0]}° ～ ${range[1]}°（总行程 ${range[1]-range[0]}°）`:'等待有效机械角度范围';
      const reachableText=reachable?` · 本次可达端点 ${manualGimbalAngle(reachable[0])}° ～ ${manualGimbalAngle(reachable[1])}°`:' · 等待本次可达端点';
      const stepText=step!==null?` · 指令分辨率 ${Number(step.toFixed(4))}°`:'';
      const permissionText=!enabled?(axis==='Tilt'?' · 俯仰方向尚未验收，保持当前实测角度':' · 水平轴未开放角度运动'):'';
      text('gimbalManual'+axis+'Range',!v.current?'等待车载范围与运动权限':rangeText+reachableText+stepText+permissionText);
    }
    let draftReason='';try{manualGimbalDraft(v);}catch(error){draftReason=error.message;}
    $('gimbalManualStart').disabled=Boolean(draftReason)||Boolean(state.gimbalManualPending);
    $('gimbalManualStart').title=draftReason||(state.gimbalManualPending?'正在等待提交回执':'按当前输入提交一次角度运动');
    let standbyReason='';try{manualGimbalDraft(v,{pan:'0',tilt:'30'});}catch(error){standbyReason=error.message;}
    $('gimbalManualStandby').disabled=Boolean(standbyReason)||Boolean(state.gimbalManualPending);
    $('gimbalManualStandby').title=standbyReason||'执行一次水平 0°、俯仰 30° 的到位运动，使用本次已确认参考，不重新清零。';
    $('gimbalManualStop').disabled=false;
    const recovering=(v.current&&m.recovery_active===true)||(v.actuatorCurrent&&v.actuators.recovery_active===true);
    $('gimbalManualRecover').disabled=!v.controlConnected||recovering||Boolean(state.gimbalManualPending);
    $('gimbalManualRecover').title=recovering?'正在重新读取双轴状态':!v.controlConnected?'等待 X5 云台控制节点连接':'取消旧目标、确认关水、重连串口并恢复已保存的零位；不会自动继续上次运动。';
    const phases={idle:'等待目标',running:'正在运动',referencing:'正在记录零位',recovering:'正在恢复连接',completed:'本次操作完成',cancelled:'运动已停止',failed:'运动中止',stopping:'正在停止'};
    text('gimbalManualPhase',!online?'整车连接失效':recovering?'正在恢复连接':v.fault?'运动已中止 · 可手动恢复':!v.controlConnected?'控制节点未就绪':!v.current?'等待控制状态':phases[m.phase]||reason(m.phase||'等待目标'));
    text('gimbalManualReason',draftReason||(state.gimbalManualPending?'正在等待提交回执；可继续编辑下一次目标，修改不会改变已提交指令。':'目标角度有效。点击“运动到角度”执行一次；输入或恢复连接不会自行转动。'));
    $('gimbalManualReason').dataset.state=!connected()?'offline':v.fault?'warn':'idle';
    const targetValid=v.current&&Array.isArray(m.target_deg)&&m.target_deg.length===2&&m.target_deg.every(finite);
    const realizedValid=v.current&&Array.isArray(m.realizable_target_deg)&&m.realizable_target_deg.length===2&&m.realizable_target_deg.every(finite);
    const goalText=targetValid?`请求目标 水平 ${manualGimbalAngle(m.target_deg[0])}° / 俯仰 ${manualGimbalAngle(m.target_deg[1])}°`:'';
    text('gimbalManualTarget',targetValid?goalText+(realizedValid?` · 可达目标 水平 ${manualGimbalAngle(m.realizable_target_deg[0])}° / 俯仰 ${manualGimbalAngle(m.realizable_target_deg[1])}°`:''):'未收到有效目标反馈');
    if(v.current&&m.last_request_id&&(!state.gimbalManualRequestId||m.last_request_id===state.gimbalManualRequestId))text('gimbalManualReceipt','回执 '+String(m.last_request_id)+' · '+manualGimbalReason(m.last_request_reason||m.last_result||'等待执行反馈',v));
  }
  function manualGimbalPayload(){
    const v=manualGimbalView();
    const values=manualGimbalDraft(v);
    return {action:'start',request_id:uuid(),...values};
  }
  async function submitManualGimbal(action){
    let payload;
    try{
      if(action==='recover'&&!manualGimbalView().controlConnected)throw Error('等待 X5 云台控制节点连接。');
      payload=['stop','recover'].includes(action)?{action,request_id:uuid()}:manualGimbalPayload();
    }
    catch(error){toast(error.message,true);return;}
    if(action!=='stop'&&state.gimbalManualPending)return;
    const generation=state.gimbalManualGeneration=(state.gimbalManualGeneration||0)+1;
    if(action!=='stop')state.gimbalManualPending=true;
    renderManualGimbal();
    try{const response=await request('/api/gimbal-manual',payload);if(generation===state.gimbalManualGeneration){state.gimbalManualRequestId=response.command_id||response.request_id;text('gimbalManualReceipt','请求已提交 · '+String(state.gimbalManualRequestId||''));toast(action==='stop'?'停止请求已提交，以实测状态为准。':action==='recover'?'已请求恢复连接，等待真实双轴回读；恢复后请重新提交角度。':'角度请求已提交，等待电机到位反馈。');}}
    catch(error){toast('云台请求未确认：'+error.message+'；不会自动重发。',true);}
    finally{if(action!=='stop')state.gimbalManualPending=false;renderManualGimbal();}
  }
  async function submitManualGimbalStandby(){
    if(state.gimbalManualPending)return;
    try{manualGimbalDraft(manualGimbalView(),{pan:'0',tilt:'30'});}
    catch(error){toast(error.message,true);renderManualGimbal();return;}
    $('gimbalManualPan').value='0';$('gimbalManualTilt').value='30';
    return submitManualGimbal('start');
  }
  async function submitManualGimbalReference(){
    if(state.gimbalManualPending)return;
    const v=manualGimbalView(),block=manualGimbalReferenceReason(v);
    if(block||!$('gimbalReferenceConfirm').checked){toast(block||'请先确认喷嘴机械零位、车辆与两轴均静止以及俯仰支撑条件。',true);renderManualGimbal();return;}
    const payload={action:'confirm_reference',request_id:uuid(),operator_zero_confirmed:true,supported_load:true,stationary_confirmed:true};
    const generation=state.gimbalManualGeneration=(state.gimbalManualGeneration||0)+1;
    state.gimbalManualPending=true;renderManualGimbal();
    try{
      const response=await request('/api/gimbal-manual',payload);
      if(generation===state.gimbalManualGeneration){state.gimbalManualRequestId=response.command_id||response.request_id;text('gimbalManualReceipt','零位记录请求已提交 · '+String(state.gimbalManualRequestId||''));toast('正在记录本次零位，以车载回执为准；未发送运动。');}
    }catch(error){toast('零位记录未确认：'+error.message+'；不会自动重发。',true);}
    finally{state.gimbalManualPending=false;$('gimbalReferenceConfirm').checked=false;renderManualGimbal();}
  }
  $('gimbalManualForm').addEventListener('submit',event=>{event.preventDefault();submitManualGimbal('start');});
  for(const axis of ['Pan','Tilt'])$('gimbalManual'+axis).addEventListener('input',renderManualGimbal);
  $('gimbalManualStandby').addEventListener('click',()=>submitManualGimbalStandby());
  $('gimbalManualStop').addEventListener('click',()=>submitManualGimbal('stop'));
  $('gimbalManualRecover').addEventListener('click',()=>submitManualGimbal('recover'));
  $('gimbalReferenceConfirm').addEventListener('change',renderManualGimbal);
  $('gimbalReferenceCapture').addEventListener('click',()=>submitManualGimbalReference());

  function taskWorkflowGuide(input) {
    const {online,extensionFresh,missionFresh,navigationFresh,actuatorFresh,selectionFresh,proposalFresh}=input;
    const e=input.extension||{},m=input.mission||{},n=input.navigation||{},a=input.actuators||{},s=input.selection||{},p=input.proposal||{};
    const result=(title,next)=>({title,next});
    if(!online||!extensionFresh)return result('等待任务连接','连上 X5 后选择拓展1或拓展2；以车载回执确认。');
    if(missionFresh&&['estop','fault'].includes(m.mode))return result('任务已停止','查看设备反馈并处理故障；不会自动继续运动。');
    if(e.mode_transition)return result('正在切换任务','等待车载模式回执，尚未开始新的导航。');
    if(e.task==='traffic')return result('独立交通任务','当前识别红绿灯；切回拓展任务后可做火场问答。');
    if(['idle','standby'].includes(e.task))return result('待机 · 等待你开启任务','说“开启任务1”做识别问答，或“开启任务2”做导航。');
    const physicalFresh=e.real_uart===true&&e.protocol_ready===true&&e.status_fresh===true;
    const region=e.region_search||{};
    const regionView=regionTaskPresentation(region);
    if(isCurrentRegionTask(e)&&regionView.terminal){
      const title=(region.region_id||'指定')+' 区 · '+regionView.title;
      if(['COMPLETE','COMPLETED'].includes(regionView.phase)&&region.exit?.confirmed_stopped===true){
        const outcome=regionTargetOutcome(region),unfinished=outcome.unresolved.length+outcome.pending.length;
        return result(title,unfinished?[outcome.summary,outcome.detail,'本轮路线与出口停稳已确认；未处置目标未计为完成，可重新下达任务。'].filter(Boolean).join('。'):
          '本轮路线与出口停稳已确认，可下达新的区域任务。');
      }
      const feedback=physicalFresh?'':'当前 FPGA 反馈尚未就绪。';
      return result(title,(regionView.detail||'本轮未完成')+'。'+feedback+'处理原因、确认实际起点与方向后重新发起任务；旧任务不会自动重发。');
    }
    if(region.active===true){
      if(!physicalFresh)return result('区域导航反馈待恢复','等待 FPGA 实际位置与停稳信息；进度以地图反馈为准。');
      const current=region.centering?.current_target_id,target=Number.isInteger(current)&&current>0?' · 消防员 #'+current:'';
      return result((region.region_id||'指定')+' 区 · '+regionView.title+(region.centering?.active===true?target:''),
        (regionView.detail?regionView.detail+'。':'')+'稳定确认消防员 → 缓慢前后对中 → 停稳观察 1 秒 → 返回原路线 → 继续搜索；不触发云台或喷水。');
    }
    const hasNavigation=navigationFresh&&Boolean(n.command_id)&&Number.isInteger(n.target_point);
    const navigationPhase=String(n.phase||n.state||'');
    if(physicalFresh&&region.phase==='COMPLETE'&&region.exit?.confirmed_stopped===true&&!e.busy&&!e.active&&e.task==='semantic'&&(!hasNavigation||e.last_result?.command_id===region.command_id))return result((region.region_id||'指定')+' 区 · 搜索结束，已在区外停车','全部搜索航点与出口停稳均已确认；可下达下一个区域任务。');
    const regionEnded=Boolean(region.command_id)&&['CANCELLED','FAILED','FAULT'].includes(String(region.phase||'').toUpperCase());
    if(physicalFresh&&regionEnded&&!e.busy&&!e.active&&(!hasNavigation||String(n.command_id)===String(region.command_id)||['fault','failed','cancelled','idle'].includes(navigationPhase))){
      const detail=regionView.detail||'查看地图下方的任务回执';
      const reference=e.region_reference;
      return result((region.region_id||'指定')+' 区任务未完成',detail+'。'+(reference?.valid===false?'请重新确认地图起点与方向，再提交区域任务。':'排除原因后重新提交区域任务；不会自动重发。'));
    }
    if(hasNavigation&&!['arrived','completed','fault','failed','cancelled','idle'].includes(navigationPhase)){
      if(!physicalFresh)return result('导航反馈待恢复','尚不能确认到达或位置；请查看 FPGA 通信状态。');
      return result('正在前往 '+n.target_point+' 号点','等 FPGA 确认到达并停稳，再自动开启 YOLO 观察。');
    }
    if(hasNavigation&&['fault','failed'].includes(navigationPhase))return result('导航未完成','查看地图下方失败原因，排除问题后重新下达指令。');
    if(actuatorFresh&&a.dry_run===false&&a.pump_on===true)return result('水泵控制输出开启','说“停止喷水”或点击顶部按钮；出水效果以现场为准。');
    if(selectionFresh&&s.valid===true&&Number.isInteger(s.target_id)){
      if(proposalFresh&&p.aim_mode==='fixed_camera_3d'&&p.calibration_ready===false)return result('已选火焰 #'+s.target_id+' · 等待安装标定','相机固定在车身，需标定相机到喷嘴云台的坐标关系。');
      if(proposalFresh&&p.valid===true&&p.geometry_locked===true)return result('火焰 #'+s.target_id+' · 喷嘴角度到位',
        p.water_aim_policy==='straight_ray'?'已按双目三维位置直线瞄准；可提交一轮短喷与关水，实际水流命中仍以观察为准。':
        p.landing_compensated===true?'需喷水时明确说“开始喷水”；也可随时说“停止喷水”。':'角度到位不等于水流命中；水流落点标定尚未完成。');
      if(proposalFresh&&p.valid===true&&p.image_locked===true)return result('火焰 #'+s.target_id+' · 图像已锁定','需喷水时明确说“开始喷水”；也可随时说“停止喷水”。');
      return result('已选火焰 #'+s.target_id+' · 等待闭环瞄准','在“视觉与瞄准”查看双轴反馈；不会自动开启水泵。');
    }
    const arrived=hasNavigation&&physicalFresh&&n.valid===true&&n.arrived===true&&n.vehicle_stopped===true;
    if(arrived)return result('已到达 '+n.target_point+' 号点 · 火场观察','可说“分析当前火场”，或按画面编号说“瞄准1号火焰”。');
    if(e.task==='vision')return result('拓展1 · 现场识别与问答','说“分析当前火场”获取回答；说“寻找火焰”开启观察。');
    if(['dialogue','semantic'].includes(e.task))return result('拓展2 · 等待你的导航指令','先在地图设置起点与方向，再说：“去 C 区域搜索消防员”；稳定确认后缓慢对中观察，不喷水。');
    return result('等待任务状态','选择拓展任务；已提交请求不代表动作完成。');
  }
  function renderTaskWorkflow() {
    const guide=taskWorkflowGuide({online:connected(),extensionFresh:fresh('extension'),missionFresh:fresh('mission'),navigationFresh:fresh('navigation')||fresh('extension'),actuatorFresh:fresh('actuators'),selectionFresh:fresh('fire_selection'),proposalFresh:fresh('fire_proposal'),extension:data('extension'),mission:data('mission'),navigation:numberedTaskState(),actuators:data('actuators'),selection:data('fire_selection'),proposal:data('fire_proposal')});
    text('taskFlowTitle',guide.title);text('taskFlowNext',guide.next);
    const next=$('taskFlowNext');if(next)next.title=guide.next;
    $('commandText').placeholder=['idle','standby'].includes(data('extension').task)?'输入“开启任务1”或“开启任务2”，也可点击上方按钮':'输入问题或指令，如“分析当前火场”“瞄准1号火焰”';
  }
  function renderStatus() {
    const live=connected(),rosReady=live&&state.data?.ros?.available===true,m=data('mission'),a=data('actuators'),pd=data('perception_diagnostics');document.body.dataset.link=rosReady?'live':'offline';
    badge('connectionBadge',!live?'连接中断':rosReady?'LIVE  车载数据':'车载节点未连接',!live?'offline':rosReady?'live':'warn');
    $('railDot').className='connection-dot'+(rosReady?' live':'');text('railConnection',rosReady?'X5 实时数据已连接':live?'服务已连接 · 等待节点':'等待连接 X5');
    const taskReady=rosReady&&state.data?.control?.command_ready;
    const banner=$('modeBanner');banner.classList.toggle('hidden',taskReady);banner.classList.toggle('error',!live);text('modeBanner',!live?(state.connectionError||'尚未连接实时车载服务。')+' 当前状态未知；指令不会自动重发。':!rosReady?'服务已连接，但尚未收到 ROS 车载连接；等待真实节点与相机。':!taskReady?'画面服务在线，任务交互尚未就绪。在 X5 工程目录运行 bash tools/start_interaction_services.sh，随后重新连接。':'');
    text('modeMetric',m.mode?(modeNames[m.mode]||m.mode):'等待状态');$('modeMetric').dataset.state=['fault','estop'].includes(m.mode)?'alarm':m.mode?'ok':'idle';text('modeDetail',m.mode?(m.gimbal_ready===true?'相机姿态：':'请求姿态（未到位）：')+({road:'低头看路',traffic:'抬头寻灯',observe:'观察楼面',inspect:'观察楼面',fire:'火焰瞄准'}[m.camera_pose]||m.camera_pose||'待反馈'):'尚无新鲜任务状态');
    document.querySelectorAll('[data-mode]').forEach(btn=>btn.classList.toggle('active',modeButtonActive(btn.dataset.mode,m)));
    const performanceInfo=pd.performance||pd;const framePerfFresh=connected()&&state.frameAt>0&&(performance.now()-state.frameAt)<2000&&state.frameStatus==='live';const previewPerf=framePerfFresh?state.previewPerformance:{};const fps=performanceInfo.processing_fps??performanceInfo.inference_fps??performanceInfo.processed_fps??performanceInfo.fps??previewPerf.processing_fps??previewPerf.inference_fps??previewPerf.fps;
    const yolo=m.yolo_requested;if(yolo===false){text('fpsMetric','按任务关闭');text('fpsDetail','交通任务时按需释放算力');}else{html('fpsMetric',esc(num(fps,1))+'<i> FPS</i>');const policy=typeof performanceInfo.dialogue_yielding==='boolean'?performanceInfo:previewPerf;text('fpsDetail',policy.dialogue_yielding===true&&finite(policy.effective_process_fps)&&policy.effective_process_fps>0?'问答时上限 '+num(policy.effective_process_fps,0)+' 帧，完成自动恢复':finite(fps)?'板端实际诊断 / 不等于展示帧率':'等待板端性能数据');}$('fpsMetric').parentElement.style.setProperty('--fps-ratio',String(yolo===false||!finite(fps)?0:Math.max(0,Math.min(1,fps/30))));$('fpsMetric').dataset.state=yolo===false||!finite(fps)?'idle':'ok';
    const counts=visibleCounts();html('targetMetric',counts.available?String(counts.fire+counts.person+counts.firefighter)+'<i> 个</i>':'—<i> 个</i>');text('countDetail',counts.available?`火焰 ${counts.fire} / 人物 ${counts.person} / 消防员 ${counts.firefighter}`:'当前检测未启用或证据过期');
    const e=data('extension'),motion=measuredVehicleMotion(topic('extension')?.data||e,fresh('extension'),m,fresh('mission'));
    text('motionMetric',motion.text);$('motionMetric').dataset.state=motion.state;$('motionMetric').title=motion.source+'；这表示当前运动事实，不代表任务已完成或已获运动许可。';
    if(fresh('extension')&&isCurrentRegionTask(e)){
      const region=e.region_search,view=regionTaskPresentation(region);
      text('modeMetric','拓展2 · '+(region.region_id||'区域')+' 区搜索消防员');
      $('modeMetric').dataset.state=view.state;text('modeDetail',view.title);
    }
    text('hardwareDetail',fresh('actuators')?(a.dry_run===true?(a.manual_pump_output_enabled===true?'云台未启用 · 手动试喷可用':'执行器 dry-run  无真实输出'):a.enabled===true?'执行器已启用  查验实际反馈':'执行器未启用'):'执行器状态尚未收到');
    text('lastUpdated',live?'状态同步 '+clock(state.data.server_time):'等待状态连接');
    renderTaskWorkflow();
  }
  function visibleCounts() {const scene=data('scene');if(scene.detector_available===true&&scene.stale!==true){const values=[scene.visible_fire_count,scene.visible_person_count,scene.visible_firefighter_count];if(values.every(finite))return{available:true,fire:values[0],person:values[1],firefighter:values[2]};}const d=data('detections');if(d.detector_available===true&&Array.isArray(d.detections)){const result={available:true,fire:0,person:0,firefighter:0};for(const item of d.detections)if(names[item.class_name])result[item.class_name]++;return result;}return{available:false,fire:0,person:0,firefighter:0};}
  function roadResultReason(value) {
    const fixtureMessages={no_supported_three_lamp_fixture:'未找到完整的红黄绿三灯板，请让三个灯位都进入画面',three_lamp_active_red:'识别到三灯板上方红灯发亮（支持中心发白）',three_lamp_active_green:'识别到三灯板下方绿灯发亮',three_lamp_brightness_ambiguous:'已找到三灯板，但红绿亮度不明确；黄灯或灯罩颜色不作为通行依据',three_lamp_both_bright:'红绿同时发亮或存在强反光，保持停车',three_lamp_multiple_fixtures:'看到多个灯板，暂不判断可通行'};
    if(fixtureMessages[value])return fixtureMessages[value];
    const messages={fixture_visible_no_clear_emission:'已找到三灯板，但未确认有红灯或绿灯点亮',no_supported_lamp:'未找到符合布局的灯板；请对准黑色竖板、上红下绿灯珠',supported_panel_and_led:'灯板与灯珠位置符合要求',red_and_green_visible:'同时看到红灯与绿灯，暂不判为可通行',multiple_fixtures_in_roi:'画面有多个灯板，请对准待测试的一个',panel_glare:'灯板过曝，请降低灯珠亮度或调整角度',invalid_image:'当前图像不可用于判断'};
    return messages[value]||reason(value);
  }
  function roadFrameFresh() {
    const age=finite(roadView.age)?roadView.age+(performance.now()-roadView.at)/1000:Infinity;
    return connected()&&state.stream&&roadView.status==='live'&&roadView.frame&&age>=0&&age<=1.5;
  }
  function roadFrameResult(frame,key){
    const result=frame?.[key];
    return result&&result.synthetic!==true&&result.capture_ros_s===frame.capture_ros_s&&result.frame_id===frame.frame_id?result:{};
  }
  function roadOverlayBoxes(frame) {
    // Raw road pixels have a different geometry from the undistorted YOLO feed.
    if(!frame||frame.synthetic===true||frame.coordinate_frame!=='raw_physical_left'||!finite(frame.capture_ros_s)||!finite(frame.width)||frame.width<=0||!finite(frame.height)||frame.height<=0)return [];
    const boxes=[];
    for(const result of [frame.traffic]){
      if(!result||result.synthetic===true||result.valid!==true||result.capture_ros_s!==frame.capture_ros_s||result.frame_id!==frame.frame_id)continue;
      const box=(['red','green','conflict'].includes(result.status)||(result.status==='unknown'&&result.fixture_layout==='three_lamp'))?result.fixture_bbox:null;
      const size=result.image_size;
      if(!Array.isArray(box)||box.length!==4||!box.every(finite)||!Array.isArray(size)||size.length!==2||!size.every(v=>finite(v)&&v>0))continue;
      const [x1,y1,x2,y2]=box;if(x2<=x1||y2<=y1)continue;
      const x=Math.max(0,Math.min(frame.width,x1/size[0]*frame.width)),y=Math.max(0,Math.min(frame.height,y1/size[1]*frame.height));
      const w=Math.max(0,Math.min(frame.width,x2/size[0]*frame.width)-x),h=Math.max(0,Math.min(frame.height,y2/size[1]*frame.height)-y);
      if(!w||!h)continue;
      const label={red:'红灯',green:'绿灯',conflict:'红绿冲突',unknown:'灯板 · 亮灯待确认'}[result.status];
      const color=result.status==='green'?'#30e49b':result.status==='red'?'#ff5b68':'#ffbf47';
      boxes.push({x,y,w,h,label,color});
    }
    return boxes;
  }
  function drawRoadFrame() {
    const c=$('roadVisionCanvas'),ctx=c.getContext('2d'),f=roadView.frame;if(!f||!roadView.bitmap)return;
    if(c.width!==f.width||c.height!==f.height){c.width=f.width;c.height=f.height;}
    ctx.drawImage(roadView.bitmap,0,0,c.width,c.height);
    ctx.font='600 17px "Microsoft YaHei", sans-serif';ctx.lineWidth=3;
    for(const b of roadOverlayBoxes(f)){
      ctx.strokeStyle=b.color;ctx.strokeRect(b.x,b.y,b.w,b.h);
      const w=Math.min(c.width,ctx.measureText(b.label).width+16),x=Math.min(b.x,c.width-w),y=b.y>=28?b.y-28:Math.min(c.height-28,b.y+2);
      ctx.fillStyle=b.color;ctx.fillRect(x,y,w,28);ctx.fillStyle='#081016';ctx.fillText(b.label,x+8,y+20);
    }
  }
  function showRoadFrameState() {
    const taskDisabled=fresh('extension')&&data('extension').task!=='traffic';
    const visible=!taskDisabled&&Boolean(roadFrameFresh()),c=$('roadVisionCanvas');
    const yielding=data('road_diagnostics').yielding_to_dialogue===true;
    const note=!connected()?'未连接 X5，等待实时识别画面':taskDisabled?'交通任务未开启。点击右侧“开启交通任务”，再把完整灯板对准镜头。':!state.stream?'画面传输已关闭，识别结果仍可在右侧查看':yielding?'正在为问答让出算力，回答结束后自动恢复':roadView.error||'等待红绿灯识别画面，请把完整灯板对准镜头';
    c.classList.toggle('hidden',!visible);$('roadVisionEmpty').classList.toggle('hidden',visible);text('roadVisionEmpty',note);
    text('roadVisionBadge',visible?'LIVE · 同帧识别':taskDisabled?'交通任务关闭':!state.stream?'传图已关闭':yielding?'为问答暂停':roadView.status==='stale'?'画面已过期':'等待画面');
    $('roadVisionBadge').dataset.state=visible?'ok':'idle';$('roadImageToggle').checked=state.stream;$('roadImageToggle').disabled=!connected();
    const f=roadView.frame,age=finite(roadView.age)?roadView.age+(performance.now()-roadView.at)/1000:null;
    const now=performance.now();while(roadView.frameTimes.length&&now-roadView.frameTimes[0]>2000)roadView.frameTimes.shift();
    const fps=roadView.frameTimes.length>1?1000*(roadView.frameTimes.length-1)/(roadView.frameTimes.at(-1)-roadView.frameTimes[0]):0;
    const diagnostic=data('road_diagnostics'),detectionFps=fresh('road_diagnostics')&&finite(diagnostic.processing_fps)?num(diagnostic.processing_fps,1):'—';
    text('roadVisionTiming',visible?`${f.width} × ${f.height} · 识别 ${detectionFps} / 展示 ${num(fps,1)} FPS · 图龄 ${Math.round(age*1000)} ms · 同帧`:note);
  }
  function stopRoadFrames(){clearTimeout(roadView.timer);roadView.epoch++;roadView.controller?.abort();roadView.frame=null;roadView.bitmap?.close();roadView.bitmap=null;roadView.status='waiting';roadView.error='';roadView.frameTimes=[];}
  function scheduleRoadFrame(ms){clearTimeout(roadView.timer);if(wantsRoadFrames()&&!roadView.busy)roadView.timer=setTimeout(pollRoadFrame,ms);}
  async function pollRoadFrame(){
    if(!wantsRoadFrames()||roadView.busy)return;
    roadView.busy=true;const epoch=roadView.epoch,start=performance.now(),controller=new AbortController();roadView.controller=controller;
    const timeout=setTimeout(()=>controller.abort(),2500);
    try{
      const response=await fetch('/api/road-frame',{cache:'no-store',signal:controller.signal});
      if(!response.ok)throw new Error(response.status===404?'X5 尚未提供识别图接口，请更新服务':'识别图连接暂不可用');
      const {metadata,jpeg}=unpack(await response.arrayBuffer());if(epoch!==roadView.epoch)return;
      const f=metadata.frame;
      if(f){
        if(f.coordinate_frame!=='raw_physical_left'||!finite(f.capture_ros_s)||typeof f.frame_id!=='string'||!Number.isInteger(f.width)||!Number.isInteger(f.height)||f.width<1||f.height<1||f.width>1280||f.height>1280)throw new Error('识别图坐标信息不完整');
        if(!roadView.bitmap||f.token!==roadView.frame?.token){
          const bitmap=await createImageBitmap(jpeg);if(epoch!==roadView.epoch){bitmap.close();return;}
          if(bitmap.width!==f.width||bitmap.height!==f.height){bitmap.close();throw new Error('识别图与尺寸不匹配');}
          roadView.bitmap?.close();roadView.bitmap=bitmap;roadView.frame=f;roadView.frameTimes.push(performance.now());drawRoadFrame();
        }
      }else{roadView.frame=null;roadView.bitmap?.close();roadView.bitmap=null;}
      roadView.status=metadata.state||'waiting';roadView.age=finite(metadata.frame_age_s)?metadata.frame_age_s+(performance.now()-start)/1000:null;
      roadView.at=performance.now();roadView.error='';
    }catch(e){if(epoch===roadView.epoch){roadView.status='stale';roadView.error=e.name==='AbortError'?'识别图传输超时，正在重新连接':e.message;}}
    finally{clearTimeout(timeout);roadView.busy=false;roadView.controller=null;showRoadFrameState();renderRoad();scheduleRoadFrame(Math.max(15,125-(performance.now()-start)));}
  }
  function selectRoadView(mode){
    roadView.mode=mode==='yolo'?'yolo':'detection';
    $('roadVisionPanel').classList.toggle('hidden',roadView.mode!=='detection');$('cameraSlot-road').classList.toggle('hidden',roadView.mode!=='yolo');
    $('roadShowDetection').setAttribute('aria-pressed',String(roadView.mode==='detection'));$('roadShowYolo').setAttribute('aria-pressed',String(roadView.mode==='yolo'));
    state.frameEpoch++;stopFrame(false);stopRoadFrames();scheduleFrame(0);scheduleRoadFrame(0);showRoadFrameState();renderRoad();
  }
  function renderRoad() {
    const lightKey=fresh('traffic_observation')?'traffic_observation':'traffic_preview';
    const sameFrame=state.view==='road'&&roadView.mode==='detection'&&state.stream;
    const f=roadFrameFresh()?roadView.frame:null;
    const m=data('mission'),t=data('traffic'),o=sameFrame?roadFrameResult(f,'traffic'):data(lightKey);
    const observationOnly=(sameFrame&&!f)||o.observation_only===true;
    const phase=t.state||m.traffic_state||'IDLE';let step=m.traffic_active===true?'scan':'request';if(phase==='WAIT_STOP')step='stop';if(['RELEASED','CROSSING','WAIT_CROSSING_COMPLETE'].includes(phase))step='release';document.querySelectorAll('[data-step]').forEach(el=>el.classList.toggle('active',!observationOnly&&fresh('mission')&&el.dataset.step===step));
    const color=String(o.valid===true?(o.status||o.state||o.color||o.classification||''):'').toLowerCase();const red=['red','red_light'].includes(color),green=['green','green_light'].includes(color);$('redLamp').classList.toggle('on',red);$('greenLamp').classList.toggle('on',green);text('lightLabel',red?'当前识别：红灯':green?'当前识别：绿灯':'灯色未知 / 无新鲜证据');$('lightLabel').dataset.state=red?'alarm':green?'ok':'idle';
    text('trafficEvidence',o.valid===true?(o.fixture_bbox?'完整灯板 · 本帧亮灯证据':'尚无有效灯板位置'):'等待有效识别画面');
    text('roadFresh',sameFrame?(f?'真实画面 · 同帧测试':'等待新鲜识别画面'):observationOnly?'真实画面 · 手动观察':fresh(lightKey)?'新鲜灯色证据':'待数据');$('roadFresh').dataset.state=(sameFrame?Boolean(f):fresh(lightKey))?'ok':'idle';text('roadDecision',observationOnly?'仅识别 · 不放行':fresh('mission')?(m.motion_allowed===true?'任务允许行驶':'任务保持停车'):'等待任务状态');$('roadDecision').dataset.state=observationOnly?'warn':fresh('mission')?(m.motion_allowed===true?'ok':'warn'):'idle';
    text('roadReason',data('road_diagnostics').yielding_to_dialogue===true?reason('yielding_to_dialogue'):observationOnly?'灯色：'+roadResultReason(o.reason)+'。':(phaseNames[phase]||phase)+'  '+(t.stop_wait_complete===true?'2 秒等待已完成。':'')+(t.stop_confirmation_basis==='elapsed_time_only'?'按等待计时寻灯；不代表已经收到实际停稳反馈。':m.mode==='standby'?'待机看到绿灯不会自行开车。':reason(t.reason||m.reason)));if(fresh('extension')&&data('extension').task!=='traffic'){text('roadFresh','交通任务关闭');text('lightLabel','灯色检测未启用');text('roadDecision','仅交通任务判定放行');text('roadReason','斑马线由 FPGA 识别；点击开启交通任务后，X5 才处理红绿灯。');$('redLamp').classList.remove('on');$('greenLamp').classList.remove('on');document.querySelectorAll('[data-step]').forEach(el=>el.classList.remove('active'));}}
  function renderBuilding() {const o=data('ocr'),s=data('scene');const b=o.building||s.recent_building_observation||{};const confirmed=b.confirmed===true;const texts=o.readings||o.texts||o.results||o.detections||[];const raw=Array.isArray(texts)?texts.map(x=>typeof x==='string'?x:x.text||'').filter(Boolean).join(' / '):typeof texts==='string'?texts:'';text('buildingNumber',confirmed?(b.number??'—')+' 号楼':'— 号楼');text('buildingName',confirmed?(b.name||'楼号已确认'):'等待有效楼牌观测');text('ocrText',raw||o.text||b.text|| (confirmed?b.name:'尚无有效识别文字'));text('ocrFresh',fresh('ocr')?(o.valid===true?'楼牌观测有效':'观测待确认'):'待数据');$('ocrFresh').dataset.state=fresh('ocr')?(o.valid===true?'ok':'warn'):'idle';text('associationState',s.building_association_verified===true?'楼栋归属已验证':'未验证  不推断整栋人数');if(data('ocr_diagnostics').yielding_to_dialogue===true){text('ocrFresh','为对话让出算力');text('buildingName',reason('yielding_to_dialogue'));$('ocrFresh').dataset.state='idle';}}
  function renderNavigation() {const e=data('extension'),n=data('navigation');if(fresh('extension')&&isCurrentRegionTask(e)){const r=e.region_search,view=regionTaskPresentation(r);text('navigationState',(r.region_id||'指定')+' 区 · '+view.title+(view.detail?' · '+view.detail:''));$('navigationState').dataset.state=view.state;return;}if(n.motion_kind==='numbered_point'){const actual=validPointId(n.target_point)?'目标 '+n.target_point+' 号点':'编号点导航';text('navigationState',actual+' · '+($('pointPhase').textContent||'等待反馈'));return;}const parts=[n.building_number?'目标 '+n.building_number+' 号楼':n.building_id?'目标楼栋 '+n.building_id:'导航状态',n.fpga_status?.state||n.proposal?.reason||n.state||n.status||'',n.reason].filter(Boolean).map(reason);text('navigationState',fresh('navigation')?[...new Set(parts)].join('  '):'未收到新鲜导航状态');$('navigationState').dataset.state=fresh('navigation')?'sel':'idle';}
  function fireLockDisplay(proposal) {
    const fixed=Object.prototype.hasOwnProperty.call(proposal,'geometry_locked')||proposal.aim_mode==='fixed_camera_3d'||proposal.camera_mount==='fixed_to_vehicle';
    const locked=proposal.valid===true&&(fixed?proposal.geometry_locked===true:proposal.image_locked===true);
    return {fixed,locked,label:fixed?'喷嘴方向':'画面锁定',value:locked?(fixed?'角度已到位':'图像已锁定'):proposal.valid===true?'正在对准':'待有效提案'};
  }
  function pumpOutputText(a, isFresh) {
    const label=a.pump_backend==='fpga_uart'?'FPGA B13 控制输出':'水泵控制输出';
    if(a.dry_run===true&&a.manual_pump_output_enabled!==true)return '执行器处于 dry-run，未向真实水泵输出。';
    if(!isFresh||typeof a.pump_on!=='boolean')return label+'状态未知，等待真实回报。';
    if(a.pump_on===true)return label+'开启；尚不代表实际出水或灭火成功。';
    return label+'关闭 · '+reason(a.pump_reason||a.guard?.reason||a.error||'实际落点与灭火效果需实物验收');
  }
  function stereoObservationAge(key='stereo_observation') {
    const envelope=topic(key),elapsed=Math.max(0,(performance.now()-state.received)/1000);
    const sampleAge=finite(envelope?.source_age_s)&&finite(envelope?.age_s)?
      Math.max(envelope.source_age_s,envelope.age_s)+elapsed+(state.observationTransportS||0):null;
    const ttl=finite(envelope?.stale_after_s)?Math.min(3,envelope.stale_after_s):3;
    return fresh(key)&&sampleAge!==null&&sampleAge<=ttl?sampleAge:null;
  }
  function stereoDisplaySample() {
    const current=data('stereo_observation'),age=stereoObservationAge();
    const usable=t=>t?.estimate_valid===true&&finite(t.range_m)&&t.range_m>0;
    const useful=o=>o.observation_only===true&&o.control_eligible===false&&
      ((o.scene_probe?.kind==='calibration_probe'&&usable(o.scene_probe))||
        (Array.isArray(o.targets)&&o.targets.some(t=>t?.class_id===0&&usable(t))));
    if(age!==null&&useful(current))return {o:current,age,cached:false,current};
    const key='stereo_last_valid_observation',last=data(key),lastAge=stereoObservationAge(key);
    if(lastAge!==null&&topic(key)?.display_cache===true&&useful(last))return {o:last,age:lastAge,cached:true,current};
    return {o:current,age,cached:false,current};
  }
  function stereoObservationView() {
    const sample=stereoDisplaySample(),sampleAge=sample.age,cached=sample.cached;
    if(sampleAge===null)return {
      value:seen('stereo_observation')?'测距状态已过期':'等待接入',
      detail:seen('stereo_observation')?'等待新的双目测距帧；历史距离不继续显示。':'等待双目测距观测；厂家参数估算需尺量核对。',
      summary:'暂无新鲜厂家参数测距观测'
    };
    const o=sample.o,s=data('fire_selection'),counterSource=cached&&fresh('stereo_observation')?sample.current:o;
    const note=o.metric_depth_verified===true?'仅测距观测':'厂家参数估算，待尺量核对';
    const counts=['left','right'].map((eye,i)=>{
      const raw=counterSource.raw_stats?.[eye],name=i===0?'左':'右';
      return name+' '+(Number.isInteger(raw?.received)&&raw.received>=0?raw.received:'—')+' 帧'+
        (Number.isInteger(raw?.accepted)&&raw.accepted>=0&&raw.accepted!==raw.received?'（有效 '+raw.accepted+'）':'');
    }).join(' / ');
    if(o.observation_only!==true||o.control_eligible!==false)return {
      value:'等待有效观测状态',detail:counts+'；观测用途标记不完整，暂不显示估算距离。',summary:'测距观测状态待核对'
    };
    const sameArray=(a,b,tolerance=0,length)=>Array.isArray(a)&&Array.isArray(b)&&a.length===b.length&&
      (length===undefined||a.length===length)&&
      a.every((value,i)=>finite(value)&&finite(b[i])&&Math.abs(value-b[i])<=tolerance);
    // A slow stereo result may belong to an earlier exposure. Never attach it
    // to the latest selected fire by proximity or its position in the list.
    const sameFrame=!cached&&s.valid===true&&Number.isInteger(s.target_id)&&
      Number.isInteger(o.source_sequence)&&o.source_sequence===s.sequence&&
      finite(o.capture_stamp_ns)&&o.capture_stamp_ns===s.capture_stamp_ns&&
      typeof o.coordinate_frame==='string'&&o.coordinate_frame===s.coordinate_frame&&
      sameArray(o.image_size_wh,s.image_size_wh,0,2);
    const targets=(Array.isArray(o.targets)?o.targets:[]).filter(t=>t&&t.class_id===0);
    const selected=sameFrame?targets.filter(t=>sameArray(t.bbox_xyxy,s.bbox_xyxy,.01,4)):[];
    const usable=t=>t?.estimate_valid===true&&finite(t.range_m)&&t.range_m>0;
    const chosen=selected.length===1?selected[0]:null;
    const probe=o.scene_probe?.kind==='calibration_probe'?o.scene_probe:null;
    const validTargets=targets.filter(usable),frame=Number.isInteger(o.source_sequence)?String(o.source_sequence):'—';
    const boxes=validTargets.slice(0,3).map((t,i)=>'框 '+
      (Number.isInteger(t.detection_index)&&t.detection_index>=0?t.detection_index+1:i+1)+' ≈ '+num(t.range_m,2)+' m');
    const boxDetail=boxes.length?'采集帧 '+frame+'：'+boxes.join('、')+
      (validTargets.length>3?'，共 '+validTargets.length+' 框':'')+'（仅对应该采集帧）':'';
    let value='暂无可靠距离',detail='',summary='厂家参数测距已接入，等待可靠距离';
    // The centre probe is independent of detector/selection updates. Frame-local
    // box distances remain visible without relabelling them as current fire IDs.
    if(usable(probe)){
      value='中心场景 ≈ '+num(probe.range_m,2)+' m';
      detail='中心区域测距，非火源';summary=value+'（非火源） · '+note;
    }else if(validTargets.length){
      value='帧 '+frame+' · '+boxes[0];
      detail='本帧火焰检测框估算，未绑定当前选定目标';summary=value+' · '+note;
    }else{
      detail=reason(chosen?.reason||probe?.reason||targets[0]?.reason||o.status||'等待双目配对画面');
      if(detail===reason('computed')||detail==='ok')detail='当前区域暂无可靠距离';
    }
    if(usable(chosen))detail+='；选定火焰 #'+s.target_id+' 同帧估算 '+num(chosen.range_m,2)+' m';
    if(boxDetail)detail+='；'+boxDetail;
    if(cached){
      const current=sample.current,computed=['computed','computed_delayed_observation'].includes(current.status);
      const failure=computed?(current.scene_probe?.reason||current.targets?.[0]?.reason||current.status):current.status;
      detail+='；本帧：'+(fresh('stereo_observation')?reason(failure||'暂无有效测距'):'测距状态已过期，等待新帧');
      value='最近一次 · '+value;summary='最近一次测距 · '+summary;
    }
    return {value,detail:(cached?'最近一次测距，采集于 '+num(sampleAge,1)+' 秒前；':'')+note+
      (o.delayed_observation===true?'；延迟观测':'')+'；'+counts+
      (cached?'':'；采集于 '+num(sampleAge,1)+' 秒前')+'；'+detail+
      '。观测距离不用于自动瞄准或喷水。',summary:summary+' · 采集于 '+num(sampleAge,1)+' 秒前'};
  }
  function renderStereoObservation() {
    const view=stereoObservationView();
    text('stereoObservationValue',view.value);text('stereoObservationDetail',view.detail);
    if($('stereoObservationDetail'))$('stereoObservationDetail').title=view.detail;
  }
  function renderFire() {const registry=data('fire_targets'),selection=data('fire_selection'),p=data('fire_proposal'),a=data('actuators'),m=data('mission'),st=data('stereo'),lock=fireLockDisplay(p);const aimIssue=lock.fixed&&p.calibration_ready===false?'固定相机三维瞄准等待相机与云台安装标定'+(p.reason?'：'+reason(p.reason):''):selection.valid===true&&p.valid!==true?(p.reason||p.geometry_reason||p.servo_plan?.reason||'等待有效瞄准提案'):'';const availability=!aimingReady()?'可继续识别；瞄准尚未就绪：'+aimingBlockReason()+(aimIssue?'；'+reason(aimIssue):''):aimIssue?'瞄准条件未满足：'+reason(aimIssue):lock.fixed?'相机固定在车身：喷嘴角度由三维位置计算；车辆居中观察不代表喷嘴命中。':'选择画面中的火焰编号，双轴自动瞄准；喷水等待你的指令。';text('fireAvailability',availability);$('fireAvailability').title=availability;const targets=Array.isArray(registry.targets)?registry.targets:[];html('targetList',targets.length?targets.map(t=>`<button class="target-pill${selection.valid&&selection.target_id===t.target_id?' selected':''}" data-target="${Number(t.target_id)}" data-selectable="${t.selectable===true}" disabled>火焰 #${esc(t.target_id)}<small>${finite(t.confidence)?num(t.confidence*100,0)+'%':t.selectable?'可选':'待确认'}</small></button>`).join(''):'<span class="empty-note">'+esc(reason(registry.reason||'当前没有新鲜可选火焰'))+'</span>');
    text('selectedTarget',selection.valid===true?'#'+selection.target_id:'未选定');html('targetDistance',p.valid===true&&finite(p.target_range_m)?esc(num(p.target_range_m,2))+'<small> m'+(p.range_is_assumption?'  固定距离假设':'')+'</small>':'—<small> m  未验证</small>');text('aimLockLabel',lock.label);text('imageLocked',lock.value);$('imageLocked').dataset.state=p.valid===true?(lock.locked?'ok':'warn'):'idle';
    text('actuatorTag',fresh('actuators')?(a.dry_run===true?(a.manual_pump_output_enabled===true?'云台未启用 · 手动试喷入口已开':'DRY-RUN  无真实输出'):a.enabled===true?'执行器已启用':'执行器未启用'):'待执行器反馈');$('actuatorTag').dataset.state=fresh('actuators')?(a.dry_run===true?'warn':a.enabled===true?'ok':'idle'):'idle';
    const gates=[['实际停稳反馈',m.vehicle_stopped===true],['有效选定目标',selection.valid===true],[lock.fixed?'喷嘴角度到位':'图像锁定',lock.locked],['水流落点补偿',p.landing_compensated===true],['喷水候选条件',p.pump_candidate===true],['真实水泵输出启用',a.enabled===true&&a.pump_enabled===true&&a.dry_run===false]];html('gateList',gates.map(([name,ok])=>`<div class="gate${ok?' pass':''}"><span></span>${esc(name)}</div>`).join(''));
    const regionActive=fresh('extension')&&regionSearchState().active===true;
    const roundText=fresh('fire_auto')?fireRoundProgress(data('fire_auto')):'';
    text('autoState',roundText?roundText:regionActive?'消防员区域任务正在执行：稳定确认后缓慢对中、停稳观察，再继续原路线；此任务不控制云台或喷水。':selection.valid===true?'已选火焰 #'+selection.target_id+'：系统闭环瞄准，水泵等待明确开始指令。':'先按画面编号选择火焰；识别到火焰不会自行瞄准或喷水。');
    text('pumpMeaning',pumpOutputText(a,fresh('actuators')));renderStereoObservation();
  }
  function renderVoice() {const v=data('voice'),raw=seen('transcript'),transcript=fresh('transcript')?data('transcript'):null;const voiceStates={ready:'语音节点就绪',idle:'等待语音',preparing:'正在准备语音识别',stopping:'正在结束车载收音',transcribing:'正在识别录音',listening:'正在车载收音',synthesizing:'正在合成播报语音',speaking:'正在车载播报',spoken:'播报进程已完成',loading_asr:'加载本地语音模型',microphone_disabled:'车载麦克风未启用',speech_preview:'文字播报预览',tts_disabled:'车载播报未启用',asr_unavailable_or_rejected:'语音识别未完成',command_submitted:'语音任务已提交',command_rejected:'语音任务未接受',listen_refused_during_speech:'播报中，暂缓收音',tts_unavailable:'音频播放不可用',audio_discarded:'本次音频已丢弃',speech_queue_full:'播报队列已满',listening_cancelled:'本次收音已取消'};
    text('micState',fresh('voice')?(voiceStates[v.state]||v.state||'语音状态待确认'):'等待麦克风状态');text('voiceBadge',fresh('voice')?(v.microphone_enabled===true?'本地音频已启用':v.microphone_enabled===false?'车载音频未启用':'语音事件已收到'):'等待语音节点');$('voiceBadge').dataset.state=fresh('voice')?(v.microphone_enabled===true?'ok':v.microphone_enabled===false?'warn':'sel'):'idle';$('micOrbit').classList.toggle('listening',v.state==='listening');text('micNote',v.capture_active===true?'现在可以对车载麦克风说话':v.listening===true?'语音处理中，请查看下方收音进度':v.reason?reason(v.reason):'点击开始后使用 X5 的 USB 接收器收音；电脑通过 Wi-Fi 发送请求');text('asrState',v.microphone_enabled===true?(v.model_present===false?'模型未就绪':v.asr_backend==='sensevoice'?(v.asr_load_error?'模型加载失败':v.asr_ready?'SenseVoice 中文':'SenseVoice 待加载'):'麦克风已启用'):v.microphone_enabled===false?'未启用':'以语音配置为准');const playback=speechPlaybackStatus(v,fresh('voice'));text('ttsState',playback.label);$('ttsState').dataset.state=playback.state;
    if(['idle','ready','command_submitted'].includes(v.state)&&v.capture_active!==true&&v.listening!==true&&!v.reason&&v.last_recognition?.state){const last=v.last_recognition;text('micNote','上次 '+clock(last.stamp)+'：'+reason(last.reason||last.state)+'；可点击开始收音。');}
    text('lastTranscript',raw?.text||'等你说出第一条指令…');text('transcriptDetail',raw?.text?(transcript?'机器人最近转写  ':'历史转写  ')+(raw.engine||'本地 ASR')+(finite(raw.confidence)?' / 解码评分 '+num(raw.confidence*100,0)+'%（不是准确率）':raw.confidence_available===false?' / 不提供置信度':'')+(finite(raw.decode_s)?' / 推理 '+num(raw.decode_s,2)+' 秒':'')+(raw.verification?.required?' / 动作复核'+(raw.verification.verified===true?'一致':'未通过'):''):'仅显示机器人实际发布的转写结果');
    const speech=seen('speech');const speechText=typeof speech==='string'?speech:speech?.text||speech?.data; text('lastSpeech',speechText||'尚未收到播报请求');text('speechStatus',playback.detail);
    const counts=visibleCounts(),s=data('scene');const facts=[['当前可见火焰',counts.available?counts.fire+' 个':'待检测'],['当前可见人物',counts.available?counts.person+' 人':'待检测'],['当前可见消防员',counts.available?counts.firefighter+' 人':'待检测'],['人物与楼栋关联',s.building_association_verified===true?'已验证':'未验证'],['双目深度',stereoDescription()],['实际灭火结果',s.extinguished_verified===true?'有验收证据':'未验证']];html('sceneFacts',facts.map(([k,v])=>`<div class="scene-fact"><span>${esc(k)}</span><strong>${esc(v)}</strong></div>`).join(''));
    renderVoicePower();
    if(v.voice_enabled===false){text('voiceBadge','车载语音已关闭');$('voiceBadge').dataset.state='idle';$('micOrbit').classList.remove('listening');}
    renderVoiceCapture();renderDialogueActivity();renderDialogue();renderTimeline();
  }
  function speechPlaybackStatus(v,isFresh) {
    if(!isFresh)return {label:'等待音箱状态',detail:'尚无新鲜播报节点反馈，不能确认是否已经播放。',state:'idle'};
    if(v.voice_enabled===false)return {label:'语音总开关已关闭',detail:'车载播报已关闭；文字提问与 Qwen 回答继续可用。',state:'idle'};
    const phase=v.tts_state||(v.state==='speaking'?'playing':v.state==='spoken'?'spoken':v.state==='tts_unavailable'?'unavailable':'');
    const error=v.tts_error||v.tts_last_result?.reason||v.reason||'';
    if(v.tts_enabled===false)return {label:'音箱播报未启用',detail:'TTS 当前未启用。',state:'warn'};
    if(phase==='unavailable'||v.tts_device_ready===false){
      const missing=/audio_output_(device|pcm)_missing/.test(error);
      return {label:missing?'USB 音箱未连接':'音频播放不可用',detail:(missing?'未检测到配置的 USB 音箱，请将音箱接到 X5 后重试。':'播报失败：'+reason(error||'音频设备不可用'))+(v.tts_audio_device?' 输出：'+v.tts_audio_device:''),state:'error'};
    }
    const states={preparing:['正在准备语音','正在检查音箱与语音模型，尚未播放。'],ready:['音箱已就绪','音频输出设备已就绪，等待播报请求。'],synthesizing:['正在合成语音','正在把回答合成为语音，尚未开始播放。'],playing:['正在音箱播报','音频播放进程正在运行。'],spoken:['播报已完成','音频播放进程已完成；实际声音以现场为准。'],expired:['播报请求已过期','本次播报已过期，未播放。']};
    const item=states[phase];
    if(item)return {label:item[0],detail:item[1],state:phase==='expired'?'warn':'ok'};
    return {label:v.tts_enabled===true?'音箱播报已启用':'等待音箱状态',detail:'/speech/say 是播报请求；播放情况以语音节点反馈为准。',state:'idle'};
  }
  function renderVoiceCapture() {
    const v=data('voice'),test=v.capture_mode==='audio_test';
    if(v.voice_enabled===false&&v.voice_control?.status!=='pending'){
      $('voiceCaptureProgress').value=0;text('voiceCaptureTime','文字指令与 Qwen 仍可用');text('voiceCaptureLevel','语音关闭');
      text('micState','语音已关闭');text('micNote','收音、识别与播报已关闭；点击“开启语音”可恢复，不会自动录音。');return;
    }
    const recording=v.capture_active===true&&!test;
    const duration=finite(v.duration_s)?v.duration_s:15;
    const elapsed=recording&&finite(v.elapsed_s)?Math.max(0,Math.min(duration,v.elapsed_s)):0;
    $('voiceCaptureProgress').max=duration;$('voiceCaptureProgress').value=elapsed;
    const ui=state.voiceControl||{},record=(state.data?.commands||[]).find(c=>c.request_id===ui.stop?.requestId);
    const view=voiceCaptureView(v,ui,record,connected(),fresh('voice'));
    text('voiceCaptureTime',view.stage==='recording'?'已收音 '+num(elapsed,1)+' / '+num(duration,0)+' 秒':view.stage==='idle'?'默认收音 '+num(duration,0)+' 秒，可手动结束':view.title);
    const now=(state.data?.server_time||0)+(performance.now()-state.received)/1000;
    const actualLevel=recording&&finite(v.level_dbfs)&&finite(v.level_stamp)&&now-v.level_stamp>=0&&now-v.level_stamp<1.6;
    text('voiceCaptureLevel',actualLevel?num(v.level_dbfs,0)+' dBFS':recording?'正在采集':test?'仅测试':'待收音');
    if(view.stage!=='idle'){text('micState',view.title);text('micNote',view.stage==='recording'?'可以连续说话；说完点“说完提交”，或等待 '+num(duration,0)+' 秒自动结束。':view.note);}
  }
  function voiceCaptureView(v,ui={},record=null,online=true,voiceFresh=true) {
    const stop=ui.stop;
    if(stop&&(!online||!voiceFresh))return {stage:'unknown',title:'等待麦克风停止确认',note:'语音状态未更新，暂不能确认是否停收；恢复连接后可再次点击结束。'};
    if(stop&&(stop.error||['rejected','expired'].includes(record?.status)))return {stage:'error',title:'结束请求未确认',note:reason(stop.error||record.reason||record.status)+'；可再次点击结束。'};
    const acknowledged=stop&&(finite(v.stop_request_seq)?v.stop_request_seq>(stop.baselineStopSeq||0):finite(v.stop_requested_at)&&v.stop_requested_at>stop.baselineStopAt);
    if(acknowledged&&v.stop_request_accepted===false)return {stage:'error',title:'结束请求未执行',note:v.stop_request_reason==='audio_test_requires_own_stop'?'正在独立收声测试，请打开“收声测试”窗口点击结束；测试未被中断。':reason(v.stop_request_reason||'语音节点拒绝结束请求')};
    if(v.listening!==true&&v.capture_active!==true&&(v.state==='asr_unavailable_or_rejected'||v.last_recognition?.state==='asr_unavailable_or_rejected'))return {stage:'error',title:'语音处理未完成',note:reason(v.reason||v.last_recognition?.reason||'请检查音频设备和识别状态')+'；请检查状态后再开始收音。'};
    if(stop&&!acknowledged)return {stage:'stop_pending',title:'结束请求待车载确认',note:'已点击结束；等待语音节点回执，HTTP 排队成功不代表麦克风已停止。'};
    if(v.capture_mode==='audio_test')return {stage:'test',title:'正在独立收声测试',note:'请在收声测试窗口结束该测试；此处不会把测试录音作为任务提交。'};
    if(v.capture_active===true&&(v.stop_requested===true||acknowledged))return {stage:'stopping',title:'正在结束车载收音',note:'正在关闭麦克风采集；已录声音将保留并转写，不需要继续说话。'};
    if(v.capture_active===true)return {stage:'recording',title:'正在车载收音',note:''};
    if(v.state==='transcribing'||v.recognition_phase==='transcribing')return {stage:'transcribing',title:'收音已结束，正在识别文字',note:'麦克风已停止采集，正在转写已录声音；识别完成后再交给任务解析或 Qwen 回答。'};
    if(v.stop_requested===true&&v.capture_stop_confirmed!==true)return {stage:'stopping',title:'正在结束语音准备',note:'已收到结束请求，等待准备过程退出；不会继续开始新的采集。'};
    if(['preparing','loading_asr'].includes(v.state)||ui.pendingStart)return {stage:'preparing',title:'正在准备收音',note:'等待“正在车载收音”再说话；准备期间也可以点击结束取消。'};
    if(v.stop_requested===true&&v.capture_stop_confirmed===true)return {stage:'ended',title:'车载收音已结束',note:'采集已停止；本次文字和回答会在下方按顺序显示。'};
    return {stage:'idle',title:'等待语音',note:''};
  }
  function renderDialogueActivity() {
    const d=data('dialogue_status'),v=data('voice');
    const navigation=data('interaction');
    if(connected()&&fresh('interaction')&&navigation.semantic?.status==='understanding'){
      $('dialogueActivity').dataset.state='thinking';
      text('dialogueActivity','旧版导航节点仍在等待 Qwen · 结果将在原指令下方显示');
      return;
    }
    const thinking=['thinking','generating','processing'].includes(d.state);
    const failed=['error','failed','timeout'].includes(d.state);
    $('dialogueActivity').dataset.state=thinking?'thinking':failed?'error':'idle';
    text('dialogueActivity',thinking?'本地 Qwen 正在思考'+(finite(d.elapsed_s)?' · '+num(d.elapsed_s,0)+' 秒':'')+' · 回答将在下方显示':failed?'上次问答：'+reason(d.reason||'模型暂不可用，请查看回答或稍后重试'):v.voice_enabled===false?'车载语音已关闭 · 可继续输入文字指令或向 Qwen 提问':v.listening===true&&v.capture_mode!=='audio_test'?'先完成收音与转写，再提交问题或明确控制指令':fresh('dialogue_status')?'本地 Qwen 问答就绪 · 现场事实由视觉与节点反馈提供':'声音转文字后，本地问答或明确任务解析');
  }
  function stereoDescription(){const p=data('fire_proposal'),s=data('stereo');if(fresh('fire_proposal')&&p.valid===true&&p.range_source==='matched_stereo_euclidean'&&finite(p.target_range_m))return '双目目标 '+num(p.target_range_m,2)+' m';if(stereoDisplaySample().age!==null)return stereoObservationView().summary;if(!fresh('stereo'))return '暂无新鲜深度状态';return s.gate==='ready'?'标定就绪 / 等待有效目标':reason(s.gate||s.status||'未完成标定');}
  const eventKey = e => e.kind||Object.keys(labels).find(k=>topic(k)?.topic===e.topic)||e.topic||'event';
  function eventContent(e) {const d=e.data; if(typeof d==='string')return d;if(!d||typeof d!=='object')return'';return d.text||d.message||d.speech||(d.reason?reason(d.reason): (d.accepted===true?'任务路由接受；不代表执行完成':d.accepted===false?'未接受：'+reason(d.reason):reason(d.state||d.status||'状态已更新')));}
  function commandReceiptMeta(id) {
    const record=(state.data?.commands||[]).find(c=>c.command_id===id||c.request_id===id);
    const prefix=connected()?'':'历史请求 · ';
    if(!record)return prefix+'已提交 · 等待任务回执';
    if(record.execution)return prefix+executionResultStatus(record.execution,record.action).text;
    if(record.status==='execution_timeout')return prefix+'规则已解析 · 执行节点回执超时，未确认下发运动';
    if(record.receipt?.semantic?.status==='understanding')return prefix+'本地 Qwen 正在理解导航 · 尚未下发运动';
    if(record.receipt?.accepted===false)return prefix+'任务未接受 · '+reason(record.receipt.reason);
    if(record.receipt?.accepted===true)return prefix+'任务已接受 · 执行进度以车载状态为准';
    const statuses={queued:'排队待发',submitted:'已发布 ROS · 等待任务回执',receipt_timeout:'任务回执超时 · 未自动重发',expired:'请求已过期',rejected:'请求被拒绝'};
    return prefix+(statuses[record.status]||'等待任务回执')+(record.reason?' · '+reason(record.reason):'');
  }
  function executionResultStatus(value,action) {
    const navigation=['navigate_point','navigate','search_region'].includes(action);
    if(value.accepted===false)return {state:'error',text:(navigation?'导航':'任务')+'未执行：'+reason(value.reason||'执行节点拒绝了请求')};
    if(value.completed===true)return {state:'receipt',text:'任务节点报告本次处理完成；车辆是否到达以 FPGA 导航状态为准。'};
    return {state:'receipt',text:(navigation?'导航':'任务')+'节点已接收，等待 FPGA 确认执行；尚未确认车辆运动。'};
  }
  function navigationExecutionTimeout(turn) {
    if(turn.execution||turn.receipt?.accepted!==true||!['navigate_point','search_region'].includes(turn.action))return false;
    return turn.status==='execution_timeout'||finite(turn.receiptAt)&&finite(state.data?.server_time)&&state.data.server_time-turn.receiptAt>5;
  }
  function voiceReceiptMeta(value) {
    if(value?.route_error)return '车载语音转写 · 未提交：'+reason(value.route_error);
    if(!value?.command_id)return '车载语音转写';
    const outcome=liveEvents().filter(e=>eventKey(e)==='extension_result'&&e.synthetic!==true&&e.data?.synthetic!==true&&e.data?.command_id===value.command_id).at(-1)?.data;
    if(outcome)return '车载语音转写 · '+executionResultStatus(outcome,value.action).text;
    const reply=liveEvents().filter(e=>eventKey(e)==='interaction'&&e.data?.command_id===value.command_id).at(-1)?.data;
    if(reply?.semantic?.status==='understanding')return '车载语音转写 · 本地 Qwen 正在理解导航，尚未下发运动';
    if(reply?.accepted===false)return '车载语音转写 · 未执行：'+reason(reply.reason);
    if(reply?.accepted===true)return '车载语音转写 · 已接收'+(['dialogue','scene','report'].includes(value.action)?'，等待回答':'，动作进度以实车反馈为准');
    return '车载语音转写 · 已提交，等待任务回执';
  }
  function answerMeta(value) {
    if(value?.rule_navigation)return '规则导航'+(value.rule_navigation.region_id?' · '+value.rule_navigation.region_id+' 区搜索消防员':'')+' · 文字与语音统一解析'+
      (value.accepted===true?' · 规则已解析，等待导航执行节点回执':' · 未下发导航');
    if(value?.semantic){
      const s=value.semantic;
      return (s.model_called===true?'本地 Qwen · '+(s.model||'导航理解'):'导航校验')+
        (finite(s.elapsed_s)?' · '+num(s.elapsed_s,1)+' 秒':'')+
        (value.accepted===true?' · 任务已提交，等待 FPGA 执行反馈':' · 未下发导航');
    }
    if(value?.accepted===false)return '问答未接受 · '+reason(value.reason);
    const scopes={system_status:'实际节点状态',current_facts:'当前视觉事实',historical_snapshot_analysis:'提问时的视觉快照',general_knowledge:'通用问答',capabilities:'功能说明',domain_reference:'项目技术参考'};
    return (['qwen','edge_llm_generate'].includes(value?.backend)?'本地 Qwen':'结构化回复')+' · '+(scopes[value?.response_scope]||reason(value?.backend))+(value?.model_error?' · 模型回退':'');
  }
  function collectDialogueTurns() {
    // Keep bounded UI history when the server's event ring advances. IDs, never
    // text or arrival proximity, associate a reply with its original question.
    const turns=state.dialogueTurns||(state.dialogueTurns=new Map());
    const commandId=value=>typeof value==='string'&&value?value:null;
    const ensure=(id,at)=>{if(!turns.has(id))turns.set(id,{id,at:finite(at)?at:0});return turns.get(id);};
    const question=(id,value,at,source='text')=>{
      if(!id||typeof value!=='string'||!value.trim())return null;
      const turn=ensure(id,at);
      if(!turn.text||(turn.source==='recovered'&&source!=='recovered')){turn.text=value;turn.at=finite(at)?at:turn.at;turn.source=source;}
      return turn;
    };
    const answer=(value,at,fallback)=>{
      if(!value||typeof value!=='object'||value.synthetic===true)return;
      const id=commandId(value.command_id)||fallback;if(!id)return;
      const turn=ensure(id,at);
      if(turn.answer?.accepted===true&&value.accepted===false&&value.reason==='duplicate_question')return;
      // A repeated event updates this answer instead of adding another bubble.
      const nextAt=finite(value.stamp)?value.stamp:at;
      if(turn.answer&&finite(nextAt)&&nextAt<turn.answerAt)return;
      turn.answer=Object.fromEntries(['text','accepted','reason','backend','response_scope','model_error','semantic','rule_navigation'].map(k=>[k,value[k]]));
      turn.answerAt=finite(nextAt)?nextAt:turn.at;
    };
    const execution=(value,at)=>{
      if(!value||typeof value!=='object'||value.synthetic===true||typeof value.accepted!=='boolean')return;
      const id=commandId(value.command_id);if(!id||!finite(value.stamp))return;
      const t=ensure(id,at),prior=t.execution;
      if(prior&&(value.stamp<prior.stamp||prior.accepted===false||prior.completed===true))return;
      t.execution=Object.fromEntries(['command_id','accepted','completed','reason','stamp'].map(k=>[k,value[k]]));
    };
    const receipt=(id,value,at)=>{
      const t=ensure(id,at),stamp=finite(value.stamp)?value.stamp:at;
      if(finite(t.receiptAt)&&finite(stamp)&&stamp<t.receiptAt)return;
      // Refreshed snapshots must not replace a completed inference with its
      // earlier pending receipt, even when events arrive out of order.
      if((t.ruleNavigation||t.semantic&&t.semantic.status!=='understanding')&&value.semantic?.status==='understanding')return;
      t.receipt={accepted:value.accepted,reason:value.reason,action:value.action,status:value.status};
      t.receiptAt=stamp;t.action=value.action||t.action;
      if(value.rule_navigation){
        t.ruleNavigation=value.rule_navigation;delete t.semantic;
        question(id,value.rule_navigation.original_text,at,'recovered');
        answer({...value,
          text:value.speech||(value.accepted===true?'导航规则已解析并提交，执行进度以 FPGA 反馈为准。':'导航未执行：'+reason(value.reason)),
          backend:'rule_navigation'},at,id);
      }
      if(value.semantic){
        t.semantic=value.semantic;
        question(id,value.semantic.original_text,at,'recovered');
        if(value.semantic.status!=='understanding')answer({...value,
          text:value.speech||(value.accepted===true?'旧版导航理解已完成，执行进度以 FPGA 反馈为准。':'导航未执行：'+reason(value.reason)),
          backend:'local_qwen_navigation'},at,id);
      }
    };
    for(const m of state.localMessages){
      const t=question(m.id,m.text,m.at);if(t){if(m.action)t.action=m.action;t.status=t.status||m.status;}
    }
    const records=(state.data?.commands||[]).filter(c=>c.synthetic!==true&&typeof c.text==='string'&&c.text.trim());
    for(const c of records){
      const id=commandId(c.command_id)||commandId(c.request_id),t=question(id,c.text,c.queued_at);if(!t)continue;
      if(finite(c.queued_at))t.at=c.queued_at;
      t.action=c.action;t.status=c.status;t.reason=c.reason;
      if(c.receipt)receipt(id,c.receipt,c.receipt.stamp??c.queued_at);
      if(c.answer)answer(c.answer,c.queued_at,id);
      if(c.execution?.command_id===id)execution(c.execution,c.execution.stamp);
    }
    const events=[...liveEvents()];
    for(const key of ['transcript','interaction','scene_answer']){
      const envelope=topic(key),v=envelope?.data;
      if(envelope?.synthetic===true||!v||v.synthetic===true)continue;
      if(commandId(v.command_id)||finite(v.result_stamp)||finite(v.stamp))events.push({kind:key,data:v,at:envelope.received_at});
    }
    const ext=topic('extension');
    if(ext?.synthetic!==true&&ext?.data?.synthetic!==true&&ext?.data?.last_result)
      events.push({kind:'extension_result',data:ext.data.last_result,at:ext.data.last_result.stamp});
    events.sort((a,b)=>(a.at||0)-(b.at||0));
    for(const e of events){
      const v=e.data;if(e.synthetic===true||!v||typeof v!=='object'||v.synthetic===true)continue;
      const key=eventKey(e),id=commandId(v.command_id);
      const fallback='event-'+key+'-'+String(v.result_stamp??v.stamp??(String(e.id)+'-'+String(e.at)));
      if(key==='transcript'||e.topic==='/voice/transcript'){
        const t=question(id||fallback,v.text,e.at,'voice');if(!t)continue;
        t.source='voice';t.action=v.action;t.routeError=v.route_error;
        t.submitted=v.submitted;
      }else if(key==='interaction'||e.topic==='/interaction/result'){
        if(!id)continue;
        // Router receipts retain the original question even if its transcript
        // has already left the server's rolling event history.
        question(id,v.question?.text,e.at,'recovered');
        receipt(id,v,e.at);
      }else if(key==='scene_answer'||e.topic==='/scene/answer')answer(v,e.at,fallback);
      else if(key==='extension_result')execution(v,e.at);
    }
    const d=data('dialogue_status');
    if(commandId(d.command_id)&&typeof d.question==='string'){
      const at=finite(d.stamp)&&finite(d.elapsed_s)?d.stamp-d.elapsed_s:d.stamp;
      question(d.command_id,d.question,at,'recovered');
    }
    const visible=[...turns.values()].filter(t=>t.text||t.answer||t.execution).sort((a,b)=>a.at-b.at||a.id.localeCompare(b.id)).slice(-25);
    state.dialogueTurns=new Map(visible.map(t=>[t.id,t]));
    return visible;
  }
  function dialogueTurnStatus(turn) {
    if(turn.execution)return executionResultStatus(turn.execution,turn.action);
    if(navigationExecutionTimeout(turn))return {state:'error',text:'导航指令已解析，但未收到执行节点回执，无法确认已经下发运动；未自动重发。'};
    if(turn.routeError)return {state:'error',text:'语音未提交：'+reason(turn.routeError)};
    if(turn.semantic?.status==='understanding')return connected()
      ?{state:'thinking',text:'本地 Qwen 正在理解点位和车头方向，尚未下发运动…'}
      :{state:'waiting',text:'连接已断开，等待重连后确认导航理解结果。'};
    if(turn.receipt?.accepted===false)return {state:'error',text:'未执行：'+reason(turn.receipt.reason)};
    if(['rejected','expired'].includes(turn.status))return {state:'error',text:reason(turn.reason||(turn.status==='expired'?'请求已过期':'请求被拒绝'))};
    if(turn.source==='voice'&&turn.submitted===false)return {state:'error',text:'这段语音未提交为任务，请核对转写内容。'};
    if(!connected())return {state:'waiting',text:'连接已断开，等待重连后获取回复。'};
    const d=data('dialogue_status');
    if(d.command_id===turn.id){
      if(['thinking','generating','processing'].includes(d.state))return {state:'thinking',text:'正在思考'+(finite(d.elapsed_s)?' · '+num(d.elapsed_s,0)+' 秒':'')+'…'};
      if(['error','failed','timeout'].includes(d.state))return {state:'error',text:reason(d.reason||'模型暂不可用，请稍后重试')};
    }
    if(turn.status==='receipt_timeout')return {state:'error',text:reason(turn.reason||'回执超时，未自动重发')};
    const asksAnswer=['dialogue','scene','report'].includes(turn.action);
    if(turn.receipt?.accepted===true&&!asksAnswer)return {state:'receipt',text:'任务已接受，执行进度以实车反馈为准。'};
    const now=state.data?.server_time;
    if(finite(now)&&now-turn.at>90)return {state:'waiting',text:asksAnswer?'暂未收到这次问题的回答，请检查任务状态。':'暂未收到新的回执，请查看系统证据。'};
    return {state:'waiting',text:turn.receipt?.accepted===true?'问题已接收，等待回答…':'已提交，等待任务回执…'};
  }
  function renderDialogue() {
    const turns=collectDialogueTurns();
    const bubble=(kind,label,body,meta,status='')=>`<div class="message ${kind}"${status?' data-state="'+esc(status)+'"':''}><span class="message-label">${esc(label)}</span><div class="message-body">${esc(body)}</div><span class="message-meta">${esc(meta)}</span></div>`;
    const content=turns.length?turns.map(t=>{
      let body=t.text?bubble('user','你',t.text,(t.source==='voice'?'车载语音':t.source==='recovered'?'任务原文':'文字提问 / 指令')+' · '+clock(t.at)):'';
      if(t.execution){
        const status=executionResultStatus(t.execution,t.action);
        body+=bubble('answer status','小车助手',status.text,'任务执行节点回执 · '+clock(t.execution.stamp),status.state);
      }else if(navigationExecutionTimeout(t)){
        const status=dialogueTurnStatus(t);
        body+=bubble('status','小车助手',status.text,'等待导航执行节点回执',status.state);
      }else if(t.answer){
        const a=t.answer;
        body+=bubble('answer'+(a.accepted===false?' status':''),t.text?'小车助手':'小车助手 · 历史回答（原问题未保留）',a.semantic||a.rule_navigation?a.text:(a.accepted===false?'这次问题未接受：'+reason(a.reason):a.text||'未收到回答正文，请查看系统证据。'),answerMeta(a)+' · '+clock(t.answerAt),a.accepted===false?(a.semantic?.status==='clarification'?'waiting':'error'):'');
      }else{
        const status=dialogueTurnStatus(t);
        body+=bubble('status','小车助手',status.text,'任务状态',status.state);
      }
      return `<section class="dialogue-turn" data-turn-id="${esc(t.id)}" aria-label="一轮问答">${body}</section>`;
    }).join(''):'<div class="empty-note">发一条消息开始对话。你的问题在右侧，小车的回答会紧接在下方。</div>';
    const box=$('dialogueMessages'),visible=box.clientHeight>0;
    const atBottom=visible?box.scrollHeight-box.scrollTop-box.clientHeight<70:state.dialogueFollowEnd!==false;
    if(state.dialogueMarkup!==content){
      // Keep the currently read turn anchored if a slower earlier answer grows.
      const top=visible?box.getBoundingClientRect?.().top:undefined;
      const anchor=Number.isFinite(top)?[...box.querySelectorAll('.dialogue-turn')].find(el=>el.getBoundingClientRect().bottom>top):null;
      const anchorId=anchor?.dataset.turnId,offset=anchor?anchor.getBoundingClientRect().top-top:0;
      box.innerHTML=content;
      state.dialogueMarkup=content;
      if(!visible){state.dialogueScrollPending=atBottom;}
      else if(atBottom)box.scrollTop=box.scrollHeight;
      else if(anchorId){const next=[...box.querySelectorAll('.dialogue-turn')].find(el=>el.dataset.turnId===anchorId);if(next)box.scrollTop+=next.getBoundingClientRect().top-top-offset;}
    }
  }
  function renderTimeline(){const relevant=new Set(['transcript','speech','voice','interaction','extension_result','scene_answer','fire_selection_ack']);const events=liveEvents().filter(e=>relevant.has(eventKey(e))||['/voice/transcript','/speech/say','/voice/status','/interaction/status','/scene/answer','/fire/selection_ack'].includes(e.topic)).slice(-35).reverse();html('eventTimeline',events.length?events.map(e=>`<div class="timeline-entry"><div class="event-head"><strong>${esc(labels[eventKey(e)]||e.kind||'车载事件')}</strong><time>${esc(clock(e.at))}</time></div><p>${esc(eventContent(e))}</p><small>${esc(e.topic||'')}</small></div>`).join(''):'<div class="empty-note">等待车载事件；每次请求与回执会在这里留痕。</div>');}
  function renderEvidence(){const ros=state.data?.ros||{},fp=data('fpga_link'),a=data('actuators'),st=data('stereo');text('domainLabel','ROS DOMAIN '+(ros.domain_id??'—'));const facts=[['ROS 订阅连接',connected()&&ros.available===true?'已连接':connected()?reason(ros.error||'未连接'):'上位机连接中断，状态未知'],['任务路由订阅者',connected()&&ros.router_ready===true?'已发现':'未就绪 / 待新鲜连接'],['UART / FPGA',fresh('fpga_link')?(fp.dry_run===true?'dry-run  不连接真车':fp.status_fresh===true&&!fp.error?'有新鲜链路状态':'待连接'):'尚无新鲜反馈'],['执行器输出',fresh('actuators')?(a.dry_run===true?(a.manual_pump_output_enabled===true?'云台 dry-run · 手动水泵入口已开':'dry-run  无真实输出'):a.hardware_written===true?'已写入硬件指令':'未确认硬件输出'):'待反馈'],['双目测距',stereoDescription()],['云台实际角度',Array.isArray(a.actual_angles_deg)?a.actual_angles_deg.join('° / ')+'°':'无实测角度反馈'],['预览服务',state.data?.preview?.upstream_available===true?'上游已连接':'等待上游'],['数据来源',connected()?'实时车载话题与状态':'历史车载数据 / 当前已断开']];html('systemFacts',facts.map(([k,v])=>`<div class="system-fact"><span>${esc(k)}</span><strong>${esc(v)}</strong></div>`).join(''));
    const commands=(state.data?.commands||[]).slice(-12).reverse();const statusNames={queued:'排队中',submitted:'已提交 ROS',observed:'收到测试反馈',receipt:'收到任务回执',execution:'收到执行节点回执',execution_timeout:'执行节点回执超时',receipt_timeout:'回执超时',expired:'已过期',rejected:'未接受'};html('receiptList',commands.length?commands.map(c=>{const accepted=c.execution?.accepted??c.receipt?.accepted;return`<div class="receipt"><div class="receipt-title"><strong>${esc(c.text||c.action)}</strong><span class="${accepted===false||c.status==='rejected'?'error':accepted===true?'ok':''}">${esc(c.execution?(accepted===false?'执行未接受':c.execution.completed===true?'节点处理完成':'执行节点已接收'):c.status==='execution_timeout'?'执行节点回执超时':accepted===false?'路由未接受':accepted===true?'路由已接受':statusNames[c.status]||c.status)}</span></div><p>${esc(c.execution?executionResultStatus(c.execution,c.action).text:reason(c.reason||c.receipt?.reason||'请结合执行器、导航与任务状态检查实际结果'))}</p><small>${esc(c.command_id||c.request_id||'')}  ${esc(clock(c.queued_at))}</small></div>`;}).join(''):'<div class="empty-note">暂无上位机任务回执</div>');
    const entries=Object.entries(state.data?.topics||{}).filter(([key])=>!['road','road_preview'].includes(key));text('topicCount',entries.length+' 个话题');html('topicRows',entries.map(([key,t])=>{const ok=fresh(key),has=t.data!==null&&t.data!==undefined;const age=finite(t.age_s)?t.age_s+(performance.now()-state.received)/1000:null;return`<tr><td>${esc(labels[key]||key)}</td><td>${esc(t.topic||'')}</td><td><span class="topic-status ${ok?'fresh':has?'stale':'missing'}">${ok?' 新鲜':has?' 历史 / 已过期':'— 未收到'}</span></td><td>${age===null?'—':esc(num(age,2))+' s'}</td><td><button class="text-btn" data-evidence="${esc(key)}">查看 JSON </button></td></tr>`;}).join(''));
  }
  $('topicRows').addEventListener('click',event=>{const btn=event.target.closest('[data-evidence]');if(!btn)return;const key=btn.dataset.evidence;text('evidenceTitle',labels[key]||key);text('evidenceJson',JSON.stringify(topic(key),null,2));$('evidenceDialog').showModal();});
  $('exportBtn').addEventListener('click',()=>{if(!state.data){toast('尚无可导出的状态证据',true);return;}const payload={exported_at:new Date().toISOString(),synthetic:false,source:'Rescue live control console',connected:connected(),note:'车载证据快照；请核对消息时间与新鲜度。任务接受不等于动作完成；水泵输出不等于真实出水。',state:state.data};const blob=new Blob([JSON.stringify(payload,null,2)],{type:'application/json;charset=utf-8'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='rescue-evidence-'+new Date().toISOString().replace(/[:.]/g,'-')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),2000);});
  function renderBuildings() {
    const configured=Array.isArray(state.data?.buildings)?state.data.buildings:[];
    const buildings=configured.filter(b=>b&&/^0*[1-9][0-9]*$/.test(String(b.number))&&Number.isSafeInteger(Number(b.number)))
      .map(b=>({...b,number:String(Number(b.number))}));
    const key=JSON.stringify(buildings);if(key===state.buildingsKey)return;state.buildingsKey=key;
    const selected=$('buildingSelect').value;
    $('buildingSelect').innerHTML=buildings.length?buildings.map(b=>`<option value="${esc(b.number)}">${esc(b.number)} 号楼${b.name?' · '+esc(b.name):''}</option>`).join(''):'<option value="">未加载车载楼栋配置</option>';
    if(buildings.some(b=>String(b.number)===selected))$('buildingSelect').value=selected;
  }
  const motorProfiles={motor1:{name:'电机 1 · PD42 · 俯仰',code:'M1',protocol:'pd42',tx:8,rx:10,port:'/dev/ttyS1'},motor2:{name:'电机 2 · ZDT_XS · 水平',code:'M2',protocol:'zdt_xs',tx:3,rx:5,port:'/dev/ttyS3'}};
  const motorUI=Object.fromEntries(Object.keys(motorProfiles).map(id=>[id,{pending:false,pendingAction:null,requestId:null,requestAction:null,waitingSince:0,notice:'',generation:0,lease:null,lastOwnedRunId:null}]));
  const motorActionNames={connect:'连接并读取',read:'刷新读数',enable:'使能电机',run_positive:'持续正转',run_negative:'持续反转',step_positive:'正向 5°',step_negative:'反向 5°',return_origin:'回到本次起点',stop:'停止并失能',disconnect:'断开串口'};
  const motorReasonNames={not_connected:'尚未连接电机',connected:'通信已建立；已记录本次起点',enabled:'已使能，可进行空载点动',target_reached:'编码器确认到位',motion_completed:'编码器确认到位',disabled:'已失能，保持力矩已释放',disconnected:'串口已断开',feedback_stale:'电机反馈已过期',move_timeout:'移动超时，已请求停止',limit_exceeded:'超出本次起点 ±15° 范围',session_limit_exceeded:'超出本次起点 ±15° 范围',enable_required:'请先点击使能电机',lease_expired:'测试使能已到期，请重新使能',busy:'电机正在执行上一项测试',idle:'等待手动连接',motor_test_node_unavailable:'测试节点尚未就绪'};
  Object.assign(motorReasonNames,{motor_test_disconnected:'串口未连接；连接只读，不自动转动',motor_test_connected_read_only:'通信成功，本次起点已记录；点击使能后才可点动',motor_test_enabled_30s:'已使能，30 秒内可点动；空闲到期后自动失能',motor_test_waiting_measured_arrival:'驱动已接收目标，正在等待编码器确认到位',motor_test_measured_arrival:'编码器位置与停止状态已确认到位',motor_test_disabled_no_holding_torque:'已确认失能，电机保持力矩已释放',motor_test_feedback_read:'已收到真实位置、状态与电压反馈',motor_test_enable_lease_expired:'本次使能已到期并失能，需要时重新点击使能',motor_test_explicit_enable_required:'请先点击使能电机，建立本次点动会话',motor_test_idle_disconnected:'空闲超过 120 秒，已结束测试并断开',motor_test_15_degree_limit:'已到本次 ±15° 范围边界；可反向点动或回到本次起点',motor_test_not_connected:'先连接并读取电机',motor_test_already_connected:'串口已经连接',motor_test_requires_stopped_feedback:'等待真实零转速与停止状态',motor_test_connect_requires_stopped_motor:'电机尚未停稳，未记录本次起点',motor_test_enabled_feedback_required:'等待电机已使能的真实反馈',motor_test_move_in_progress:'上一项点动尚未完成',motor_test_fault_reconnect_required:'测试发生异常；先断开并检查，再重新连接',motor_test_connect_failed:'没有建立有效 PD42 通信，请检查供电、接线和驱动菜单',motor_test_interlock_lost:'其他任务或反馈状态发生变化，测试已停止',motor_test_driver_fault:'驱动报告堵转、过载或欠压',motor_test_arrival_timeout:'未在时限内确认到位，已请求失能',motor_test_requires_idle_mission:'请先结束巡逻、导航或自动灭火，再做空载测试',motor_test_mission_busy_or_estop:'当前交通、灭火或急停状态不允许点动',motor_test_production_actuator_active:'正式云台或水泵控制已启用，不允许测试争用',motor_test_fpga_uart_active:'FPGA 串口正在使用，当前测试被阻止',motor_test_system_status_stale:'任务、执行器或 FPGA 状态尚未更新',motor_test_subscriber_unavailable:'电机测试节点尚未启动',motor_test_worker_busy:'串口正在处理上一条请求，请等待结果',motor_test_queued:'请求已接收，等待串口执行',motor_test_queued_stop:'停止请求已接收，等待执行',motor_test_superseded_by_stop:'该请求已被停止操作取消',expired_motor_test_request:'请求已过期，未执行；不会自动重发',duplicate_motor_test_request:'重复请求未再次执行'});
  Object.assign(motorReasonNames,{different_motor_connected:'另一台电机仍连接，请先断开后切换',motor_test_different_motor_connected:'另一台电机仍连接，请先断开后切换',motor_test_other_motor_busy:'另一台电机请求正在处理中，请等待完成后切换',invalid_motor_id:'不支持这台电机，未下发请求',invalid_motor_test_motor_id:'不支持这台电机，未下发请求'});
  motorReasonNames.motor_test_connect_failed='没有建立有效通信，请检查所选电机的协议、供电、接线和驱动菜单';
  Object.assign(motorReasonNames,{zdt_configuration_not_read:'尚未读取 ZDT_XS 驱动参数，请先连接并读取',zdt_serial_configuration_mismatch:'ZDT_XS 串口模式、波特率或校验方式与配置不一致',zdt_command_ack_required:'请将驱动命令应答设置为 Receive 或 Both 后重新连接',zdt_closed_loop_required:'点动需要驱动处于闭环模式，请核对菜单后重新连接',zdt_input_scale_unknown:'尚未确认输入缩放方式，暂不能换算点动目标',zdt_position_scale_unknown:'尚未确认细分或电机步距角，暂不能换算点动目标',zdt_motion_parameters_unverified:'驱动点动参数未通过核对，请检查模式、步距角与缩放设置',zdt_step_resolution_too_coarse:'当前参数无法表示所需小角度，请检查细分与缩放设置',zdt_firmware_mode_mismatch:'驱动配置与选项报告的固件模式不一致，请检查后重新连接',zdt_configuration_changed_reconnect:'驱动菜单参数已变化，请断开后重新连接'});
  labels.motor_test='双串口空载电机测试';
  Object.assign(motorReasonNames,{
    zdt_undocumented_options:'驱动器新增选项尚未完整适配，不是串口断开',
    zdt_options_layout_unverified:'驱动参数格式尚未确认，保留只读反馈',
    zdt_vendor_position_only:'当前只开放已实测的小角度位置测试；连续旋转未开放',
    zdt_vendor_target_mismatch:'当前位置与驱动保留目标不一致，未使能追赶旧目标',
    zdt_retained_target_mismatch:'当前位置与驱动保留目标未稳定一致，暂不使能；不会自动追赶旧目标',
    zdt_limited_position_only:'当前只开放 0.9° 位置测试，连续旋转未开放',
    zdt_limited_position_boundary:'已到本次 0.9° 测试边界，请先回到起点',
  });
  function motorReason(value){return motorReasonNames[value]||reason(value)||'等待反馈';}
  function motorAccessView(){
    const online=connected(),a=data('actuators'),control=state.data?.control||{};
    const production=online&&fresh('actuators')&&a.dry_run===false&&a.enabled===true;
    const blocked=online&&control.motor_test_ready!==true?motorReason(control.motor_test_reason||'motor_test_node_unavailable'):'';
    const offline='与 X5 的整车连接已失效，无法确认电机通信状态；请先恢复车载服务连接。';
    return {online,production,blocked:online?blocked:offline,
      title:!online?'整车连接失效':production?'正式云台正在管理两路电机串口':blocked?'空载调试暂不可用':'空载调试可用',
      detail:!online?offline:production?'正式云台控制占用两路串口，当前显示的是空载调试页。请使用“云台到角度”查看实际角度、故障原因并提交控制。'+(blocked?' 当前限制：'+blocked:''):blocked,
      visible:!online||production||Boolean(blocked)};
  }
  function motorActionLabel(id,action){
    const m=motorState(id).m,step=finite(m.step_deg)&&m.step_deg>0?m.step_deg:5;
    return action==='step_positive'?'正向 '+step+'°':action==='step_negative'?'反向 '+step+'°':motorActionNames[action]||action;
  }
  Object.assign(motorReasonNames,{motor_test_continuous_waiting_speed:'驱动已接受速度指令，等待实测转速',motor_test_continuous_running:'实测转速已确认，正在连续运行',motor_test_keepalive_expired:'控制台续租中断，已请求停止并失能',motor_test_continuous_requires_stop:'请先停止并失能，再重新使能选择方向'});
  function motorState(id){
    const topLive=connected()&&fresh('motor_test'),top=topLive?data('motor_test'):{},value=top.motors?.[id]??((top.motor_id||'motor1')===id?top:{});
    const now=(state.data?.server_time||0)+(performance.now()-state.received)/1000;
    const live=Boolean(topLive&&value&&value.synthetic!==true&&(!top.motors||finite(value.stamp)&&now-value.stamp>=-.05&&now-value.stamp<=1));
    return {top,live,m:live?value:{},feedback:live&&value.connected===true&&value.feedback_fresh===true};
  }
  function motorReceipt(id,view){
    const ui=motorUI[id],{top,m,live}=view;
    if(!live)return null;
    if(ui.requestId){
      const matches=(top.recent_requests||[]).filter(r=>r.request_id===ui.requestId&&(r.motor_id||'motor1')===id&&r.action===ui.requestAction);
      const terminal=matches.findLast?.(r=>['completed','rejected','error'].includes(r.result));
      const recent=terminal||matches.at(-1);
      if(recent)return {last_request_id:recent.request_id,last_action:recent.action,last_result:recent.result,reason:recent.reason};
      if(m.last_request_id===ui.requestId&&(m.last_motor_id||m.motor_id||'motor1')===id&&m.last_action===ui.requestAction)return m;
      return null;
    }
    return (m.last_motor_id||m.motor_id||'motor1')===id?m:null;
  }
  function motorActionAllowed(id,action){
    const ui=motorUI[id],view=motorState(id),m=view.m;
    if(['stop','disconnect'].includes(action))return connected()&&state.data?.ros?.motor_test_ready===true;
    if(!connected()||state.data?.control?.motor_test_ready!==true||!view.live||ui.pending)return false;
    const waiting=ui.requestId&&!motorReceipt(id,view)&&performance.now()-ui.waitingSince<5000;
    if(waiting||m.busy===true)return false;
    if(action==='connect')return m.connected!==true;
    if(action==='read')return m.connected===true;
    if(!view.feedback)return false;
    if(action==='enable')return m.can_enable===true;
    if(['run_positive','run_negative'].includes(action))return m.can_run===true&&!ui.lease&&!m.continuous_running&&!m.continuous_pending;
    return m.can_move===true&&!ui.lease&&!m.continuous_running&&!m.continuous_pending;
  }
  function renderMotorTest(){
    const access=motorAccessView();
    text('motorServiceBadge',!access.online?'整车连接失效':access.blocked|| (state.data?.ros?.motor_test_ready?'电机测试节点在线':'等待测试节点'));
    $('motorControlNotice').hidden=!access.visible;$('motorControlNotice').dataset.state=!access.online?'offline':access.production?'production':'blocked';
    text('motorControlTitle',access.title);text('motorControlDetail',access.detail);
    $('motorStopAll').disabled=!(connected()&&state.data?.ros?.motor_test_ready);
    for(const [id,profile] of Object.entries(motorProfiles)){
      const ui=motorUI[id],view=motorState(id),{m,live,feedback}=view,receipt=motorReceipt(id,view);
      const waiting=ui.requestId&&!receipt,waitingText=waiting?(performance.now()-ui.waitingSince<5000?'已提交，等待电机节点反馈':'未收到匹配回执，不自动重发'):'';
      const protocol=m.protocol||profile.protocol,isZdt=protocol==='zdt_xs',mode={emm:'EMM',x:'X'}[m.firmware_mode];
      const model=protocol==='pd42'?'PD42':isZdt?'ZDT_XS':'协议待确认';
      text(id+'Protocol',model+(isZdt?' · '+(mode?mode+' 模式':'模式待确认'):'')+' · '+profile.port);
      const limited=m.limited_position_only===true&&m.motion_profile==='zdt_vendor_fd8_bench';
      text(id+'Configuration',!live?'等待新鲜车载参数':limited?'已适配 0.9° 位置测试 · '+(m.position_test_reason?motorReason(m.position_test_reason):'本次范围 ±0.9°；连续旋转未开放'):m.motion_configuration_valid===false?'参数未就绪：'+motorReason(m.motion_configuration_reason):m.motion_configuration_valid===true?'参数有效 · 控制权限以实时反馈为准':isZdt?'等待核对驱动模式与参数':'原生协议 · 角度由车端换算');
      $(id+'Configuration').title=m.motion_configuration_detail||'';
      $(id+'Configuration').dataset.state=m.motion_configuration_valid===false?'warn':m.motion_configuration_valid===true?'ok':'idle';
      text(id+'Raw',feedback?(isZdt?'ZDT 原始位置':'PD42 位置计数')+' '+(Number.isInteger(m.position_counts)?m.position_counts:'—')+' · 起点 '+(Number.isInteger(m.reference_counts)?m.reference_counts:'—'):'暂无有效位置读数');
      text(id+'Link',!access.online?'整车连接失效':!live?'调试状态未知':m.connected?'空载测试已连接':'空载测试未连接');
      text(id+'Angle',feedback&&finite(m.position_deg)?num(m.position_deg,2)+'°':'—');
      text(id+'Speed',feedback&&finite(m.speed_rpm)?num(m.speed_rpm,1)+' RPM':'—');
      text(id+'Voltage',feedback&&finite(m.voltage_v)?num(m.voltage_v,2)+' V':'—');
      text(id+'Enabled',feedback?(m.enabled===true?'已使能':m.enabled===false?'未使能':'使能未知'):'使能未知');
      const states={0:'停止',1:'完成',2:'运行',3:'过载',4:'堵转',5:'欠压',STOPPED:'停止',COMPLETED:'完成',RUNNING:'运行',OVERLOAD:'过载',STALLED:'堵转',UNDERVOLTAGE:'欠压'};
      const running=feedback&&m.continuous_running===true;
      text(id+'RunState',!live?'状态未知':!m.connected?'等待连接':m.continuous_pending?'等待实测转速':running?(m.continuous_direction===-1?'持续反转':m.continuous_direction===1?'持续正转':'连续运行 · 方向待确认'):feedback?(states[m.run_state]??m.run_state??'状态待确认'):'等待实测反馈');
      $(id+'Card').dataset.running=String(running);
      text(id+'Lease',limited?'位置指令 + 编码器确认 · 等待到位后再点下一步':ui.lease?(running&&m.run_id===ui.lease.runId?'本页保持运行 · 失联保护 '+(finite(m.keepalive_remaining_s)?num(m.keepalive_remaining_s,1)+'s':'待反馈'):'等待确认本次启动'):running?(m.run_id===ui.lastOwnedRunId?'本页已停止续租 · 等待实测停止':'其他控制端启动 · 本页不续租'):'本页尚未启动连续运行');
      const receipts={accepted:'已接受，等待实测',completed:'已完成',rejected:'未执行',error:'执行异常'};
      text(id+'Receipt',ui.pending?'正在提交':waitingText||(receipt?.last_action?motorActionLabel(id,receipt.last_action)+' · '+(receipts[receipt.last_result]||receipt.last_result||'等待反馈'):'尚未下发测试'));
      text(id+'Reason',access.blocked||ui.notice||(!live?'等待新鲜状态；此页面不会自动启动电机。':m.error?motorReason(m.reason)+'：'+String(m.error):motorReason(receipt?.reason||m.reason)));
      text(id+'RequestId',receipt?.last_request_id?'回执：'+receipt.last_request_id:'PIN '+profile.tx+' / '+profile.rx+' · 地址 1 · 115200');
    }
    document.querySelectorAll('[data-motor-action]').forEach(button=>{
      const id=button.dataset.motorId,action=button.dataset.motorAction,m=motorState(id).m;
      if(['step_positive','step_negative'].includes(action))button.textContent=motorActionLabel(id,action)+' 点动';
      button.disabled=!motorActionAllowed(id,action);
      button.title=!access.online?access.blocked:button.disabled&&!['stop','disconnect'].includes(action)&&access.blocked?access.blocked:m.limited_position_only&&['run_positive','run_negative'].includes(action)?motorReason('zdt_vendor_position_only'):button.disabled&&!['stop','disconnect'].includes(action)?motorReason(m.position_test_reason||m.motion_configuration_reason||m.reason||'等待通信与真实反馈'):'';
    });
  }
  function clearMotorLease(id){const ui=motorUI[id];if(ui.lease)ui.lease.active=false;ui.lease=null;}
  async function submitMotorTest(id,action){
    if(!Object.hasOwn(motorProfiles,id)||!Object.hasOwn(motorActionNames,action))return;
    const ui=motorUI[id],stopping=['stop','disconnect'].includes(action),running=['run_positive','run_negative'].includes(action);
    if(stopping)clearMotorLease(id); // Even a failed stop request must stop renewing the previous run.
    if(!motorActionAllowed(id,action)){if(stopping)ui.notice='停止请求未确认；已停止本页续租，板端失联保护将停止电机。';renderMotorTest();return;}
    const generation=++ui.generation;ui.pending=true;ui.pendingAction=action;ui.notice='';renderMotorTest();
    try{
      const response=await request('/api/motor-test',{action,request_id:uuid(),motor_id:id});
      if(generation!==ui.generation)return;
      if(typeof response.command_id!=='string'||!response.command_id)throw Error('缺少请求回执 ID');
      ui.requestId=response.command_id;ui.requestAction=action;ui.waitingSince=performance.now();
      if(running){
        if(state.view!=='motor'||state.devicePane!=='motors'||document.hidden||!connected())throw Error('控制台已离开测试页');
        ui.lease={runId:response.command_id,startedAt:performance.now(),nextAt:performance.now(),busy:false,active:true,observedRunning:false};
        ui.lastOwnedRunId=response.command_id;
      }
      toast('已提交'+motorProfiles[id].name+'：'+motorActionLabel(id,action)+'，请观察实测反馈。');
    }catch(error){
      if(generation===ui.generation){clearMotorLease(id);ui.notice='请求未确认：'+error.message+'；不会自动重发。';if(running)bestEffortMotorStop(id);toast(ui.notice,true);}
    }finally{if(generation===ui.generation){ui.pending=false;ui.pendingAction=null;renderMotorTest();}}
  }
  async function stopAllMotors(){
    for(const id of Object.keys(motorProfiles))clearMotorLease(id);
    return Promise.allSettled(Object.keys(motorProfiles).map(id=>submitMotorTest(id,'stop')));
  }
  function bestEffortMotorStop(id){
    const body=JSON.stringify({action:'stop',request_id:uuid(),motor_id:id});
    try{if(typeof navigator!=='undefined'&&navigator.sendBeacon?.('/api/motor-test',new Blob([body],{type:'application/json'})))return;}catch{}
    try{void fetch('/api/motor-test',{method:'POST',headers:{'Content-Type':'application/json'},body,keepalive:true,cache:'no-store'}).catch(()=>{});}catch{}
  }
  function endMotorRuns(message){
    for(const id of Object.keys(motorProfiles)){
      const ui=motorUI[id];
      if(!ui.lease&&!['run_positive','run_negative'].includes(ui.pendingAction))continue;
      clearMotorLease(id);ui.generation++;ui.pending=false;ui.pendingAction=null;
      ui.notice=message+'；已停止续租并请求停止，不会自动恢复。';bestEffortMotorStop(id);
    }
  }
  async function maintainMotorRuns(){
    if(state.view!=='motor'||state.devicePane!=='motors'||document.hidden||!connected()){
      endMotorRuns(!connected()?'控制台连接中断':'已离开电机测试页');return;
    }
    await Promise.allSettled(Object.keys(motorProfiles).map(async id=>{
      const ui=motorUI[id],lease=ui.lease;if(!lease||!lease.active||lease.busy)return;
      const view=motorState(id),m=view.m,receipt=motorReceipt(id,view),now=performance.now();
      const failed=receipt&&receipt.last_request_id===lease.runId&&['rejected','error'].includes(receipt.last_result);
      const confirmed=view.live&&view.feedback&&m.continuous_running===true&&m.run_id===lease.runId;
      if(!view.live||failed||lease.observedRunning&&!confirmed||!confirmed&&now-lease.startedAt>=5000){
        clearMotorLease(id);ui.notice=failed?'本次持续运行未获执行，已停止续租。':'未能持续确认本次实测运行，已停止续租并请求停止。';bestEffortMotorStop(id);return;
      }
      if(!confirmed||now<lease.nextAt)return;
      lease.observedRunning=true;lease.busy=true;lease.nextAt=now+1000;
      try{await request('/api/motor-heartbeat',{motor_id:id,run_id:lease.runId});}
      catch(error){if(ui.lease===lease){clearMotorLease(id);ui.notice='续租未确认：'+error.message+'；已请求停止，不自动恢复。';bestEffortMotorStop(id);}}
      finally{lease.busy=false;}
    }));
    renderMotorTest();
  }
  // MOTOR_TEST_EVENTS: listeners stay outside the offline logic-test fragment.
  document.querySelectorAll('[data-motor-action]').forEach(button=>button.addEventListener('click',()=>submitMotorTest(button.dataset.motorId,button.dataset.motorAction)));
  $('motorOpenGimbal').addEventListener('click',()=>selectDevicePane('gimbal'));
  $('motorStopAll').addEventListener('click',()=>stopAllMotors());
  setInterval(()=>{void maintainMotorRuns();},1000);
  function renderAll(){renderExtension();renderPointNavigation();renderStatus();renderRoad();renderBuilding();renderBuildings();renderNavigation();renderFire();renderVoice();renderEvidence();renderControls();renderMotorTest();renderGimbalFeedback();renderManualGimbal();renderFpgaMap();if($('fireSetupDialog').open)renderFireSetup();if($('pumpManualDialog').open)renderManualPump();refreshCenteringOverlay();}
  async function pollState(){
    const requestStarted=performance.now();
    try{
      const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(2500)});if(!response.ok)throw new Error('车载服务 HTTP '+response.status);
      const result=await response.json();
      if(result.schema_version!==1||!result.topics)throw new Error('车载服务协议不兼容，请更新 X5 上位机服务');
      if(result.mode!=='live'||result.synthetic!==false){
        state.data=null;state.pendingAction=null;if($('actionDialog').open)$('actionDialog').close();
        state.frameEpoch++;stopFrame(true);stopRoadFrames();state.bitmap?.close();state.bitmap=null;state.frame=null;context.clearRect(0,0,canvas.width,canvas.height);
        throw new Error('已拒绝非实时车载数据，请连接 X5 实车服务');
      }
      state.data=result;state.received=performance.now();state.observationTransportS=Math.max(0,(state.received-requestStarted)/1000);state.connected=true;state.connectionError='';
      if(result.preview?.enabled!==undefined&&state.stream!==Boolean(result.preview.enabled)){state.stream=Boolean(result.preview.enabled);$('streamToggle').checked=state.stream;state.frameEpoch++;stopFrame(true);stopRoadFrames();}
      renderAll();if(state.stream&&!state.frameBusy)scheduleFrame(0);if(!roadView.busy)scheduleRoadFrame(0);showRoadFrameState();
    }catch(error){state.connected=false;state.connectionError=error.message||'连接中断';state.frameEpoch++;stopFrame(true);stopRoadFrames();showRoadFrameState();renderAll();}
    finally{setTimeout(pollState,document.hidden?1800:600);}
  }

  function unpack(buffer){if(buffer.byteLength<4)throw new Error('图像数据不完整');const size=new DataView(buffer).getUint32(0,false);if(size<2||size>4*1024*1024||size+4>buffer.byteLength)throw new Error('图像元数据长度无效');const metadata=JSON.parse(decoder.decode(new Uint8Array(buffer,4,size)));if(metadata.transport_version!==1)throw new Error('预览协议不兼容');if(metadata.synthetic===true||metadata.frame?.synthetic===true||metadata.mode==='demo')throw new Error('拒绝非相机实时画面');const jpeg=new Blob([new Uint8Array(buffer,4+size)],{type:'image/jpeg'});if((metadata.frame===null&&jpeg.size)||(metadata.frame!==null&&!jpeg.size))throw new Error('图像和元数据不一致');return{metadata,jpeg};}
  function drawFrame(overlay,probe){if(!state.bitmap||!state.frame)return;const frame=state.frame;if(canvas.width!==frame.width||canvas.height!==frame.height){canvas.width=frame.width;canvas.height=frame.height;}context.drawImage(state.bitmap,0,0,canvas.width,canvas.height);const font=Math.max(18,Math.round(canvas.width/47));context.font=`600 ${font}px Bahnschrift, "Microsoft YaHei", sans-serif`;const lineWidth=Math.max(2,canvas.width/360);if(frame.detector_available!==false)for(const d of frame.detections||[]){if(!Array.isArray(d.bbox_xyxy)||d.bbox_xyxy.length!==4||!d.bbox_xyxy.every(finite)||!finite(d.confidence))continue;const[x1,y1,x2,y2]=d.bbox_xyxy,x=Math.max(0,Math.min(frame.width,x1)),y=Math.max(0,Math.min(frame.height,y1)),w=Math.max(0,Math.min(frame.width,x2)-x),h=Math.max(0,Math.min(frame.height,y2)-y);if(!w||!h)continue;const color=colors[d.class_name]||'#d5e4e9',label=(names[d.class_name]||d.class_name)+' '+num(d.confidence*100,0)+'%',tw=Math.min(frame.width,context.measureText(label).width+16),th=font+12,lx=Math.min(x,frame.width-tw),ly=y>=th?y-th:Math.min(frame.height-th,y+3);context.strokeStyle=color;context.lineWidth=lineWidth;context.globalAlpha=.55;context.strokeRect(x,y,w,h);context.globalAlpha=1;const corner=Math.max(10*(canvas.width/1280),Math.min(28*(canvas.width/1280),Math.min(w,h)*.22));context.lineWidth=lineWidth*1.6;context.beginPath();context.moveTo(x,y+corner);context.lineTo(x,y);context.lineTo(x+corner,y);context.moveTo(x+w-corner,y);context.lineTo(x+w,y);context.lineTo(x+w,y+corner);context.moveTo(x+w,y+h-corner);context.lineTo(x+w,y+h);context.lineTo(x+w-corner,y+h);context.moveTo(x+corner,y+h);context.lineTo(x,y+h);context.lineTo(x,y+h-corner);context.stroke();context.fillStyle=color;context.fillRect(lx,ly,tw,th);context.fillStyle='#0B0D10';context.fillText(label,lx+8,ly+font+1);}drawCenteringOverlay(frame,overlay);drawStereoProbeOverlay(frame,probe);}
  // CENTERING_OVERLAY_MODEL: geometry comes only from this image's registry.
  function exactFrameRegistry(frame,registry) {
    if(!frame||!registry||typeof frame.token!=='string'||registry.frame_token!==frame.token||
        typeof registry.registry_session!=='string'||!registry.registry_session||
        typeof registry.coordinate_frame!=='string'||!registry.coordinate_frame||
        registry.coordinate_frame!==frame.coordinate_frame||
        !Array.isArray(registry.image_size_wh)||registry.image_size_wh.length!==2||
        registry.image_size_wh[0]!==frame.width||registry.image_size_wh[1]!==frame.height||
        !Array.isArray(registry.targets))return null;
    return registry;
  }
  function applyFrameRegistryUpdate(frame,registry,key='fire_registry') {
    return exactFrameRegistry(frame,registry)?{...frame,[key]:registry}:frame;
  }
  function centeringOverlayModel(frame,input) {
    const e=input.extension||{},host=e.fpga_host||{},r=e.region_search||{},c=r.centering||{};
    const gimbalOnly=false;
    const hidden={visible:false,boxes:[],status:''};
    if(input.extensionFresh!==true||input.frameFresh!==true||e.real_uart!==true||e.status_fresh!==true||
        !['dialogue','semantic'].includes(e.task)||
        !['selected','scope','link_ok','fresh'].every(key=>host[key]===true)||c.enabled!==true)return hidden;
    const tolerance=Number.isFinite(c.center_tolerance_norm)&&c.center_tolerance_norm>=.01&&c.center_tolerance_norm<=.08?c.center_tolerance_norm:.05;
    const taskView=regionTaskPresentation(r);
    if(r.command_id&&taskView.terminal){
      const outcome=regionTargetOutcome(r);
      return {visible:true,tolerance,showCenterGuide:!gimbalOnly,boxes:[],status:taskView.title+(taskView.detail?' · '+taskView.detail:'')+
        (outcome.detail?' · '+outcome.detail:'')};
    }
    const elapsed=Number.isFinite(input.elapsed)&&input.elapsed>=0?input.elapsed:0;
    const phase=r.active===true?r.phase:'IDLE';
    const labels={IDLE:'等待区域搜索',ENTERING:'前往搜索区域',NAVIGATING:'沿路线巡检',EXITING:'前往出口 · 继续检测',
      WAIT_STOP:'发现消防员 · 停车准备对齐，尚未完成',PAUSING:'停车准备对齐 · 等待实际停稳',
      CENTER_COLLECT:'停稳后确认本批候选，尚未完成居中',CENTER_SELECT:'选择下一消防员',
      CENTER_MOVING:'正在对齐',CENTER_STOP:'已请求停车 · 等待停稳',CENTER_DWELL:'已停稳居中',
      CENTER_REACQUIRE:'返回原观察位置重找队列消防员',CENTER_RETURN:'回到原路线暂停位置',WAIT_SPRAY_RANGE:'核对喷水距离',EXTINGUISHING:'瞄准并短喷',WAIT_PUMP_OFF:'等待实际关水'};
    if(gimbalOnly)Object.assign(labels,{WAIT_STOP:'发现稳定消防员 · 等待实际停稳',PAUSING:'已请求停车 · 等待实际停稳',
      CENTER_COLLECT:'车辆保持停稳 · 确认本批消防员',CENTER_DWELL:'车辆保持停稳 · 确认目标以便云台瞄准',
      EXTINGUISHING:'车辆保持停稳 · 云台瞄准与短喷',WAIT_PUMP_OFF:'确认当前目标关水 · 尚未完成整批'});
    const candidateRegistry=exactFrameRegistry(frame,frame?.firefighter_registry);
    const registry=candidateRegistry?.target_class==='firefighter'?candidateRegistry:null,boxes=[];
    const queue=Array.isArray(c.queue)?c.queue:[];
    const session=c.current_registry_session,sessionMatches=!session||registry?.registry_session===session;
    const validTarget=row=>Number.isInteger(row?.target_id)&&row.target_id>0&&row.selectable===true&&
      Number.isFinite(row.confidence)&&row.confidence>=.9&&row.confidence<=1&&
      Array.isArray(row.bbox_xyxy)&&row.bbox_xyxy.length===4&&row.bbox_xyxy.every(Number.isFinite)&&
      0<=row.bbox_xyxy[0]&&row.bbox_xyxy[0]<row.bbox_xyxy[2]&&row.bbox_xyxy[2]<=frame.width&&
      0<=row.bbox_xyxy[1]&&row.bbox_xyxy[1]<row.bbox_xyxy[3]&&row.bbox_xyxy[3]<=frame.height;
    const rows=registry&&sessionMatches&&frame.detector_available!==false?registry.targets.filter(validTarget):[];
    const currentRow=rows.find(row=>row.target_id===c.current_target_id);
    const centeredInFrame=Boolean(currentRow&&Math.abs((currentRow.bbox_xyxy[0]+currentRow.bbox_xyxy[2])/(2*frame.width)-.5)<=tolerance);
    if(!gimbalOnly&&phase==='CENTER_DWELL'&&!centeredInFrame)labels.CENTER_DWELL=currentRow?'当前画面偏离中线，等待任务更新':'等待当前目标同帧居中证据';
    if(phase==='CENTER_SELECT'&&(r.reason||c.reason)==='rechecking_queued_flame_at_original_view')labels.CENTER_SELECT='已回原观察位置，等待队列消防员重现';
    if(phase==='CENTER_SELECT'&&r.reason==='waiting_new_batch_candidate_confirmation')labels.CENTER_SELECT='等待本批新候选连续多帧确认';
    const movement={forward:'前进对齐',reverse:'后退对齐',stopped:'车辆停止'};
    const parts=[labels[phase]||'任务阶段待更新'];
    const holdText=searchPumpRecoveryText(r);
    if(holdText)parts.splice(0,parts.length,holdText);
    if(r.execute_fire===false&&r.visual_degraded===true)parts.push('视觉暂缺 · 取消本次对中，恢复原路线');
    if(!holdText&&r.active===true&&phase==='CENTER_MOVING')parts.push(movement[c.move_direction]||'等待方向反馈');
    if(!holdText&&r.active===true&&['CENTER_RETURN','CENTER_REACQUIRE'].includes(phase)&&['forward','reverse'].includes(c.move_direction))
      parts.push(c.move_direction==='forward'?'前进返回':'后退返回');
    if(!gimbalOnly&&!holdText&&r.active===true&&c.active===true&&phase==='CENTER_DWELL'&&centeredInFrame&&Number.isFinite(c.dwell_remaining_s)&&c.dwell_remaining_s>=0&&c.dwell_remaining_s<=1)
      parts.push('居中剩余 '+Math.max(0,c.dwell_remaining_s-elapsed).toFixed(1)+' 秒');
    const batchActive=r.active===true&&c.active===true&&!['WAIT_STOP','PAUSING','ENTERING','NAVIGATING','EXITING'].includes(phase);
    if(batchActive&&typeof c.current_batch_id==='string'&&/^batch-\d+$/.test(c.current_batch_id))
      parts.push('本批目标 '+c.current_batch_id.slice(6));
    if(batchActive&&phase==='CENTER_COLLECT'){
      const candidates=queue.filter(row=>row?.status==='candidate').length;
      const confirmed=queue.filter(row=>['queued','aligning','centered','completed'].includes(row?.status)).length;
      parts.push('待稳定 '+candidates+' · 本批已确认 '+confirmed+' · 本帧合格编号 '+rows.length);
    }else if(batchActive&&gimbalOnly){
      const confirmed=queue.filter(row=>finite(row?.confirmed_at)).length;
      parts.push('本批喷射及关水已确认 '+queue.filter(regionSprayConfirmed).length+'/'+confirmed);
    }else if(batchActive&&Number.isInteger(c.completed_count)&&c.completed_count>=0&&Number.isInteger(c.total_count)&&c.total_count>=c.completed_count)
      parts.push('本批'+(r.execute_fire===false?'居中观察完成 ':'短喷流程完成 ')+c.completed_count+'/'+c.total_count);
    if(batchActive&&Number.isInteger(c.current_target_id)&&!currentRow&&phase!=='CENTER_COLLECT')
      parts.push('当前目标未在本帧出现，等待重新确认');
    if(batchActive&&phase==='CENTER_COLLECT'&&(r.reason||c.collect_reason))
      parts.push(regionReason(r.reason||c.collect_reason));
    if(batchActive&&phase==='CENTER_SELECT'&&r.reason==='waiting_new_batch_candidate_confirmation')
      parts.push(reason(r.reason));
    const skipped=queue.filter(row=>row?.status==='skipped'&&typeof row.reason==='string').sort((a,b)=>(b.finished_at||0)-(a.finished_at||0))[0];
    if(batchActive&&skipped&&['CENTER_SELECT','CENTER_RETURN','CENTER_REACQUIRE'].includes(phase))
      parts.push((gimbalOnly&&!finite(skipped.confirmed_at)?'候选未确认 #':'本批未处置 #')+skipped.target_id+'：'+reason(skipped.reason)+
        (gimbalOnly?'；本批全部确认目标处理后才续行':'；继续其他目标/恢复路线'));
    const cooldown=r.fire_retrigger?.cooldown_remaining_s;
    if(r.active===true&&Number.isFinite(cooldown)&&cooldown>0&&cooldown<=30)
      parts.push('新目标冷却 '+Math.max(0,cooldown-elapsed).toFixed(1)+' 秒');
    if(r.active===true&&registry&&sessionMatches){
      const seen=new Set();
      for(const row of rows){
        const id=row?.target_id,b=row?.bbox_xyxy;
        if(seen.has(id))continue;
        seen.add(id);
        const matching=queue.filter(item=>(item?.target_id===id&&item.registry_session===registry.registry_session)||
          (Array.isArray(item?.observed_id_aliases)&&item.observed_id_aliases.some(alias=>Array.isArray(alias)&&alias.length===2&&alias[0]===registry.registry_session&&alias[1]===id)));
        const queued=matching.length===1?matching[0]:null;
        const status=queued?.status||'confirmed';
        const current=c.active===true&&id===c.current_target_id&&registry.registry_session===session;
        const batch=typeof queued?.batch_id==='string'&&/^batch-\d+$/.test(queued.batch_id)?'本批目标 '+queued.batch_id.slice(6)+' · ':'';
        const completed=status==='completed'&&(!gimbalOnly||regionSprayConfirmed(queued));
        boxes.push({targetId:id,bbox:b.slice(),current,status:status==='completed'&&!completed?'unconfirmed':status,
          label:batch+'#'+id+(current?' 当前':completed?(gimbalOnly?' 已喷且关水确认':' 已处理'):status==='completed'?' 喷射待确认':status==='skipped'?' 已跳过':status==='queued'?(gimbalOnly?' 待瞄准':' 待对齐'):' 待任务确认')});
      }
    }
    if(r.active===true&&(!registry||!sessionMatches))parts.push('等待同帧消防员编号');
    return {visible:true,tolerance,showCenterGuide:!gimbalOnly,boxes,status:parts.join(' · ')};
  }
  // CENTERING_OVERLAY_MODEL_END
  function currentCenteringOverlay(frame) {
    const now=performance.now(),elapsed=state.frameAt?(now-state.frameAt)/1000:Infinity;
    const age=finite(state.frameAge)?state.frameAge+elapsed:Infinity;
    const update=finite(state.frameUpdateAge)?state.frameUpdateAge+elapsed:Infinity;
    const preview=state.data?.preview;
    return centeringOverlayModel(frame,{extension:data('extension'),extensionFresh:fresh('extension'),
      frameFresh:connected()&&state.stream&&state.frameStatus==='live'&&state.frameToken===frame?.token&&
        preview?.upstream_available===true&&preview?.fresh===true&&age>=0&&age<=Math.min(.8,state.frameStale)&&update>=0&&update<=Math.min(.8,state.frameStale),
      elapsed:Math.max(0,(now-state.received)/1000)+(topic('extension')?.age_s||0)});
  }
  function refreshCenteringOverlay() {
    if(!state.bitmap||!state.frame){$('centeringOverlayStatus')?.classList.add('hidden');return;}
    const view=currentCenteringOverlay(state.frame),probe=stereoProbeOverlay(state.frame);
    const signature=state.frame.token+'|'+JSON.stringify(view)+'|'+JSON.stringify(probe);
    if(state.centeringOverlaySignature===signature)return;
    state.centeringOverlaySignature=signature;drawFrame(view,probe);
  }
  function stereoProbeOverlay(frame) {
    const sample=stereoDisplaySample();
    if(state.view!=='fire'||sample.age===null||!state.stream||state.frameStatus!=='live'||
        state.frameToken!==frame?.token)return null;
    const o=sample.o,p=o.scene_probe,b=p?.bbox_xyxy;
    const elapsed=state.frameAt?(performance.now()-state.frameAt)/1000:Infinity;
    const age=finite(state.frameAge)?state.frameAge+elapsed:Infinity;
    const update=finite(state.frameUpdateAge)?state.frameUpdateAge+elapsed:Infinity;
    if(state.data?.preview?.upstream_available!==true||state.data?.preview?.fresh!==true||
        age<0||age>Math.min(.8,state.frameStale)||update<0||update>Math.min(.8,state.frameStale)||
        o.observation_only!==true||o.control_eligible!==false||p?.kind!=='calibration_probe'||
        !Array.isArray(o.image_size_wh)||o.image_size_wh.length!==2||
        o.image_size_wh[0]!==frame.width||o.image_size_wh[1]!==frame.height||
        typeof o.coordinate_frame!=='string'||!o.coordinate_frame||o.coordinate_frame!==frame.coordinate_frame||
        !Array.isArray(b)||b.length!==4||!b.every(finite)||
        b[0]<0||b[1]<0||b[2]>frame.width||b[3]>frame.height||b[2]<=b[0]||b[3]<=b[1]||
        Math.abs((b[0]+b[2])/2-frame.width/2)>1||Math.abs((b[1]+b[3])/2-frame.height/2)>1)return null;
    // This is a fixed sampling-area guide, not a depth result or target box
    // attached to the newer preview image. Never put a distance on this overlay.
    return {bbox:b.slice(),label:'固定中心测距区域（非火源）'};
  }
  function drawStereoProbeOverlay(frame,probe) {
    if(!probe)return;
    const [x1,y1,x2,y2]=probe.bbox,scale=Math.max(1,frame.width/1280),font=Math.max(14,frame.width/70);
    context.save();context.strokeStyle='#d7e2e8';context.lineWidth=1.3*scale;
    context.setLineDash([7*scale,5*scale]);context.strokeRect(x1,y1,x2-x1,y2-y1);
    context.setLineDash([]);context.font=`500 ${font}px "Microsoft YaHei",sans-serif`;
    const width=Math.min(frame.width,context.measureText(probe.label).width+12),height=font+8;
    const x=Math.max(0,Math.min(frame.width-width,(x1+x2-width)/2));
    const y=Math.max(0,Math.min(frame.height-height,y2+5));
    context.fillStyle='rgba(11,13,16,.8)';context.fillRect(x,y,width,height);
    context.fillStyle='#d7e2e8';context.fillText(probe.label,x+6,y+font+1);context.restore();
  }
  function drawCenteringOverlay(frame,view=currentCenteringOverlay(frame)) {
    const node=$('centeringOverlayStatus');
    if(node){node.classList.toggle('hidden',!view.visible);text('centeringOverlayStatus',view.status);}
    if(!view.visible)return;
    const w=frame.width,h=frame.height,left=w*(.5-view.tolerance),right=w*(.5+view.tolerance);
    context.save();
    if(view.showCenterGuide!==false){
    context.fillStyle='rgba(38,198,218,.08)';context.fillRect(left,0,right-left,h);
    context.strokeStyle='#26c6da';context.lineWidth=Math.max(2,w/500);context.setLineDash([w/75,w/100]);
    context.beginPath();for(const x of [left,right]){context.moveTo(x,0);context.lineTo(x,h);}context.stroke();
    context.setLineDash([]);context.beginPath();context.moveTo(w*.5,0);context.lineTo(w*.5,h);context.stroke();
    }
    const font=Math.max(18,w/50);context.font=`600 ${font}px "Microsoft YaHei",sans-serif`;
    for(const item of view.boxes){
      const[x1,y1,x2,y2]=item.bbox,color=item.current?'#ffe26b':item.status==='completed'?'#8ba7a2':item.status==='skipped'?'#9aa4ad':'#ff8b4c';
      context.lineWidth=Math.max(2,w/400)*(item.current?2.1:1.1);context.strokeStyle=color;
      context.strokeRect(x1,y1,x2-x1,y2-y1);
      const tw=Math.min(w,context.measureText(item.label).width+16),th=font+10;
      const x=Math.min(x1,w-tw),y=Math.max(0,Math.min(h-th,y2+3));
      context.fillStyle=color;context.fillRect(x,y,tw,th);context.fillStyle='#091013';context.fillText(item.label,x+8,y+font+1);
    }
    context.restore();
  }
  function showFrameState(){
    const elapsed=state.frameAt?(performance.now()-state.frameAt)/1000:Infinity;
    const age=finite(state.frameAge)?state.frameAge+elapsed:null,update=finite(state.frameUpdateAge)?state.frameUpdateAge+elapsed:null;
    const previewReady=connected()&&state.data?.preview?.upstream_available===true&&state.data?.preview?.fresh===true;
    let current=state.frameStatus;const limit=state.frameStale;
    if(!connected()||!previewReady)current='waiting';
    else if(!state.stream)current='paused';
    else if(current==='live'&&(!finite(age)||age>limit||!finite(update)||update>limit))current='stale';
    const visible=current==='live'&&state.bitmap;
    $('cameraEmpty').classList.toggle('hidden',Boolean(state.bitmap));$('cameraStale').classList.toggle('hidden',Boolean(visible)||!state.bitmap);
    text('cameraStale',current==='paused'?'画面传输已关闭 · 推理状态以板端诊断为准':current==='waiting'?'相机服务未就绪 · 上次画面不代表当前结果':'画面已过期 · 不代表当前结果');
    text('frameBadge',visible?'LIVE  同帧识别':current==='paused'?'传图已关闭':!previewReady?'相机服务未就绪':state.bitmap?'画面已过期':'等待画面');
    $('frameBadge').dataset.state=visible?'ok':'idle';$('frameBadge').className='frame-badge'+(visible?' live':'');text('frameSource','RDK X5 / BPU');
    text('frameGeometry',state.frame?state.frame.width+' × '+state.frame.height+'  帧 '+state.frame.sequence:'—');
    text('frameTiming',visible?'图像年龄 '+Math.round(age*1000)+' ms'+(state.frame.detector_available===false?'  YOLO 按任务关闭':'  检测框与图像来自同一帧'):!previewReady?'相机服务未就绪；未确认当前推理状态':current==='paused'?'传图已关闭；识别是否运行以推理诊断为准':'等待新鲜车载画面，过期画面不会作为实时结果展示');
    const now=performance.now();while(state.frameTimes.length&&now-state.frameTimes[0]>2000)state.frameTimes.shift();
    const fps=state.frameTimes.length>1?1000*(state.frameTimes.length-1)/(state.frameTimes.at(-1)-state.frameTimes[0]):0;text('displayFps','展示 '+(visible?num(fps,1):'—')+' FPS');
    refreshCenteringOverlay();
  }
  function stopFrame(clear){clearTimeout(state.frameTimer);state.frameController?.abort();if(clear){state.frameToken='';state.frameTimes=[];state.frameStatus='waiting';}showFrameState();}
  function scheduleFrame(ms){clearTimeout(state.frameTimer);if(connected()&&wantsYoloFrames()&&!document.hidden&&state.stream)state.frameTimer=setTimeout(pollFrame,ms);}
  async function pollFrame(){if(!connected()||state.frameBusy||!state.stream||!wantsYoloFrames()||document.hidden)return;state.frameBusy=true;const epoch=state.frameEpoch,start=performance.now(),controller=new AbortController();state.frameController=controller;const timeout=setTimeout(()=>controller.abort(),2500);try{const response=await fetch('/api/frame?after='+encodeURIComponent(state.frameToken),{cache:'no-store',signal:controller.signal});if(!response.ok)throw new Error('HTTP '+response.status);const {metadata,jpeg}=unpack(await response.arrayBuffer());if(epoch!==state.frameEpoch)return;if(metadata.stream?.enabled===false){state.stream=false;$('streamToggle').checked=false;state.frameStatus='paused';return;}const generation=metadata.stream?.generation;if(state.frameGeneration!==generation){state.frameGeneration=generation;state.frameToken='';state.frameTimes=[];}if(metadata.frame&&metadata.frame.token!==state.frameToken){const f=metadata.frame;if(!Number.isInteger(f.width)||!Number.isInteger(f.height)||f.width<=0||f.height<=0||f.width>8192||f.height>8192||!Array.isArray(f.detections)||!jpeg.size)throw new Error('无效预览帧');const bitmap=await createImageBitmap(jpeg);if(epoch!==state.frameEpoch){bitmap.close();return;}if(bitmap.width!==f.width||bitmap.height!==f.height){bitmap.close();throw new Error('图像尺寸与标注不一致');}state.bitmap?.close();state.bitmap=bitmap;state.frame={...f,detector_available:metadata.detector_available??f.detector_available};state.frameToken=f.token;state.frameTimes.push(performance.now());}if(metadata.fire_registry_update&&state.frameToken===state.frame?.token)state.frame=applyFrameRegistryUpdate(state.frame,metadata.fire_registry_update);if(metadata.firefighter_registry_update&&state.frameToken===state.frame?.token)state.frame=applyFrameRegistryUpdate(state.frame,metadata.firefighter_registry_update,'firefighter_registry');const requestAge=(performance.now()-start)/1000;state.frameAge=finite(metadata.frame_age_s)?metadata.frame_age_s+requestAge:null;state.frameUpdateAge=finite(metadata.update_age_s)?metadata.update_age_s+requestAge:null;state.frameAt=performance.now();state.frameStale=finite(metadata.stale_after_s)?metadata.stale_after_s:3;state.frameStatus=metadata.stream?.active===false?'waiting':metadata.state||'waiting';state.previewPerformance=metadata.performance||{};}catch(error){if(epoch===state.frameEpoch)state.frameStatus='stale';}finally{clearTimeout(timeout);state.frameBusy=false;state.frameController=null;showFrameState();scheduleFrame(Math.max(0,1000/30-(performance.now()-start)));}}
  async function changeImageTransfer(source){
    const desired=source.checked;
    $('streamToggle').disabled=true;$('roadImageToggle').disabled=true;
    state.frameEpoch++;stopFrame(true);stopRoadFrames();
    try{await request('/api/stream',{enabled:desired});state.stream=desired;toast(desired?'已请求开启画面传输。':'已请求关闭画面传输；识别任务未因该请求改变。');}
    catch(error){toast('传图开关未确认：'+error.message,true);}
    finally{$('streamToggle').checked=state.stream;$('streamToggle').disabled=!connected();showFrameState();showRoadFrameState();scheduleFrame(0);scheduleRoadFrame(0);}
  }
  $('streamToggle').addEventListener('change',event=>changeImageTransfer(event.target));
  $('roadImageToggle').addEventListener('change',event=>changeImageTransfer(event.target));
  $('roadShowDetection').addEventListener('click',()=>selectRoadView('detection'));
  $('roadShowYolo').addEventListener('click',()=>selectRoadView('yolo'));
  document.addEventListener('visibilitychange',()=>{if(document.hidden){endMotorRuns('控制台已隐藏');state.frameEpoch++;stopFrame(false);stopRoadFrames();}else{showFrameState();showRoadFrameState();scheduleFrame(0);scheduleRoadFrame(0);}});
  window.addEventListener('pagehide',()=>{endMotorRuns('控制台已关闭');state.frameEpoch++;state.frameController?.abort();state.bitmap?.close();stopRoadFrames();});

  selectView(location.hash.slice(1)||'overview');renderAll();pollState();pollFpgaMap();scheduleFrame(0);
  setInterval(()=>{showFrameState();showRoadFrameState();if(!connected()){state.connected=false;renderAll();}else{renderStereoObservation();if(state.view==='road')renderRoad();}},700);
})();
