// Extensible fixed-size host protocol. See docs/host_protocol_20261027.md.
// Frame: A5 5A 01 sequence opcode arg0 arg1 arg2 arg3 XOR(bytes 2..8).
module vehicle_host_link #(parameter CLK_HZ=33000000,BAUD=115200,LINK_TIMEOUT_MS=2000)(
 input clk,rst_n,rx,selected,drive_page,navigation_page,navigation_ready,
 input planning_page,point_select_ready,point_route_ready,point_planning,start_heading_valid,
 input [3:0] selected_point,input [2:0] point_error,
 input map_start_valid,input [7:0] map_start_x,map_start_y,input signed [15:0] map_start_heading,
 input [31:0] map_start_edit_generation,
 output reg point_select,point_start,point_cancel,output reg [3:0] point_id,
 input [3:0] motion_status,motion_phase,
 input event_valid,input [7:0] event_code,input [15:0] event_value,
 input route_page,input [7:0] extension_result,
 input traffic_start_blocked,
 input [31:0] extension_status,extension_capabilities,
 input pump_supported,input [7:0] pump_result,input [31:0] pump_status,
 output reg pump_command_valid,
 input extension_event_valid,input [7:0] extension_event_code,extension_event_task,extension_event_status,
 output reg extension_command_valid,output [7:0] extension_opcode,extension_sequence,
 output [31:0] extension_arguments,
 // Read-only map telemetry. Optional ports remain disabled in old embeddings.
 input map_telemetry_enabled,input [3:0] map_mode,input map_pose_valid,map_imu_available,
 input signed [23:0] map_pose_x_q8,map_pose_y_q8,input signed [15:0] map_heading,
 input [31:0] map_signature,input [15:0] map_generation,input map_route_valid,map_planning,
 input [11:0] map_route_count,output [10:0] map_route_index,
 input [16:0] map_route_x_q8,map_route_y_q8,
 input [15:0] map_barrier_generation,input [3:0] map_barrier_valid,
 input [35:0] map_barrier_x0,map_barrier_y0,map_barrier_x1,map_barrier_y1,
 output tx,output reg run,nav_session,output reg [15:0] speed_limit,
 output link_ok,output reg [7:0] task_id
);
 wire byte_valid,frame_error;wire [7:0] byte_data;
 host_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) receiver(.clk(clk),.rst_n(rst_n),.rx(rx),
  .valid(byte_valid),.data(byte_data),.framing_error(frame_error));
 localparam MS_CYCLES=CLK_HZ/1000;localparam MW=$clog2(MS_CYCLES+1);
 reg [MW-1:0] ms_timer;reg [11:0] link_age;reg [5:0] frame_age;
 wire ms_tick=ms_timer==MS_CYCLES-1;
 assign link_ok=selected && link_age<LINK_TIMEOUT_MS;
 reg [3:0] pos;reg [7:0] seq,opcode,a0,a1,a2,a3,checksum;reg version_ok;
 wire frame_ready=byte_valid && pos==9 && byte_data==checksum && version_ok;
 wire map_page=planning_page===1'b1;
 // Extension motion/physical stopping must finish before a new legacy run
 // or numbered route is accepted. Unconnected optional status stays false.
 wire extension_motion_busy=(extension_status[7:0]===8'd1) ||
     (extension_status[7:0]===8'd2) || (extension_status[7:0]===8'd3) ||
     (extension_status[7:0]===8'd6);
 wire scope=selected && (drive_page || navigation_page || map_page || route_page===1'b1);
 wire automatic_handoff=previous_map && navigation_page && handoff_armed && run && nav_session;
 wire page_change=drive_page!=previous_drive || navigation_page!=previous_nav || map_page!=previous_map;
 wire stable_scope=selected==previous_selected && (!page_change || automatic_handoff);
 assign extension_opcode=opcode;assign extension_sequence=seq;assign extension_arguments={a3,a2,a1,a0};
 wire [15:0] argument={a1,a0};
 wire telemetry_opcode=opcode>=8'h30 && opcode<=8'h3b;
 wire readonly_opcode=telemetry_opcode || opcode==8'h1a;
 assign map_route_index={a3[2:0],a2};
 reg [15:0] pose_token;reg pose_snapshot;
 wire [15:0] next_pose_token=pose_token>=16'hfffe ? 16'd1:pose_token+16'd1;
 reg signed [15:0] snapshot_x,snapshot_y,snapshot_heading;
 reg [7:0] snapshot_flags,snapshot_task,snapshot_mode,snapshot_status;
 wire [7:0] pose_flags={4'd0,nav_session,map_route_valid,map_imu_available,map_pose_valid};
 wire [20:0] route_x_mm_wide=map_route_x_q8*4'd10;
 wire [20:0] route_y_mm_wide=map_route_y_q8*4'd10;
 wire [15:0] route_x_mm=route_x_mm_wide>>8,route_y_mm=route_y_mm_wide>>8;
 wire [8:0] barrier_x=a3==0?map_barrier_x0[a2[1:0]*9+:9]:map_barrier_x1[a2[1:0]*9+:9];
 wire [8:0] barrier_y=a3==0?map_barrier_y0[a2[1:0]*9+:9]:map_barrier_y1[a2[1:0]*9+:9];
 wire [15:0] barrier_x_mm=barrier_x*16'd10,barrier_y_mm=barrier_y*16'd10;
 reg previous_selected,previous_drive,previous_nav,previous_map,handoff_armed;
 reg [3:0] previous_status;reg was_running;
 reg event_pending;reg [7:0] pending_event,pending_task;reg [15:0] pending_value;
 reg [7:0] reply_seq,reply_op,r0,r1,r2,r3,reply_xor;
 reg tx_active,send_byte;reg [3:0] tx_index;reg [7:0] tx_data;wire tx_busy,tx_done;
 reg busy_pending;reg [7:0] busy_seq,busy_op,busy_result;
 wire reply_idle=!tx_active && !tx_busy && !send_byte;
 uart_byte_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) transmitter(.clk(clk),.rst_n(rst_n),
  .data_byte(tx_data),.send_en(send_byte),.baud_set(3'd4),.uart_tx(tx),.tx_done(tx_done),.uart_state(tx_busy));
 reg [7:0] result;
 wire [7:0] flags={3'd0,link_ok,nav_session,run,scope,selected};
 always @*begin
  result=0;
  case(opcode)
   8'h01,8'h06:result=0;
   8'h02,8'h04:if(!scope)result=1;else if(argument>1000 || a2!=0 || a3!=0)result=2;
   8'h03:if(!scope)result=1;else if(a0>1 || a2!=0 || a3!=0)result=2;
      else if(a0==1 && start_heading_valid===1'b0)result=7;
      else if((a0==0 && !drive_page)||(a0==1 && (!(navigation_page || (map_page && point_route_ready)) || !navigation_ready)))result=3;
      else if(a0==0 && traffic_start_blocked===1'b1)result=3;
      else if(nav_session || run || extension_motion_busy)result=4;
   8'h05:if(!scope)result=1;
   8'h08:if(!selected || !map_page)result=1;
      else if(a0<1 || a0>9 || a2!=0 || a3!=0)result=2;
      else if(run || nav_session || extension_motion_busy || point_planning===1'b1)result=4;
      else if(start_heading_valid===1'b0)result=7;
      else if(point_select_ready!==1'b1)result=3;
   8'h09:result=0;
   8'h30,8'h31,8'h32,8'h33,8'h34,8'h35,8'h36,8'h38,8'h39,8'h3a,8'h3b:
      if(map_telemetry_enabled!==1'b1)result=5;
   8'h07:if(!scope)result=1;else if(!nav_session || !navigation_page || motion_status>=3)result=3;
   8'h25:if(!scope)result=1;
      else if({a3,a2,a1,a0}!=0)result=2;
      else if(run || extension_motion_busy || pump_supported!==1'b1 ||
              pump_status[1]!==1'b1 || pump_status[0]!==1'b0)result=4;
      else result=extension_result;
   8'h14,8'h15:result=extension_result;
   8'h18:result=pump_supported!==1'b1 ? 8'd5:pump_result;
   8'h19:result=pump_supported!==1'b1 ? 8'd5:8'd0;
   8'h1a:result=pump_supported!==1'b1 ? 8'd5:pump_result;
   8'h10,8'h11,8'h12,8'h13,8'h16,8'h17,
   8'h20,8'h21,8'h22,8'h23,8'h24:result=!scope?8'd1:extension_result;
   default:result=5;
  endcase
  // ON and vehicle motion are mutually exclusive, including overlapping UART
  // requests before measured wheel speed has had time to change.
  if(pump_status[0]===1'b1 && (opcode==8'h03 || opcode==8'h07 || opcode==8'h10 || opcode==8'h11))result=4;
 end
 task reply(input [7:0] s,op,b0,b1,b2,b3);begin
  reply_seq<=s;reply_op<=op;r0<=b0;r1<=b1;r2<=b2;r3<=b3;
  reply_xor<=8'h01^s^op^b0^b1^b2^b3;tx_active<=1;tx_index<=0;
 end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   ms_timer<=0;link_age<=LINK_TIMEOUT_MS;frame_age<=0;pos<=0;seq<=0;opcode<=0;
   a0<=0;a1<=0;a2<=0;a3<=0;checksum<=0;version_ok<=0;
   run<=0;nav_session<=0;speed_limit<=200;task_id<=0;
   previous_selected<=0;previous_drive<=0;previous_nav<=0;previous_map<=0;handoff_armed<=0;
   point_select<=0;point_start<=0;point_cancel<=0;point_id<=0;previous_status<=0;was_running<=0;
   event_pending<=0;pending_event<=0;pending_task<=0;pending_value<=0;
   reply_seq<=0;reply_op<=0;r0<=0;r1<=0;r2<=0;r3<=0;reply_xor<=0;
   tx_active<=0;send_byte<=0;tx_index<=0;tx_data<=0;
   busy_pending<=0;busy_seq<=0;busy_op<=0;busy_result<=0;
   extension_command_valid<=0;pump_command_valid<=0;
   pose_token<=0;pose_snapshot<=0;snapshot_x<=0;snapshot_y<=0;snapshot_heading<=0;
   snapshot_flags<=0;snapshot_task<=0;snapshot_mode<=0;snapshot_status<=0;
  end else begin
   previous_selected<=selected;previous_drive<=drive_page;previous_nav<=navigation_page;previous_map<=map_page;
   point_select<=0;point_start<=0;point_cancel<=0;
   if(automatic_handoff || !scope || selected!=previous_selected || (page_change && !automatic_handoff))handoff_armed<=0;previous_status<=motion_status;
   send_byte<=0;
   extension_command_valid<=0;pump_command_valid<=0;
   // Stop pumping does not depend on control page, link lease or reply space.
   if(frame_ready && (opcode==8'h19 || opcode==8'h05) && pump_supported===1'b1)
     pump_command_valid<=1;
   if(ms_tick)begin
    ms_timer<=0;
    if(link_age<LINK_TIMEOUT_MS)link_age<=link_age+1'b1;
    if(pos!=0 && frame_age<25)frame_age<=frame_age+1'b1;
   end else ms_timer<=ms_timer+1'b1;
   if(frame_error || frame_age>=20)begin pos<=0;frame_age<=0;end
   else if(byte_valid)begin
    frame_age<=0;
    case(pos)
     0:if(byte_data==8'ha5)pos<=1;
     1:if(byte_data==8'h5a)pos<=2;else pos<=(byte_data==8'ha5)?1:0;
     2:begin version_ok<=byte_data==1;checksum<=byte_data;pos<=3;end
     3:begin seq<=byte_data;checksum<=checksum^byte_data;pos<=4;end
     4:begin opcode<=byte_data;checksum<=checksum^byte_data;pos<=5;end
     5:begin a0<=byte_data;checksum<=checksum^byte_data;pos<=6;end
     6:begin a1<=byte_data;checksum<=checksum^byte_data;pos<=7;end
     7:begin a2<=byte_data;checksum<=checksum^byte_data;pos<=8;end
     8:begin a3<=byte_data;checksum<=checksum^byte_data;pos<=9;end
     9:pos<=byte_data==8'ha5 && !frame_ready?1:0;
     default:pos<=0;
    endcase
   end
   // No stale command can arm the car after a mode or driving-page change.
   if(!scope || selected!=previous_selected || (page_change && !automatic_handoff))begin
    run<=0;nav_session<=0;was_running<=0;link_age<=LINK_TIMEOUT_MS;
   end else if(!link_ok && run)begin
    run<=0;event_pending<=1;pending_event<=8'hf2;pending_task<=task_id;pending_value<=1;
   end
   if(selected!=previous_selected || (page_change && !automatic_handoff))begin
    pos<=0;frame_age<=0;
   end
   if(nav_session && motion_status==2)was_running<=1;
   if(nav_session && (was_running || motion_status>=4) && motion_status>=3 && motion_status!=previous_status)begin
    // Keep the finished session selected so its final status stays visible.
    // A new navigation task requires leaving/re-entering its page.
    run<=0;was_running<=0;event_pending<=1;
    pending_event<=motion_status==3?8'hf0:8'hf1;pending_task<=task_id;pending_value<={12'd0,motion_status};
   end
   if(event_valid && !event_pending)begin
    event_pending<=1;pending_event<=event_code;pending_task<=task_id;pending_value<=event_value;
   end
   if(extension_event_valid===1'b1 && !event_pending)begin
    event_pending<=1;pending_event<=extension_event_code;pending_task<=extension_event_task;
    pending_value<={8'd0,extension_event_status};
   end
   // Stop-and-wait requests: a busy transmitter rejects mutations; STOP
   // still takes effect immediately. Host waits for reply before next command.
   if(frame_ready && scope && stable_scope && !readonly_opcode &&
      (reply_idle || opcode==8'h05))begin
    link_age<=0;
    if(result==0)case(opcode)
     8'h02,8'h04:speed_limit<=argument;
     8'h03:begin run<=1;nav_session<=a0==1;task_id<=a1;was_running<=0;
       if(a0==1 && map_page)begin point_start<=1;handoff_armed<=1;end end
     8'h05:begin run<=0;handoff_armed<=0;if(map_page)point_cancel<=1;end
     8'h08:begin point_select<=1;point_id<=a0[3:0];task_id<=a1;end
     8'h07:run<=1;
     8'h12,8'h20,8'h25:begin run<=0;nav_session<=0;was_running<=0;handoff_armed<=0;end
     default:;
    endcase
    if(result==0 && (opcode==8'h05 || (opcode>=8'h10 && opcode<=8'h17) || (opcode>=8'h20 && opcode<=8'h25)))
      extension_command_valid<=1;
    if(result==0 && pump_supported===1'b1 && (opcode==8'h18 || opcode==8'h12 ||
       opcode==8'h20 || opcode==8'h03 || opcode==8'h07 || opcode==8'h10 || opcode==8'h11))
      pump_command_valid<=1;
   end
   if(frame_ready && !reply_idle && (!busy_pending || opcode==8'h05 || opcode==8'h19))begin
    busy_pending<=1;busy_seq<=seq;busy_op<=opcode|8'h80;
    busy_result<=(opcode==8'h05 || opcode==8'h19)?result:8'd4;
   end
   if(reply_idle)begin
    if(frame_ready)begin
     if(telemetry_opcode && result==0)begin
      case(opcode)
       8'h30:reply(seq,8'hb0,0,1,8'h3f,1);
       8'h31:begin
        if({a3,a2,a1,a0}!=0)reply(seq,8'hb1,8'hff,8'hff,8'hff,8'hff);
        else begin
         pose_token<=next_pose_token;pose_snapshot<=1;
         snapshot_x<=map_pose_x_q8>>>8;snapshot_y<=map_pose_y_q8>>>8;snapshot_heading<=map_heading;
         snapshot_flags<=pose_flags;snapshot_task<=task_id;snapshot_mode<={4'd0,map_mode};snapshot_status<={motion_phase,motion_status};
         reply(seq,8'hb1,next_pose_token[7:0],next_pose_token[15:8],pose_flags,task_id);
        end
       end
       8'h32:if(pose_snapshot && argument==pose_token && a2==0 && a3==0 && snapshot_flags[0])
        reply(seq,8'hb2,snapshot_x[7:0],snapshot_x[15:8],snapshot_y[7:0],snapshot_y[15:8]);
        else reply(seq,8'hb2,8'hff,8'hff,8'hff,8'hff);
       8'h33:if(pose_snapshot && argument==pose_token && a2==0 && a3==0 && snapshot_flags[0])
        reply(seq,8'hb3,snapshot_heading[7:0],snapshot_heading[15:8],snapshot_mode,snapshot_status);
        else reply(seq,8'hb3,8'hff,8'hff,8'hff,8'hff);
       8'h34:reply(seq,8'hb4,map_generation[7:0],map_generation[15:8],map_route_count[7:0],
        {map_route_valid,map_planning,2'd0,map_route_count[11:8]});
       8'h35:if(argument==map_generation && map_route_valid && !map_planning && {a3,a2}<map_route_count && a3<8)
        reply(seq,8'hb5,route_x_mm[7:0],route_x_mm[15:8],route_y_mm[7:0],route_y_mm[15:8]);
        else reply(seq,8'hb5,8'hff,8'hff,8'hff,8'hff);
       8'h36:reply(seq,8'hb6,map_signature[7:0],map_signature[15:8],map_signature[23:16],map_signature[31:24]);
       8'h3b:if(map_start_valid===1'b1 && {a3,a2,a1,a0}==0)
        reply(seq,8'hbb,map_start_edit_generation[7:0],map_start_edit_generation[15:8],map_start_edit_generation[23:16],map_start_edit_generation[31:24]);
        else reply(seq,8'hbb,0,0,0,0);
       8'h3a:if(map_start_valid===1'b1 && {a3,a2,a1,a0}==0)
        reply(seq,8'hba,map_start_x,map_start_y,map_start_heading[7:0],map_start_heading[15:8]);
        else reply(seq,8'hba,8'hff,8'hff,0,0);
       8'h38:reply(seq,8'hb8,map_barrier_generation[7:0],map_barrier_generation[15:8],{4'd0,map_barrier_valid},0);
       8'h39:if(argument==map_barrier_generation && a2<4 && a3<2 && map_barrier_valid[a2[1:0]])
        reply(seq,8'hb9,barrier_x_mm[7:0],barrier_x_mm[15:8],barrier_y_mm[7:0],barrier_y_mm[15:8]);
        else reply(seq,8'hb9,8'hff,8'hff,8'hff,8'hff);
       default:reply(seq,opcode|8'h80,5,0,0,0);
      endcase
     end else if(opcode==8'h1a)begin
      if(result==0)reply(seq,8'h9a,pump_status[7:0],pump_status[15:8],pump_status[23:16],pump_status[31:24]);
      else reply(seq,8'h9a,8'hff,8'hff,8'hff,8'hff);
     end else if(opcode==8'h09)reply(seq,8'h89,0,{4'd0,selected_point},
       nav_session?8'd4:point_planning===1'b1 ? 8'd1:point_route_ready===1'b1 ? 8'd2:point_error!=0?8'd3:8'd0,{5'd0,point_error});
     else if(result==0 && opcode==8'h08)reply(seq,8'h88,0,a0,1,a1);
     else if(result==0 && opcode==8'h24)reply(seq,8'ha4,0,a0,a1,0);
     else if(opcode==8'h06 || opcode==8'h01)reply(seq,opcode|8'h80,0,{motion_phase,motion_status},flags,task_id);
     else if(opcode==8'h14)reply(seq,8'h94,extension_status[7:0],extension_status[15:8],extension_status[23:16],extension_status[31:24]);
     else if(opcode==8'h15)reply(seq,8'h95,extension_capabilities[7:0],extension_capabilities[15:8],extension_capabilities[23:16],extension_capabilities[31:24]);
     else if(result==0 && (opcode==8'h02 || opcode==8'h04))
      reply(seq,opcode|8'h80,result,flags,a0,a1);
     else reply(seq,opcode|8'h80,result,flags,speed_limit[7:0],speed_limit[15:8]);
    end else if(busy_pending)begin
     if(busy_op==8'hbb)reply(busy_seq,busy_op,0,0,0,0);
     else if((busy_op>=8'hb1 && busy_op<=8'hba) || busy_op==8'h9a)reply(busy_seq,busy_op,8'hff,8'hff,8'hff,8'hff);
     else reply(busy_seq,busy_op,busy_result,flags,speed_limit[7:0],speed_limit[15:8]);busy_pending<=0;
    end else if(event_pending)begin
     reply(0,pending_event,pending_task,pending_value[7:0],pending_value[15:8],0);event_pending<=0;
    end
   end
   if(tx_active && !tx_busy && !send_byte)begin
    send_byte<=1;
    case(tx_index)
     0:tx_data<=8'ha5;1:tx_data<=8'h5a;2:tx_data<=1;3:tx_data<=reply_seq;4:tx_data<=reply_op;
     5:tx_data<=r0;6:tx_data<=r1;7:tx_data<=r2;8:tx_data<=r3;9:tx_data<=reply_xor;
    endcase
    if(tx_index==9)tx_active<=0;else tx_index<=tx_index+1'b1;
   end
  end
 end
endmodule
