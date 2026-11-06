`timescale 1ns/1ps
module vision_netlist_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fb=0,req=0,pv=0;reg [11:0] x=0,y=0;reg [15:0] rgb=0;
 wire [2:0] actual_visibility,expected_visibility;
 wire [15:0] actual,expected;
 wire actual_update,expected_update,actual_valid,expected_valid,actual_junction,expected_junction;
 wire signed [8:0] actual_near,expected_near,actual_heading,expected_heading;
 GSR GSR(.GSRI(1'b1));
 vision_video dut(.turn_hint(2'sd0),.clk(clk),.rst_n(rst),.frame_begin(fb),.data_req(req),.pixel_valid(pv),.x(x),.y(y),.rgb(rgb),.threshold(8'd96),.road_visibility(actual_visibility),.preview(actual),.road_update(actual_update),.road_valid(actual_valid),.road_junction(actual_junction),.road_near(actual_near),.road_heading(actual_heading));
 vision_video_reference ref_dut(.turn_hint(2'sd0),.clk(clk),.rst_n(rst),.frame_begin(fb),.data_req(req),.pixel_valid(pv),.x(x),.y(y),.rgb(rgb),.threshold(8'd96),.road_visibility(expected_visibility),.preview(expected),.road_update(expected_update),.road_valid(expected_valid),.road_junction(expected_junction),.road_near(expected_near),.road_heading(expected_heading));
 integer metric_checks=0;
 always @(negedge clk)if(rst && expected_update)begin
  if(actual_visibility!==expected_visibility || actual_update!==expected_update || actual_valid!==expected_valid || actual_junction!==expected_junction || actual_near!==expected_near || actual_heading!==expected_heading)
   $fatal(1,"synthesized normalized lane arithmetic mismatch: %b/%b %d/%d %d/%d",actual_valid,expected_valid,actual_near,expected_near,actual_heading,expected_heading);
  if(metric_checks==5 && (!actual_valid || actual_near>=0))$fatal(1,"clipped-edge arithmetic sign incorrect");
  metric_checks=metric_checks+1;if(actual_valid)valid_checks=valid_checks+1;
 end
 integer xx,yy,c,checks=0,kind,valid_checks=0;
 initial begin
 repeat(10)@(negedge clk);rst=1;
 for(kind=0;kind<6;kind=kind+1)begin
 for(yy=0;yy<480;yy=yy+1)for(xx=0;xx<640;xx=xx+1)begin
  @(negedge clk);req=1;pv=1;x=80+xx;y=yy;
  c=((xx/31)^(yy/29)*13)&63;rgb={c[1:0],c[1:0],c[1],c[3:2],c[3:2],c[3:2],c[5:4],c[5:4],c[5]};
  if(kind==1)rgb=xx>160 && xx<560?16'h0000:16'hffff;
  if(kind==2)rgb=(yy>340?xx>120 && xx<520:xx>60 && xx<600)?16'h0000:16'hffff;
  if(kind==3)begin
    rgb=xx>160 && xx<560?16'h0000:16'hffff;
    if(yy*78/480==58 || yy*78/480==44 || yy*78/480==30)rgb=16'hffff;
  end
  if(kind==4)rgb=xx>40 && xx<600?16'h0000:16'hffff;
  if(kind==5)rgb=xx<230?16'h0000:16'hffff;
 end
 @(negedge clk);pv=0;req=0;fb=1;@(negedge clk);fb=0;wait(expected_update);@(negedge clk);
 end
 for(yy=0;yy<480;yy=yy+1)for(xx=0;xx<800;xx=xx+1)begin
  @(negedge clk);req=1;x=xx;y=yy;
  @(posedge clk);#1;
  if(actual!==expected)begin
   $display("reference address=%d",ref_dut.read_addr);
   $fatal(1,"synthesized preview mismatch x=%d y=%d got=%h expected=%h",xx,yy,actual,expected);
  end
  checks=checks+1;
 end
 if(metric_checks!=6 || valid_checks<5)$fatal(1,"synthesized metric projection was never exercised");
 $display("PASS synthesized vision: %d preview pixels and %d normalized lane arithmetics match RTL using actual Gowin BSRAM",checks,metric_checks);$finish;
 end
 initial begin #70000000;$fatal(1,"netlist preview timeout");end
endmodule
