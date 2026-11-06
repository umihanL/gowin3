`timescale 1ns/1ps
module font_rows_tb;
 `include "nav_font.vh"
 reg [15:0] rows[0:4095];
 reg [255:0] bitmap;
 integer code,y;
 initial begin
  $readmemh("src/navigation/nav_font_rows.mem",rows);
  for(code=0;code<256;code=code+1)begin
   bitmap=nav_glyph(code);
   for(y=0;y<16;y=y+1)
    if(rows[code*16+y]!==bitmap[255-y*16-:16])
     $fatal(1,"font row mismatch code=%0d y=%0d",code,y);
  end
  $display("PASS font rows: all 65536 font pixels match original bitmaps");$finish;
 end
endmodule
