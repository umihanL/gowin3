// Existing 2 cm planner cells; numbering follows the annotated map.
function [15:0] nav_destination;
 input [3:0] id;
 begin
  case(id)
   1:nav_destination={8'd17,8'd17};
   2:nav_destination={8'd80,8'd12};
   3:nav_destination={8'd142,8'd17};
   4:nav_destination={8'd147,8'd88};
   5:nav_destination={8'd142,8'd158};
   6:nav_destination={8'd80,8'd163};
   7:nav_destination={8'd17,8'd158};
   8:nav_destination={8'd13,8'd88};
   9:nav_destination={8'd73,8'd88};
   default:nav_destination=0;
  endcase
 end
endfunction
