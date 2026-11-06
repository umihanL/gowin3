`timescale 1ns/1ps
module nav_astar_tb;
    localparam W=9,H=8,N=W*H;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,start=0,cancel=0;
    reg [7:0] sx=0,sy=0,gx=8,gy=7;
    wire busy,done,pv;wire [2:0] error;
    wire [14:0] ma,pa;wire [7:0] px,py;wire [13:0] cost;wire [15:0] expanded;
    reg map[0:N-1];reg blocked;
    reg central_map[0:N-1];reg [7:0] directions_map[0:N-1];
    reg central_q;reg [7:0] directions_q;
    astar_grid #(.WIDTH(W),.HEIGHT(H)) dut(.clk(clk),.rst_n(rst_n),.start(start),.cancel(cancel),
        .start_x(sx),.start_y(sy),.goal_x(gx),.goal_y(gy),.busy(busy),.done(done),.error(error),
        .map_addr(ma),.map_blocked(blocked),.map_directions(directions_q),.map_central(central_q),.path_valid(pv),.path_addr(pa),.path_x(px),.path_y(py),
        .path_cost(cost),.expanded(expanded));
    always @(posedge clk)begin
        blocked<=ma<N?map[ma]:1'b1;
        central_q<=central_map[ma];directions_q<=directions_map[ma];
    end
    function integer edge_cost;
        input integer cell_id,dir;
        begin edge_cost=dir<4?5:7;end
    endfunction
    integer queue[0:N-1],distance[0:N-1],seen[0:N-1];
    integer head,tail,node,candidate,j,i,xx,yy,oracle,steps,last_x,last_y,cases=0;
    integer seed=5640,random_value,trace_cost,ddx,ddy;
    task bfs;
        begin
            for(i=0;i<N;i=i+1)begin distance[i]=-1;seen[i]=0;end
            oracle=-1;
            if(sx<W && gx<W && sy<H && gy<H && !map[sy*W+sx] && !map[gy*W+gx])begin
                distance[sy*W+sx]=0;
                for(tail=0;tail<N;tail=tail+1)begin
                    node=-1;
                    for(i=0;i<N;i=i+1)if(!seen[i] && distance[i]>=0 &&
                        (node<0 || distance[i]<distance[node]))node=i;
                    if(node>=0)begin
                        seen[node]=1;
                        for(j=0;j<8;j=j+1)begin
                            xx=node%W;yy=node/W;
                            case(j)
                                0:xx=xx+1;1:xx=xx-1;2:yy=yy+1;3:yy=yy-1;
                                4:begin xx=xx+1;yy=yy+1;end
                                5:begin xx=xx-1;yy=yy-1;end
                                6:begin xx=xx+1;yy=yy-1;end
                                7:begin xx=xx-1;yy=yy+1;end
                            endcase
                            candidate=yy*W+xx;
                            if(xx>=0 && xx<W && yy>=0 && yy<H && !map[candidate] &&
                                (j<4 || (!map[(node/W)*W+xx] && !map[yy*W+node%W])) &&
                                (distance[candidate]<0 || distance[node]+edge_cost(candidate,j)<distance[candidate]))
                                distance[candidate]=distance[node]+edge_cost(candidate,j);
                        end
                    end
                end
                oracle=distance[gy*W+gx];
            end
            for(i=0;i<N;i=i+1)seen[i]=0;
        end
    endtask
    task run_case;
        input integer expected_error;
        begin
            bfs;steps=0;trace_cost=0;last_x=-1;last_y=-1;cases=cases+1;
            @(negedge clk);start=1;@(negedge clk);start=0;
            wait(done);@(posedge clk);@(negedge clk);
            if(error!==expected_error)$fatal(1,"case %0d error got %0d expected %0d",cases,error,expected_error);
            if(error==0)begin
                if(cost!==oracle || trace_cost!=oracle)$fatal(1,"shortest path case %0d cost=%0d oracle=%0d vertices=%0d",cases,cost,oracle,steps);
                if(last_x!=sx || last_y!=sy)$fatal(1,"trace did not reach start");
            end
        end
    endtask
    integer trace_direction;
    always @(posedge clk)if(pv)begin
        if(px>=W || py>=H || pa!=py*W+px || map[pa])$fatal(1,"invalid/blocked trace vertex");
        if(steps==0 && (px!=gx || py!=gy))$fatal(1,"trace must begin at goal");
        if(steps!=0)begin
            ddx=px>last_x?px-last_x:last_x-px;ddy=py>last_y?py-last_y:last_y-py;
            if(ddx>1 || ddy>1 || ddx+ddy==0)$fatal(1,"nonadjacent trace");
            if(ddx && ddy && (map[last_y*W+px] || map[py*W+last_x]))$fatal(1,"diagonal corner cutting");
            if(ddx && ddy)trace_direction=last_x>px?(last_y>py?4:6):(last_y<py?5:7);
            else trace_direction=last_x>px?0:last_x<px?1:last_y>py?2:3;
            trace_cost=trace_cost+edge_cost(last_y*W+last_x,trace_direction);
        end
        if(seen[pa])$fatal(1,"trace loop");
        seen[pa]=1;last_x=px;last_y=py;steps=steps+1;
    end
    integer trial;
    initial begin
        for(i=0;i<N;i=i+1)begin map[i]=0;central_map[i]=1;directions_map[i]=8'hff;end
        repeat(4)@(negedge clk);rst_n=1;
        run_case(0);
        for(i=0;i<H-1;i=i+1)map[i*W+4]=1;
        sx=1;sy=0;gx=7;gy=0;run_case(0);
        map[(H-1)*W+4]=1;run_case(3);
        map[sy*W+sx]=1;run_case(2);
        sx=W;run_case(1);
        for(trial=0;trial<40;trial=trial+1)begin
            for(i=0;i<N;i=i+1)begin
                random_value=$random(seed);map[i]=(random_value&255)<60;
                central_map[i]=random_value[8];directions_map[i]=random_value[16:9];
            end
            sx=0;sy=0;gx=8;gy=7;map[0]=0;map[N-1]=0;
            bfs;run_case(oracle<0?3:0);
        end
        for(i=0;i<N;i=i+1)begin map[i]=0;central_map[i]=1;directions_map[i]=8'hff;end
        sx=4;sy=3;gx=4;gy=3;run_case(0);
        sx=0;sy=0;gx=8;gy=7;
        @(negedge clk);start=1;@(negedge clk);start=0;
        repeat(80)@(negedge clk);cancel=1;@(negedge clk);cancel=0;
        if(busy)$fatal(1,"cancel did not stop planner");
        run_case(0);
        $display("PASS A*: %0d Dijkstra comparisons, blocked/outside endpoints, no path, same cell, cancel/restart, every trace edge",cases);$finish;
    end
    initial begin #100000000;$fatal(1,"A* timeout");end
endmodule
