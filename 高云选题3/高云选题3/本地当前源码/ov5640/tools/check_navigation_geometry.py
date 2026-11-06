"""Independent diagnostic model of raster search, smoothing and body clearance."""
from pathlib import Path
import heapq
import math
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
raw = np.loadtxt(ROOT/'src/navigation/map_raw_1cm.mem', dtype=int).reshape(320,352)
plan = np.loadtxt(ROOT/'src/navigation/map_plan_2cm.mem', dtype=int).reshape(160,176)
heading = np.array([int(v,16) for v in (ROOT/'src/navigation/map_heading_2cm.mem').read_text().split()]).reshape(160,176)
central = np.loadtxt(ROOT/'src/navigation/map_central_2cm.mem', dtype=int).reshape(160,176)
directions = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,-1),(1,-1),(-1,1)]


def solve(start, goal):
    def heuristic(x,y):
        a,b = abs(x-start[0]),abs(y-start[1])
        return 5*max(a,b)+2*min(a,b)
    pending = [(heuristic(*goal),0,goal[1]*176+goal[0],goal)]
    costs, parents = {goal:0}, {}
    while pending:
        _, neg_g, _, current = heapq.heappop(pending)
        if -neg_g != costs[current]:
            continue
        if current == start:
            route = [current]
            while current != goal:
                current = parents[current]
                route.append(current)
            return np.array(route)*512+256
        x,y = current
        for direction,(dx,dy) in enumerate(directions):
            nx,ny = x+dx,y+dy
            neighbour = nx,ny
            if not (0<=nx<176 and 0<=ny<160) or plan[ny,nx]:
                continue
            if dx and dy and (plan[y,nx] or plan[ny,x]):
                continue
            cost = costs[current]+(7 if dx and dy else 5)+(0 if central[ny,nx] else 3)+(0 if (int(heading[ny,nx])>>(direction^1))&1 else 20)
            if cost < costs.get(neighbour, 999999):
                costs[neighbour], parents[neighbour] = cost,current
                heapq.heappush(pending,(cost+heuristic(nx,ny),-cost,ny*176+nx,neighbour))


def smooth(route, passes):
    result = route.copy()
    for _ in range(passes):
        for i in range(1,len(result)-1):
            nx,ny = (result[i-1]+2*result[i]+result[i+1])//4
            if not plan[ny//512,nx//512]:
                result[i] = nx,ny
    return result


def check(points):
    ox,oy = np.meshgrid(np.arange(-25,26), np.arange(-25,26))
    for i,b in enumerate(points):
        a,c = points[max(0,i-1)],points[min(len(points)-1,i+1)]
        tangent = c-b if i<len(points)-1 else b-a
        hx,hy = map(int,tangent)
        length = math.hypot(hx,hy)
        if not length:
            return 'zero',i
        if i and i+1<len(points):
            ab,bc = b-a,c-b
            cross = abs(int(ab[0])*int(bc[1])-int(ab[1])*int(bc[0]))
            if np.dot(ab,bc)<=0 or np.linalg.norm(ab)*np.linalg.norm(bc)*np.linalg.norm(c-a)<cross*24*256:
                return 'radius',i
        rx,ry = ox*256+128-int(b[0])%256,oy*256+128-int(b[1])%256
        forward,side = (rx*hx+ry*hy)/length,(ry*hx-rx*hy)/length
        inside = (forward>=-7.8*256)&(forward<=19.8*256)&(np.abs(side)<=10.3*256)
        xx,yy = int(b[0])//256+ox[inside],int(b[1])//256+oy[inside]
        if np.any(xx<0) or np.any(xx>=352) or np.any(yy<0) or np.any(yy>=320):
            return 'boundary',i
        if np.any(raw[yy,xx]):
            return 'body',i,tuple(b/256)
    return 'ok',len(points)


if __name__ == '__main__':
    for start,goal in [((13,125),(45,146)),((12,125),(45,148)),((13,110),(45,148))]:
        route = solve(start,goal)
        if route is None:
            print(start,goal,'no path')
        else:
            for passes in (32,64,128,256):
                print(start,goal,passes,check(smooth(route,passes)))
