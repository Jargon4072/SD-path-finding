#!/usr/bin/env python
import asyncio
import websockets
import json
import math
import csv
from BehaviourPlanner import BehaviourPlanner
from scipy import interpolate
import os


def distance(x1, y1, x2, y2):
  #print(x1,y1,x2,y2)
  return math.sqrt(math.pow((x2-x1),2)+math.pow((y2-y1),2))


def getXY(s,d,maps_s,maps_x,maps_y):

  prev_wp = -1

  while(s > float(maps_s[prev_wp+1]) and (prev_wp < (len(maps_s)-1) )):
    prev_wp+=1

  wp2 = (prev_wp+1)%len(maps_x)

  heading = math.atan2((float(maps_y[wp2])-float(maps_y[prev_wp])),(float(maps_x[wp2])-float(maps_x[prev_wp])))
  #the x,y,s along the segment
  seg_s = s-float(maps_s[prev_wp])


  seg_x = float(maps_x[prev_wp])+seg_s*math.cos(heading)
  seg_y = float(maps_y[prev_wp])+seg_s*math.sin(heading)

  perp_heading = heading-math.pi/2

  x = seg_x + d*math.cos(perp_heading)
  y = seg_y + d*math.sin(perp_heading)

  return x,y




def ClosestWaypoint(x,y,maps_x,maps_y) :

  #print(x,y,maps_x,maps_y)
  closestLen = 100000
  closestWaypoint = 0

  for i in range (0,len(maps_x)):

    map_x = float(maps_x[i])
    map_y = float(maps_y[i])
    #print(map_x,map_y)
    dist = distance(x,y,map_x,map_y)
    if(dist < closestLen):
      closestLen = dist
      closestWaypoint = i

  return closestWaypoint


def NextWaypoint(x,y,theta,maps_x,maps_y):

  #print(x,y,theta,maps_x,maps_y)
  closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y)

  map_x = float(maps_x[closestWaypoint])
  map_y = float(maps_y[closestWaypoint])

  heading = math.atan2( (map_y-y),(map_x-x) ) #arctan2

  angle = abs(theta-heading)

  if(angle > math.pi/4):
    closestWaypoint+=1

  return closestWaypoint

def getFrenet(x,y,theta,maps_x,maps_y):

  #print(x,y,theta,maps_x,maps_y)
  next_wp = NextWaypoint(x,y, theta, maps_x,maps_y)
  #prev_wp
  prev_wp = next_wp-1
  if(next_wp == 0):
    prev_wp  = len(maps_x)-1

  n_x = float(maps_x[next_wp])-float(maps_x[prev_wp])
  n_y = float(maps_y[next_wp])-float(maps_y[prev_wp])
  x_x = x - float(maps_x[prev_wp])
  x_y = y - float(maps_y[prev_wp])

  # find the projection of x onto n
  proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
  proj_x = proj_norm*n_x
  proj_y = proj_norm*n_y

  frenet_d = distance(x_x,x_y,proj_x,proj_y)

  #see if d value is positive or negative by comparing it to a center point

  center_x = 1000-float(maps_x[prev_wp])
  center_y = 2000-float(maps_y[prev_wp])
  centerToPos = distance(center_x,center_y,x_x,x_y)
  centerToRef = distance(center_x,center_y,proj_x,proj_y)

  if(centerToPos <= centerToRef):
    frenet_d *= -1

  # calculate s value
  frenet_s = 0
  for i in range (0,prev_wp):
    frenet_s += distance(float(maps_x[i]),float(maps_y[i]),float(maps_x[i+1]),float(maps_y[i+1]))

  frenet_s += distance(0,0,proj_x,proj_y)

  return frenet_s,frenet_d

async def hello(websocket,path):
    data= await websocket.recv()
    #print(data)
    #await asyncio.sleep(.2)
    if len(data)>2:
    	if data[0]=='4' and data[1]=='2':
    		#print(data)
    		str1="[";
    		x=data.find(str1)
    		str2="{";
    		y=data.find(str2)
    		j=json.loads(data[2:])
    		event=j[0];
    		#print(event)
    		if event=="telemetry":
                 #print(j[1])
                 car_x = j[1]["x"];
                 car_y = j[1]["y"];
                 car_s = j[1]["s"];
                 car_d = j[1]["d"];
                 car_yaw = j[1]["yaw"];
                 car_speed = j[1]["speed"];
                 #Previous path data given to the Planner
                 previous_path_x = j[1]["previous_path_x"];
                 previous_path_y = j[1]["previous_path_y"];
                 # Previous path's end s and d values
                 end_path_s = j[1]["end_path_s"];
                 end_path_d = j[1]["end_path_d"];
                 sensor_fusion=j[1]["sensor_fusion"];
                 print(car_speed)
                 #print(sensor_fusion)
                 next_x_vals=[];
                 next_y_vals=[];
                 path_size = len(previous_path_x);
                 #print (path_size)

                 #Start with remaining old path
                 for i in range(0,path_size):
                    next_x_vals.append(previous_path_x[i])
                    next_y_vals.append(previous_path_y[i])


                 ptsx=[];
                 ptsy=[];

                 ref_x = car_x
                 ref_y = car_y
                 ref_yaw = math.radians(car_yaw)

                 #print(ref_x)
                 #print(ref_yaw)
                 #print(ref_y)


                 if path_size < 2 :
                    prev_car_x = car_x - math.cos(car_yaw)
                    prev_car_y = car_y - math.sin(car_yaw)
                    ptsx.append(prev_car_x)
                    ptsx.append(car_x)
                    ptsy.append(prev_car_y)
                    ptsy.append(car_y)
                    ref_vel = car_speed

                 else : #Otherwise, use previous x and y and calculate angle based on change in x & y
                    ref_x = previous_path_x[path_size-1]
                    ref_y = previous_path_y[path_size-1]
                    ref_x_prev = previous_path_x[path_size-2]
                    ref_y_prev = previous_path_y[path_size-2]
                    ref_yaw = math.atan2(ref_y-ref_y_prev,ref_x-ref_x_prev)
                    print("pathsize>2")
                    ref_vel = bp.target_vehicle_speed

                    #Append starter points for spline later
                    ptsx.append(ref_x_prev)
                    ptsx.append(ref_x)
                    ptsy.append(ref_y_prev)
                    ptsy.append(ref_y)



                 frenet_vec=[0,0];
                 # Plan the rest of the path based on calculations
                 #print(ref_x,ref_y,ref_yaw)
                 frenet_vec[0],frenet_vec[1] = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y)
                 print(frenet_vec[0],frenet_vec[1])
                 move = bp.lanePlanner(frenet_vec[0], frenet_vec[1], sensor_fusion)
                 lane = bp.curr_lane
                 print(lane)
                 next_d = (lane * 4) + 2 + move
                 # Double-check that the car has not incorrectly chose a blocked lane
                 check_lane = bp.laneCalc(next_d)
                 print(check_lane)
                 front_vehicle=[1.0,1.0];
                 back_vehicle=[1.0,1.0];
                 front_vehicle[0],front_vehicle[1] = bp.closestVehicle(frenet_vec[0], check_lane, sensor_fusion, True)
                 back_vehicle[0],back_vehicle[1] = bp.closestVehicle(frenet_vec[0], check_lane, sensor_fusion, False)
                 #Reset to current lane and leading vehicle if not enough room
                 if front_vehicle[0] < 10 or back_vehicle[0] < 10 or bp.avg_scores[check_lane] <= -5 :
                    next_d = (lane* 4) + 2
                    if check_lane != lane :
                        bp.target_vehicle_speed = bp.curr_lead_vehicle_speed



                 wp1=[0,0];
                 wp2=[0,0];
                 wp3=[0,0];

                 wp1[0],wp1[1] = getXY(car_s+50, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y)
                 wp2[0],wp2[1] = getXY(car_s+100, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y)
                 wp3[0],wp3[1] = getXY(car_s+150, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y)
                 ptsx.append(wp1[0])
                 ptsx.append(wp2[0])
                 ptsx.append(wp3[0])

                 ptsy.append(wp1[1])
                 ptsy.append(wp2[1])
                 ptsy.append(wp3[1])

                 if len(ptsx) > 2:   # Spline fails if not greater than two points - Otherwise just use rest of old pat
                 #Shift and rotate points to local coordinates
                    for i in range(0,len(ptsx)):
                        shift_x = ptsx[i] - ref_x
                        shift_y = ptsy[i] - ref_y

                        ptsx[i] = (shift_x*math.cos(0-ref_yaw)-shift_y*math.sin(0-ref_yaw))
                        ptsy[i] = (shift_x*math.sin(0-ref_yaw)+shift_y*math.cos(0-ref_yaw))
                    #create a spline
                    #set (x,y) points to the spline
                    #ptsx.sort()
                    #ptsy.sort()
                    #print(ptsx,ptsy)
                    #yx=[x for _, x in sorted(zip(ptsx,ptsy))]
                    #print(yx)
                    #yx.sort()
                    #ptsx.sort()
                    tck=interpolate.CubicSpline(ptsx,ptsy)
                    target_x = 30
                    target_y = tck(target_x)
                    print(target_y)
                    target_dist = math.sqrt(math.pow(target_x,2)+math.pow(target_y,2))
                    #print(target_dist)
                    x_add_on = 0
                    MAX_ACCEL= 10 # m/s/s
                    accel = (MAX_ACCEL) * 0.02 * 0.8 # Limit acceleration within acceptable range

                    for i in range(0,50-path_size) :
                        print(ref_vel,bp.target_vehicle_speed)
                        if float(ref_vel) < (float(bp.target_vehicle_speed )- accel) :  #Accelcmerate if under target speed
                            ref_vel += accel
                            print("here")
                        elif float(ref_vel) > (float(bp.target_vehicle_speed) + accel) : # Brake if below target
                            ref_vel -= accel
                        #Calculate points along new path
                        N = (target_dist/(.02*float(ref_vel)))
                        x_point = x_add_on+(target_x)/N
                        y_point = tck(x_point)
                        x_add_on = x_point
                        x_ref = x_point
                        y_ref = y_point
                        #Rotate and shift back to normal
                        x_point = (x_ref*math.cos(ref_yaw)-y_ref*math.sin(ref_yaw))
                        y_point = (x_ref*math.sin(ref_yaw)+y_ref*math.cos(ref_yaw))
                        x_point += ref_x
                        y_point += ref_y
                        next_x_vals.append(x_point)
                        next_y_vals.append(y_point)
                    bp.target_vehicle_speed = ref_vel  # Save the end speed to be used for the next path




                 msgJson={"next_x": next_x_vals, "next_y": next_y_vals}
                 msg = "42[\"control\","+ json.dumps(msgJson)+"]";
                 await websocket.send(msg);






    else :
        str3="42[\"manual\",{}]";
        await websocket.send(str3)
        print("sent")

if __name__ == "__main__" :
  map_waypoints_x=[];
  map_waypoints_y=[];
  map_waypoints_s=[];
  map_waypoints_dx=[];
  map_waypoints_dy=[];
  bp=BehaviourPlanner()

  #Waypoint map to read from
  #string map_file_ = "../data/highway_map.csv";
  #The max s value before wrapping around the track back to 0
  with open('highway_map.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=' ')
    for row in readCSV:
        #print(row[0])
        #print(row[0],row[1],row[2],)
        map_waypoints_x.append(row[0])
        map_waypoints_y.append(row[1])
        map_waypoints_s.append(row[2])
        map_waypoints_dx.append(row[3]);
        map_waypoints_dy.append(row[4])

  max_s = 6945.554


  start_server = websockets.serve(hello, 'localhost', 4567)
  asyncio.get_event_loop().run_until_complete(start_server)
  asyncio.get_event_loop().run_forever()
