#!/usr/bin/python
import math

class BehaviourPlanner:

    def __init__(self):
      self.curr_lane=None
      self.curr_lead_vehicle_speed = 22.352 - 0.5
      self.target_vehicle_speed=0.0;
      self.avg_scores = [0,0,0];

    
    def laneCalc(self,d):
    	# Check which lane the d-value comes from
    	#Left is 0, middle is 1, right is 2
      if (d < 4):
        lane = 0
      elif (d < 8):
        lane = 1
      else:
        lane = 2
      return lane


    def closestVehicle(self,s,lane,sensor_fusion,direction):
      dist = 10000
      velocity = 22.352 - 0.5 #Set in case of no cars
    	#Check each vehicle in sensor range
      for vehicle in range(0,len(sensor_fusion)):
        vehicle_s = sensor_fusion[vehicle][5]
        vehicle_d = sensor_fusion[vehicle][6]
        vehicle_v = math.sqrt(math.pow(sensor_fusion[vehicle][3], 2)+math.pow(sensor_fusion[vehicle][4], 2))
        vehicle_lane = BehaviourPlanner.laneCalc(self,vehicle_d)


        if (vehicle_lane == lane): # { // if same lane
          if (direction == True):
            if (vehicle_s > s and (vehicle_s - s) < dist) : #{ // and ahead of my vehicle
              dist = vehicle_s - s
              velocity = vehicle_v
          else:
            if (s >= vehicle_s and (s - vehicle_s) < dist): #{ // if behind my vehicle
              dist = s - vehicle_s
              velocity = vehicle_v
      
      if (dist <= 0) :  #Avoid dividing by zero in laneScore()
        dist = 1.0

      if (lane == self.curr_lane and direction == True):
        self.curr_lead_vehicle_speed = velocity

      return dist, velocity




    def laneScore(self,s,lane,sensor_fusion):
      scores = [0,0,0]

      for i in range(0,3): # {
        if (i == lane): # {  #// benefit to keeping lane
          scores[i] += 1.0

        front_vehicle=[1.0,1.0];
        back_vehicle=[1.0,1.0];

        front_vehicle[0],front_vehicle[1] = BehaviourPlanner.closestVehicle(self,s, i, sensor_fusion, True)
        back_vehicle[0],back_vehicle[1] = BehaviourPlanner.closestVehicle(self, s, i, sensor_fusion, False)

        if (front_vehicle[0] > 1000 and back_vehicle[0] > 1000):
          scores[i] += 5; # if wide open lane, move into that lane
      
        else :
          if (front_vehicle[0] < 10):
            scores[i] -= 5; # if car too close in front, negative score
        
          if (back_vehicle[0] < 10):
            scores[i] -= 5; #if car too close in back, negative score


        if(front_vehicle[0] !=0 and front_vehicle[1]!=0 and back_vehicle[0]!=0 and back_vehicle[1] !=0) :
          scores[i] += 1 - (10/(front_vehicle[0]/3))#benefit for large open distance in lane in front
          scores[i] += 1 - (10/(back_vehicle[0]/3)) # benefit for large open distance in lane in back
          scores[i] += 1 - (10/(front_vehicle[1]/2)) #benefit for faster car speed in lane in front
          scores[i] += 1 / (back_vehicle[1]/2) #benefit for slower car speed in lane in back

        
        self.avg_scores[i] = (self.avg_scores[i] * 10) - self.avg_scores[i]
        self.avg_scores[i] += scores[i]
        self.avg_scores[i] /= 10


        #Only compare applicable lanes
      #print(self.avg_scores)
      if (lane == 0):
        if(self.avg_scores[0]>self.avg_scores[1]):
          return 0
        else:
          return 1
      elif (lane == 1):
        if(self.avg_scores[0]>self.avg_scores[1] and self.avg_scores[0]>self.avg_scores[1]):
          return 0
        elif(self.avg_scores[1]>self.avg_scores[2] and self.avg_scores[1]<self.avg_scores[0]):
          return 1
        else:
          return 2
      else:
        if(self.avg_scores[1]>self.avg_scores[2]):
          return 1
        else:
          return 2
        
    def lanePlanner(self,s,d,sensor_fusion):
      lane = BehaviourPlanner.laneCalc(self,d)
      distance, velo = BehaviourPlanner.closestVehicle(self,s, lane, sensor_fusion,True)
      self.curr_lane = lane # Keep the current lane to later calculate desired move
      #check if blocked, i.e. car is within 20 meters
      # if lots of space, stay in lane and go near the speed limit
      if (distance > 20) :
        print("here")
        new_lane = lane
        self.target_vehicle_speed = 22.352 - 0.5
        self.avg_scores = [0,0,0] #// Reset average scores for laneScore()
        return 0

      else:
        new_lane = BehaviourPlanner.laneScore(self,s, lane, sensor_fusion)
        vehdist,vehvelo = BehaviourPlanner.closestVehicle(self,s, new_lane, sensor_fusion, True)
        self.target_vehicle_speed = vehvelo


      #Space between middle of each lane is four meters, so move accordingly
      if (new_lane == lane) :
        return 0
      elif (new_lane < lane):
        return -4
      else:
        return 4
