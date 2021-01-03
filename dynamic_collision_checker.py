import numpy as np
from math import sin, cos, pi, sqrt

class dynamic_collision_check:
    def __init__(self, paths,other_vechile_state,circle_radius):
        self.other_vechile_velx = other_vechile_state[0]  #velx
        self.other_vechile_vely = other_vechile_state[1]    #vely
        self.path = paths
        self.radius = circle_radius                              # footprint can be changed to two overlapping circle as in turtlebot
        self.min_dist_bet_vech = np.zeros((len(paths[0][0]),1))  # but  needs yaw of the vechile and be diff to implement. but will make 
                                                                 #    better obstacle avoidance     
        for i in range(len(paths[0][0])):
            self.min_dist_bet_vech[i] = 2*self.radius + i * 0.05  # increase in radius has to be changed

    def generate_other_vechile_waypoints(self, other_vech_position, look_ahead_time_of_planner):
        x0 = other_vech_position[0]
        y0 = other_vech_position[1]
        other_vech_waypoints = np.zeros((len(self.path[0][0]),2))
        dt = look_ahead_time_of_planner/len(self.path[0][0])
        for i in range(1,len(self.path[0][0])):
            other_vech_waypoints[i-1][0] = x0 + self.other_vechile_velx*i*dt
            other_vech_waypoints[i-1][1] = y0 + self.other_vechile_vely*i*dt

        return other_vech_waypoints

    def Dynamic_collision_check(self, other_vech_waypoints):
        collision_check_arr = np.zeros((len(self.path)))
        for i in range(len(self.path)):
            path = self.path[i]
            collision_free = True

            x_dist = np.square(path[0] - other_vech_waypoints[:,0])
            y_dist = np.square(path[1] - other_vech_waypoints[:,1])

            obstacle_dist = np.sqrt(x_dist + y_dist)
            collision_check = obstacle_dist - self.min_dist_bet_vech
            collision_free = collision_free and not np.any(collision_check < 0)
            
            collision_check_arr[i] = collision_free

        return collision_check_arr

ego_waypoints = np.random.randint(10,size=(20,2,30))
other_car_state = [1,4]
R = 5
other_vech_pos = [1,2]
Time_ahead = 1

object = dynamic_collision_check(ego_waypoints,other_car_state,R)
wayp = object.generate_other_vechile_waypoints(other_vech_pos,Time_ahead)
collision_ch = object.Dynamic_collision_check(wayp)
print(collision_ch)
            



                


