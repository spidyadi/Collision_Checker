import numpy as np
import scipy.spatial
from math import sin, cos, pi, sqrt

class DynamicCollisionChecker:
    def __init__(self, circle_offset, circle_radii, growth_factor):
        self._circle_offset = circle_offset
        self._circle_radii   = circle_radii
        self._growth_factor	 = growth_factor


	# Takes in a set of paths and obstacles, and returns an array
	# of bools that says whether or not each path is collision free.
	def dynamic_collision_check(paths, ego_state, other_vehicle_states, look_ahead_time):
		""" Returns a bool array on whether each path is collision free.
		
		args:
				paths: A list of paths in the global frame.  
					A path is a list of points of the following format:
						[x_points, y_points, t_points]:
							x_points: List of x values (m)
							y_points: List of y values (m)
							t_points: List of yaw values (rad)
						Example of accessing the ith path, jth point's t value:
							paths[i][2][j]
				ego_state: ego state vector for the vehicle. (global frame)
					format: [ego_x, ego_y, ego_yaw, ego_speed]
						ego_x and ego_y     : position (m)
						ego_yaw             : top-down orientation [-pi to pi] (ground frame)
						ego_speed : speed (m/s)
				other_vehicle_states: other vehicles' state vectors
					Each state vector is of the format (global frame):
						[pos_x, pos_y, yaw, speed]
							pos_x and pos_y	: position (m)
							yaw 			: top-down orientation [-pi to pi] (ground frame)
							speed 			: speed (m/s)
						Example of accessing the ith car's speed would be:
							other_vehicle_states[i][3]
				look_ahead_time: The look ahead time to which the paths have been generated (s)

			returns:
				collision_check_array: A list of boolean values which classifies
					whether the path is collision-free (true), or not (false). The
					ith index in the collision_check_array list corresponds to the
					ith path in the paths list.
		"""
		time_step = look_ahead_time/len(paths[0][0])

		other_vehicle_paths = np.zeros((len(other_vehicle_states), 3, len(paths[0][0])), dtype=float)

		for i in range(len(other_vehicle_paths)):
			vehicle_state = other_vehicle_states[i]
			vehicle_path = np.zeros((3, len(paths[0][0])), dtype = float)

			vehicle_path[0] = vehicle_state[0] + time_step*vehicle_state[3]*cos(vehicle_state[2])*range(1, len(paths[0][0])+1, 1)
			vehicle_path[1] = vehicle_state[1] + time_step*vehicle_state[3]*sin(vehicle_state[2])*range(1, len(paths[0][0])+1, 1)
			vehicle_path[2] = vehicle_state[2]

			other_vehicle_paths[i] = vehicle_path

		collision_check_array = np.zeros(len(paths), dtype=bool)
        for i in range(len(paths)):
            collision_free = True
            path           = paths[i]

            for j in range(len(path[0])):

            	ego_circle_locations = np.zeros((1, 2))
            	
            	ego_circle_locations[:, 0] = path[0][j] + self._circle_offset*cos(path[2][j])
                ego_circle_locations[:, 1] = path[1][j] + self._circle_offset*sin(path[2][j])

                for k in range(len(other_vehicle_states)):
                	other_circle_locations = np.zerso((1,2))

                	other_circle_locations[:, 0] = other_vehicle_paths[k][0][j] + self._circle_offset*cos(other_vehicle_paths[k][2][j])
                	other_circle_locations[:, 1] = other_vehicle_paths[k][1][j] + self._circle_offset*sin(other_vehicle_paths[k][2][j])

                	collision_dists = scipy.spatial.distance.cdist(other_circle_locations, ego_circle_locations)
                	collision_dists = np.subtract(collision_dists, self._circle_radii*(2 + self._growth_factor*time_step*(j+1)))
                	collision_free = collision_free and not np.any(collision_dists < 0)

                	if not collision_free:
                        break
                if not collision_free:
                    break

            collision_check_array[i] = collision_free

        return collision_check_array






