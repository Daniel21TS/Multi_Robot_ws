#!/usr/bin/python3
#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

# ***************
#   OBSTACLE
M = 75
#   unknown
N = 50
#   free
# ----0-----
#   unknown
# ***************
def callback(cmap: OccupancyGrid):

    data = list(cmap.data)
    for y in range(cmap.info.height):
        for x in range(cmap.info.width):
            i = x + (cmap.info.height - 1 - y) * cmap.info.width
            if data[i] >= M:  
                data[i] = 100
            elif (data[i] >= 0) and (data[i] < N):  # free
                data[i] = 0
            else:  # unknown
                data[i] = -1
    cmap.data = tuple(data)
    pub.publish(cmap)

if __name__ == '__main__':
    rospy.init_node('map_to_cmap_converter')
    pub = rospy.Publisher('cmap', OccupancyGrid, queue_size=10)
    rospy.Subscriber('map', OccupancyGrid, callback)
    rospy.spin()











def calculate_information_gain(self, map, frontier_points):
         # Step 1: Define a Probability Distribution
 
        
        # Step 2: Compute Initial Entropy (all map)

        
        # Iterate over each frontier point
        for size, distance, frontier_point in frontier_points:
            # Step 3: Simulate Hypothetical Measurements
            # Retrieve the robot's pose from the frontier point
            robot_pose = self.robot_pose  # Assuming the frontier point provides the robot's pose
            
            # Define the laser scanner parameters
            num_scan_points = 360
            min_range = 0.10
            max_range = 5.0
            angle_min = -1.570796
            angle_max = 1.570796
            
            # Simulate laser scan by casting rays from the robot's pose
            scan_angles = np.linspace(angle_min, angle_max, num_scan_points)
            scan_ranges = []
            
            for angle in scan_angles:
                # Calculate the direction of the ray based on the robot's pose and the current scan angle
                ray_direction = np.array([math.cos(angle), math.sin(angle), 0.0])
                
                # Cast the ray from the robot's position and obtain the intersection point with the occupancy grid map
                intersection_point = self.cast_ray(robot_pose, ray_direction, map)
                
                if intersection_point is None:
                    # No intersection found, set the range to maximum
                    range_value = max_range
                else:
                    # Calculate the Euclidean distance between the robot and the intersection point
                    range_value = np.linalg.norm(intersection_point - np.array([robot_pose.x, robot_pose.y, robot_pose.z]))

                    
                    # Apply sensor noise or uncertainties if necessary
                    
                scan_ranges.append(range_value)
            
            # Step 4: Update the Probability Distribution
            grid_indices = self.convert_to_grid_indices(frontier_point.point)
            neighborhood_size = 3  # Adjust the size according to your requirements
            
            # Iterate over the neighborhood of grid cells
            for i in range(-neighborhood_size, neighborhood_size + 1):
                for j in range(-neighborhood_size, neighborhood_size + 1):
                    neighbor_indices = (grid_indices[0] + i, grid_indices[1] + j)
                    
                    if self.is_within_map_bounds(neighbor_indices, (map.info.width, map.info.height)):
                        occupancy_value = map.data[neighbor_indices[1] * map.info.width + neighbor_indices[0]]
                        
                        if occupancy_value == 100:
                            probability_distribution[neighbor_indices] = 1.0
                        elif occupancy_value == 0:
                            probability_distribution[neighbor_indices] = 0.0
                        # Uncomment the following lines if you want to set uncertain cells to 0.5 probability
                        #elif occupancy_value == -1:
                        #    probability_distribution[neighbor_indices] = 0.5
            
            # Normalize the probability distribution
            probability_distribution /= np.sum(probability_distribution)
            
            # Step 5: Calculate Entropy After Exploration
            probabilities = []
        
            # Simulate hypothetical measurements for the current frontier point
            for angle in scan_angles:
                # Calculate the direction of the ray based on the robot's pose and the current scan angle
                ray_direction = np.array([math.cos(angle), math.sin(angle), 0.0])

                # Cast the ray from the robot's position and obtain the intersection point with the occupancy grid map
                intersection_point = self.cast_ray(robot_pose, ray_direction, map)
                
                if intersection_point is None:
                    # No intersection found, set the probability to 0.0 (unknown)
                    probability = 0.0
                else:
                    # Calculate the grid cell indices of the intersection point
                    intersection_indices = self.convert_to_grid_indices(intersection_point)
                    
                    # Calculate the probability of occupancy based on the updated probability distribution
                    probability = probability_distribution[intersection_indices]
                
                probabilities.append(probability)
            
            # Calculate the entropy after exploration
            entropy_after_exploration = -sum(p * math.log2(p) for p in probabilities if p != 0)
            
            
            # Step 6: Calculate Information Gain
            information_gain = initial_entropy - entropy_after_exploration

            # Create a tuple or list with size and middle_point
            frontier_data = (size, distance, information_gain, frontier_point)
            frontier_points.append(frontier_data)
        
        return frontier_points