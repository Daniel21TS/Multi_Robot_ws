#!/usr/bin/python3

import rospy
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PointStamped
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
import tf.msg
import tf2_ros
import tf.transformations as tf_trans
import tf2_msgs.msg


def filter_frontier_points(frontier_points, threshold_distance):
    # Function to filter frontier points by joining close points into one

    # Initialize a list to store the filtered frontier points
    filtered_points = []

    # Iterate over each frontier point
    while frontier_points:
        # Take the first point as a reference
        reference_point = frontier_points.pop(0)
        reference_x, reference_y = reference_point.point.x, reference_point.point.y

        # Iterate over the remaining frontier points
        i = 0
        while i < len(frontier_points):
            # Calculate the Euclidean distance between the reference point and the current point
            current_x, current_y = frontier_points[i].point.x, frontier_points[i].point.y
            distance = math.sqrt((current_x - reference_x) ** 2 + (current_y - reference_y) ** 2)

            # If the distance is below the threshold, remove the current point
            if distance <= threshold_distance:
                frontier_points.pop(i)
            else:
                i += 1

        # Add the reference point to the filtered points
        filtered_points.append(reference_point)

    return filtered_points

class MultiFrontierExploration:
    def __init__(self):
        self.frontier_points = []  # List to store frontier points
        # Robot positions
        self.robot_0_pose = None
        self.robot_1_pose = None

        # ROS node initialization
        rospy.init_node('multi_frontier_exploration')

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to TF topic for each robot
        self.tf_sub_robot_0 = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback_robot_0)
        self.tf_sub_robot_1 = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback_robot_1)

        # Subscribe to the occupancy grid map
        rospy.Subscriber('/map_merge', OccupancyGrid, self.map_callback)

        # Publisher for frontier points
        self.frontier_pub_0 = rospy.Publisher('/frontier_points_robot_0', PointStamped, queue_size=10)

        # Publisher for frontier points as markers
        self.frontier_marker_pub = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=10)

        # Publisher for MoveBaseGoal messages
        self.move_base_pub_0 = rospy.Publisher('/robot_0/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.move_base_pub_1 = rospy.Publisher('/robot_1/move_base/goal', MoveBaseActionGoal, queue_size=10)

    
    def map_callback(self, msg):
        # Perform frontier extraction on the received map
        self.extract_frontiers(msg)
    #----------------------------------------------------------------------------------------------------------------
        # Assign frontier points to diffrent robots based on distance
        frontier_points_robot_0 = []
        frontier_points_robot_1 = []

        # Nearest Point Method
        for point in self.frontier_points:
            # Calculate distances to each robot's position
            distance_robot_0 = self.calculate_distance(point, self.robot_0_pose)
            distance_robot_1 = self.calculate_distance(point, self.robot_1_pose)
            #print("Distance 0 {}".format(distance_robot_0))
            #print("Distance 1 {}".format(distance_robot_1))
            #print("Robot_0 Pose x={} y={}".format(self.robot_0_pose.x,self.robot_0_pose.y))
            #print("Robot_1 Pose x={} y={}".format(self.robot_1_pose.x,self.robot_1_pose.y))

            # Assign to the closest robot
            if distance_robot_0 < distance_robot_1:
                frontier_points_robot_0.append(point)
            else:
                frontier_points_robot_1.append(point)

        # Falf Map Method
#        for point in self.frontier_points:
#            # Compare the x-coordinate of the frontier point with the midpoint of the map
#            if point.point.x <= 2:
#                frontier_points_robot_0.append(point)  # Assign the frontier point to robot_0 (left part)
#            else:
#                frontier_points_robot_1.append(point)  # Assign the frontier point to robot_1 (right part)


        # Update the frontier points for each robot
        self.frontier_points_robot_0 = frontier_points_robot_0
        self.frontier_points_robot_1 = frontier_points_robot_1
    #----------------------------------------------------------------------------------------------------------------
        # Publish frontier points as markers
        marker_array = MarkerArray()

        # Create markers for frontier points of robot_0
        for i, point in enumerate(self.frontier_points_robot_0):
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id  # Use the frame ID of the occupancy grid
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point.point.x
            marker.pose.position.y = point.point.y
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        
        # Create markers for frontier points of robot_1
        for i, point in enumerate(self.frontier_points_robot_1):
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id  # Use the frame ID of the occupancy grid
            marker.id = i + len(self.frontier_points_robot_0)  # Offset the ID to avoid collision
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point.point.x
            marker.pose.position.y = point.point.y
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        # Publish the marker array
        self.frontier_marker_pub.publish(marker_array)
    #----------------------------------------------------------------------------------------------------------------
        # Publish frontier points
        #for point in self.frontier_points_robot_0:
        #    self.frontier_pub_0.publish(point)
    #----------------------------------------------------------------------------------------------------------------
        # Print frontier points on the console
        print("Frontier Points Robot_0:")
        for i, point in enumerate(self.frontier_points_robot_0):
            print("Point {}: x={}, y={}".format(i+1, point.point.x, point.point.y))

        # Print frontier points on the console
        print("Frontier Points Robot_1:")
        for i, point in enumerate(self.frontier_points_robot_1):
            print("Point {}: x={}, y={}".format(i+1, point.point.x, point.point.y))
    #----------------------------------------------------------------------------------------------------------------
        # Send the robot to the first frontier point
        #if self.frontier_points:
            #self.send_goal(self.frontier_points[0])
    #----------------------------------------------------------------------------------------------------------------
        # Send the robot_0 to the nearest frontier point 
        nearest_point_0 = None
        nearest_distance_0 = float('inf')

        for point_0 in self.frontier_points_robot_0:
            distance_0 = math.sqrt((point_0.point.x - self.robot_0_pose.x) ** 2 + (point_0.point.y - self.robot_0_pose.y) ** 2)
            if distance_0 < nearest_distance_0:
                nearest_point_0 = point_0
                nearest_distance_0 = distance_0

        if nearest_point_0 is not None:
            self.send_goal_0(nearest_point_0.point)
    #----------------------------------------------------------------------------------------------------------------
        # Send the robot_1 to the nearest frontier point 
        nearest_point_1 = None
        nearest_distance_1 = float('inf')

        for point_1 in self.frontier_points_robot_1:
            distance_1 = math.sqrt((point_1.point.x - self.robot_1_pose.x) ** 2 + (point_1.point.y - self.robot_1_pose.y) ** 2)
            if distance_1 < nearest_distance_1:
                nearest_point_1 = point_1
                nearest_distance_1 = distance_1

        if nearest_point_1 is not None:
            self.send_goal_1(nearest_point_1.point)
    #----------------------------------------------------------------------------------------------------------------
        # If the robot reach the target point, delete this point

        
    def extract_frontiers(self, occupancy_grid):
        frontiers = []

        # Get the map dimensions and resolution
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution

        # Convert the occupancy grid data to a 2D matrix
        grid = [[0] * width for _ in range(height)]
        for i in range(height):
            for j in range(width):
                index = j + width * i
                grid[i][j] = occupancy_grid.data[index]
        
        # Iterate over the grid to find frontier points
        for i in range(height):
            for j in range(width):
                if grid[i][j] == 0:  # Unoccupied cell
                    neighbors = self.get_neighbors(grid, i, j)
                    for neighbor in neighbors:
                        if grid[neighbor[0]][neighbor[1]] == -1:  # Unknown cell
                            frontiers.append((i, j))
                            break

        # Convert frontier points to PointStamped messages
        self.frontier_points = []
        for frontier in frontiers:
            x = (frontier[1] + 0.5) * resolution + occupancy_grid.info.origin.position.x
            y = (frontier[0] + 0.5) * resolution + occupancy_grid.info.origin.position.y
            point = PointStamped()
            point.point.x = x
            point.point.y = y
            point.point.z = 0.0
            self.frontier_points.append(point)
        #----------------------------------------------------------------------------------------------------------------
        # Filter frontiers points
        threshold_distance = 1.0
        self.frontier_points = filter_frontier_points(self.frontier_points, threshold_distance)
        #----------------------------------------------------------------------------------------------------------------
        # Filter frontiers points close to walls! Check if there's a occ cell on a 6x6 grid around the frontier point
        filtered_points = []
        for point in self.frontier_points:
            # Convert frontier point coordinates to grid indices
            grid_x = int((point.point.x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution)
            grid_y = int((point.point.y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution)

            # Check if the point is close to an occupied cell in the occupancy grid
            close_to_wall = False
            for dx in range(-2, 4):  # Modified range for x-axis (-2 to 3)
                for dy in range(-2, 4):  # Modified range for y-axis (-2 to 3)
                    neighbor_x = grid_x + dx
                    neighbor_y = grid_y + dy

                    # Check if the neighbor cell is an occupied cell (value = 100 in the occupancy grid)
                    if 0 <= neighbor_x < occupancy_grid.info.width and 0 <= neighbor_y < occupancy_grid.info.height:
                        cell_index = neighbor_y * occupancy_grid.info.width + neighbor_x
                        if occupancy_grid.data[cell_index] == 100:
                            close_to_wall = True
                            break

                if close_to_wall:
                    break

            # If the point is not close to a wall, add it to the filtered points
            if not close_to_wall:
                filtered_points.append(point)

        self.frontier_points = filtered_points
    #----------------------------------------------------------------------------------------------------------------

    def get_neighbors(self, grid, i, j):
        neighbors = []
        height = len(grid)
        width = len(grid[0])

        if i > 0:
            neighbors.append((i - 1, j))  # Upper neighbor
        if i < height - 1:
            neighbors.append((i + 1, j))  # Lower neighbor
        if j > 0:
            neighbors.append((i, j - 1))  # Left neighbor
        if j < width - 1:
            neighbors.append((i, j + 1))  # Right neighbor

        return neighbors
    
    def send_goal_0(self, point):
        goal_msg = MoveBaseActionGoal()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.goal.target_pose.header.frame_id = "map_merged"
        goal_msg.goal.target_pose.pose.position.x = point.x
        goal_msg.goal.target_pose.pose.position.y = point.y
        goal_msg.goal.target_pose.pose.orientation.w = 1.0
        self.move_base_pub_0.publish(goal_msg)

    def send_goal_1(self, point):
        goal_msg = MoveBaseActionGoal()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.goal.target_pose.header.frame_id = "map_merged"
        goal_msg.goal.target_pose.pose.position.x = point.x
        goal_msg.goal.target_pose.pose.position.y = point.y
        goal_msg.goal.target_pose.pose.orientation.w = 1.0
        self.move_base_pub_1.publish(goal_msg)

    def tf_callback_robot_0(self, tf_msg):
        try:
            # Lookup the transform between the map frame and the robot's base frame
            transform = self.tf_buffer.lookup_transform("robot_0/cmap", "robot_0/base_link", rospy.Time())

            # Extract the position from the transform
            self.robot_0_pose = transform.transform.translation

            # Print the robot's pose
            #rospy.loginfo("Robot Pose: x={}, y={}".format(self.robot_pose.x, self.robot_pose.y))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to retrieve transform for robot_0")

    def tf_callback_robot_1(self, tf_msg):
        try:
            # Lookup the transform between the map frame and the robot's base frame
            transform = self.tf_buffer.lookup_transform("robot_1/cmap", "robot_1/base_link", rospy.Time())

            # Extract the position from the transform
            self.robot_1_pose = transform.transform.translation

            # Print the robot's pose
            #rospy.loginfo("Robot Pose: x={}, y={}".format(self.robot_pose.x, self.robot_pose.y))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to retrieve transform for robot_1")
    
    def calculate_distance(self, point1, point2):
        if point1 is None or point2 is None:
            rospy.logwarn("One or both points are None. Cannot calculate distance.")
            return None
        # Extract x and y coordinates from PointStamped objects
        x1 = point1.point.x
        y1 = point1.point.y
        x2 = point2.x
        y2 = point2.y

        # Calculate Euclidean distance between the points
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return distance

if __name__ == '__main__':

    frontier_extractor = MultiFrontierExploration()
    rospy.spin()