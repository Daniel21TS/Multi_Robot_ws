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
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Point


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
#----------------------------------------------------------------------------------------------------------------

class MultiFrontierExploration:
    def __init__(self):
        self.frontier_points = []  # List to store frontier points
        # Robot positions
        self.robot_0_pose = None
        self.robot_1_pose = None
        self.robot_2_pose = None

        # ROS node initialization
        rospy.init_node('multi_frontier_exploration')

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscribe to TF topic for each robot
        self.tf_sub_robot_0 = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback_robot_0)
        self.tf_sub_robot_1 = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback_robot_1)
        self.tf_sub_robot_2 = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback_robot_2)

        # Subscribe to the occupancy grid map
        rospy.Subscriber('/map_merge', OccupancyGrid, self.map_callback)

        # Publisher for frontier points as markers
        self.frontier_marker_pub = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=10)
        self.marker_publisher = rospy.Publisher('cluster_markers', Marker, queue_size=10)

        # Publisher for MoveBaseGoal messages
        self.move_base_pub_0 = rospy.Publisher('/robot_0/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.move_base_pub_1 = rospy.Publisher('/robot_1/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.move_base_pub_2 = rospy.Publisher('/robot_2/move_base/goal', MoveBaseActionGoal, queue_size=10)

    
    def map_callback(self, msg):
        # Perform frontier extraction and delete points close to walls on the received map
        self.extract_frontiers(msg)
    #----------------------------------------------------------------------------------------------------------------
        # Transform frontier points into clusters
        epsilon = 0.3
        min_samples = 5
        cluster_points = self.cluster_frontier_points(self.frontier_points, epsilon, min_samples)

        # Remove wrong clusters
        cluster_points = [points for points in cluster_points if math.sqrt((points[-1][0] - points[0][0]) ** 2 + (points[-1][1] - points[0][1]) ** 2) <= 4.5]
    #----------------------------------------------------------------------------------------------------------------
        # Visualize the clusters
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Line width
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        for i, points in enumerate(cluster_points):

            # Add the first point
            first_point = points[0]
            # Add the last point
            last_point = points[-1]

            # Add the line connecting the first and last points
            marker.points.append(Point(first_point[0], first_point[1], 0.0))
            marker.points.append(Point(last_point[0], last_point[1], 0.0))

        self.marker_publisher.publish(marker)
    #----------------------------------------------------------------------------------------------------------------
        # Calculate the size of frontier and the middle point
        table = self.calculate_cluster_sizes(cluster_points)
#        print("Cluster Sizes Table:")
#        print("Distance | Middle Point")
#        for distance, middle_point in table:
#            print(f"{distance:<9} | {middle_point}")
    #----------------------------------------------------------------------------------------------------------------
        # Share the Frontiers
        # Assign frontier points to diffrent robots based on distance
        frontier_robot_0 = []
        frontier_robot_1 = []
        frontier_robot_2 = []

        distance = 0.0

        # Nearest Point Method
        for size, middle_point in table:
            # Calculate distances to each robot's position
            distance_robot_0 = self.calculate_distance(middle_point, self.robot_0_pose)
            distance_robot_1 = self.calculate_distance(middle_point, self.robot_1_pose)
            distance_robot_2 = self.calculate_distance(middle_point, self.robot_2_pose)

            # Create a tuple or list with size and middle_point
            frontier_data = (size, distance, middle_point)

            # Assign to the closest robot
            if distance_robot_0 < distance_robot_1 and distance_robot_0 < distance_robot_2:
                distance = distance_robot_0
                frontier_robot_0.append(frontier_data)
            elif distance_robot_1 < distance_robot_0 and distance_robot_1 < distance_robot_2:
                distance = distance_robot_2
                frontier_robot_1.append(frontier_data)
            else:
                distance = distance_robot_2
                frontier_robot_2.append(frontier_data)

        # Update the frontier points for each robot
        self.frontier_robot_0 = frontier_robot_0
        self.frontier_robot_1 = frontier_robot_1
        self.frontier_robot_2 = frontier_robot_2

    #----------------------------------------------------------------------------------------------------------------
        # Tranform size and distance to score
        wD=0.3
        wS=0.7
        new_frontier_robot_0 = []
        for size,distance, middle_point in frontier_robot_0:
            score = wS*size - wD*distance 
            new_frontier_robot_0.append((size, distance, score, middle_point))

        frontier_robot_0 = new_frontier_robot_0

        new_frontier_robot_1 = []
        for size,distance, middle_point in frontier_robot_1:
            score = wS*size - wD*distance 
            new_frontier_robot_1.append((size, distance, score, middle_point))

        frontier_robot_1 = new_frontier_robot_1

        new_frontier_robot_2= []
        for size,distance, middle_point in frontier_robot_2:
            score = wS*size - wD*distance 
            new_frontier_robot_2.append((size, distance, score, middle_point))

        frontier_robot_2 = new_frontier_robot_2

        #----------------------------------------------------------------------------------------------------------------
        # Print all robots frontier points
        print("Frontier robot 0:")
        for data in frontier_robot_0:
            size, distance, score, middle_point = data
            print("size={}, distance={}, score={}, x={}, y={}".format(size, distance, score, middle_point.point.x , middle_point.point.y))

        print("Frontier robot 1:")
        for data in frontier_robot_1:
            size, distance, score, middle_point= data
            print("size={}, distance={}, score={}, x={}, y={}".format(size, distance, score, middle_point.point.x , middle_point.point.y))

        print("Frontier robot 2:")
        for data in frontier_robot_2:
            size, distance, score, middle_point= data
            print("size={}, distance={}, score={}, x={}, y={}".format(size, distance, score, middle_point.point.x , middle_point.point.y))

    #----------------------------------------------------------------------------------------------------------------
        # Send the robot_0 to the bigest frontier
        max_score_0 = -float('inf')  # Initialize with negative infinity
        max_middle_point_0 = None

        for data in frontier_robot_0:
            size, distance, score, middle_point = data
            if score > max_score_0:
                max_score_0 = score
                max_middle_point_0 = middle_point

        if max_middle_point_0 is not None:
            # Send robot 0 to the middle point with the maximum score
            self.send_goal_0(max_middle_point_0)
        else:
            print("No middle point found in frontier_robot_0")
    #----------------------------------------------------------------------------------------------------------------
        # Send the robot_1 to the bigest frontier
        max_score_1 = -float('inf')  # Initialize with negative infinity
        max_middle_point_1 = None

        for data in frontier_robot_1:
            size, distance, score, middle_point = data
            if score > max_score_1:
                max_score_1 = score
                max_middle_point_1 = middle_point

        if max_middle_point_1 is not None:
            # Send robot 1 to the middle point with the maximum score
            self.send_goal_1(max_middle_point_1)
        else:
            print("No middle point found in frontier_robot_1")
    #----------------------------------------------------------------------------------------------------------------
        # Send the robot_2 to the bigest frontier
        max_score_2 = -float('inf')  # Initialize with negative infinity
        max_middle_point_2 = None

        for data in frontier_robot_2:
            size, distance, score, middle_point = data
            if score > max_score_2:
                max_score_2 = score
                max_middle_point_2 = middle_point

        if max_middle_point_2 is not None:
            # Send robot 2 to the middle point with the maximum score
            self.send_goal_2(max_middle_point_2)
        else:
            print("No middle point found in frontier_robot_2")



        
    #----------------------------------------------------------------------------------------------------------------

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
        # Filter frontiers points close to walls! Check if there's a occ cell on a 6x6 grid around the frontier point
        filtered_points = []
        for point in self.frontier_points:
            # Convert frontier point coordinates to grid indices
            grid_x = int((point.point.x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution)
            grid_y = int((point.point.y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution)

            # Check if the point is close to an occupied cell in the occupancy grid
            close_to_wall = False
            for dx in range(-6, 8):  # Modified range for x-axis (-2 to 3)
                for dy in range(-6, 8):  # Modified range for y-axis (-2 to 3)
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

    def cluster_frontier_points(self, frontier_points, epsilon, min_samples):
        # Extract numerical coordinates from PointStamped objects
        points = [[point.point.x, point.point.y] for point in frontier_points]

        # Convert points to a NumPy array
        points = np.array(points)

        # Perform DBSCAN clustering
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        labels = dbscan.fit_predict(points)

        # Collect points for each cluster
        clusters = {}
        for i, label in enumerate(labels):
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(points[i])

        # Return cluster points
        return list(clusters.values())
    #----------------------------------------------------------------------------------------------------------------
    def calculate_cluster_sizes(self, cluster_points):
        table = []

        for points in cluster_points:
            first_point = points[0]
            last_point = points[-1]

            # Calculate the size between the first and last points
            size = math.sqrt((last_point[0] - first_point[0]) ** 2 + (last_point[1] - first_point[1]) ** 2)

            # Determine the middle point or the point next to the one in the middle
            middle_index = len(points) // 2
            if len(points) % 2 == 0:
                middle_point = points[middle_index - 1]
            else:
                middle_point = points[middle_index]

            # Add the size and middle point to the result list
            x=middle_point[0]
            y=middle_point[1]
            middle_point = PointStamped()
            middle_point.point.x = x
            middle_point.point.y = y
            middle_point.point.z = 0.0

            table.append((size, middle_point))

        return table
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
        goal_msg.goal.target_pose.pose.position.x = point.point.x
        goal_msg.goal.target_pose.pose.position.y = point.point.y
        goal_msg.goal.target_pose.pose.orientation.w = 1.0
        self.move_base_pub_0.publish(goal_msg)

    def send_goal_1(self, point):
        goal_msg = MoveBaseActionGoal()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.goal.target_pose.header.frame_id = "map_merged"
        goal_msg.goal.target_pose.pose.position.x = point.point.x
        goal_msg.goal.target_pose.pose.position.y = point.point.y
        goal_msg.goal.target_pose.pose.orientation.w = 1.0
        self.move_base_pub_1.publish(goal_msg)

    def send_goal_2(self, point):
        goal_msg = MoveBaseActionGoal()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.goal.target_pose.header.frame_id = "map_merged"
        goal_msg.goal.target_pose.pose.position.x = point.point.x
        goal_msg.goal.target_pose.pose.position.y = point.point.y
        goal_msg.goal.target_pose.pose.orientation.w = 1.0
        self.move_base_pub_2.publish(goal_msg)

    def tf_callback_robot_0(self, tf_msg):
        try:
            # Lookup the transform between the map frame and the robot's base frame
            transform = self.tf_buffer.lookup_transform("map_merged", "robot_0/base_link", rospy.Time())

            # Extract the position from the transform
            self.robot_0_pose = transform.transform.translation

            # Print the robot's pose
            #rospy.loginfo("Robot Pose: x={}, y={}".format(self.robot_pose.x, self.robot_pose.y))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to retrieve transform for robot_0")

    def tf_callback_robot_1(self, tf_msg):
        try:
            # Lookup the transform between the map frame and the robot's base frame
            transform = self.tf_buffer.lookup_transform("map_merged", "robot_1/base_link", rospy.Time())

            # Extract the position from the transform
            self.robot_1_pose = transform.transform.translation

            # Print the robot's pose
            #rospy.loginfo("Robot Pose: x={}, y={}".format(self.robot_pose.x, self.robot_pose.y))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to retrieve transform for robot_1")

    def tf_callback_robot_2(self, tf_msg):
        try:
            # Lookup the transform between the map frame and the robot's base frame
            transform = self.tf_buffer.lookup_transform("map_merged", "robot_2/base_link", rospy.Time())

            # Extract the position from the transform
            self.robot_2_pose = transform.transform.translation

            # Print the robot's pose
            #rospy.loginfo("Robot Pose: x={}, y={}".format(self.robot_pose.x, self.robot_pose.y))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to retrieve transform for robot_2")
    
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