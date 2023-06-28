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

        # ROS node initialization
        rospy.init_node('multi_frontier_exploration')

        self.frontier_points = []  # List to store frontier points
        self.frontier_robot = []  # List to store frontier points for each robot
        self.robot_pose = None  # Robot position
        self.robot_namespace = rospy.get_param('~robot_namespace')

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to TF topic for each robot
        self.tf_sub_robot = rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback_robot)

        # Subscribe to the occupancy grid map
        rospy.Subscriber('/map_merge', OccupancyGrid, self.map_callback)

        # Publisher for frontier points as markers
        self.marker_publisher = rospy.Publisher('cluster_markers', Marker, queue_size=10)

        # Publisher for MoveBaseGoal messages
        self.move_base_pub = rospy.Publisher('/' + self.robot_namespace + '/move_base/goal', MoveBaseActionGoal, queue_size=10)

    def map_callback(self, msg):
        # Perform frontier extraction and delete points close to walls on the received map
        self.extract_frontiers(msg)

        # Transform frontier points into clusters
        epsilon = 0.3
        min_samples = 5
        cluster_points = self.cluster_frontier_points(self.frontier_points, epsilon, min_samples)

        # Remove wrong clusters
        cluster_points = [points for points in cluster_points if
                          math.sqrt((points[-1][0] - points[0][0]) ** 2 + (points[-1][1] - points[0][1]) ** 2) <= 4.5]

        # Visualize the clusters as markers
        self.visualize_clusters(cluster_points)

        # Calculate gain information for each frontier cluster
        self.calculate_gain_info(cluster_points)

        # Select the best frontier cluster for each robot
        self.select_best_frontier()

        # Send MoveBaseGoal messages to the selected frontier clusters
        self.send_move_base_goals()

    def extract_frontiers(self, map_msg):
        # Convert the occupancy grid data to a numpy array
        map_data = np.array(map_msg.data)

        # Get the map resolution
        resolution = map_msg.info.resolution

        # Get the map width and height
        width = map_msg.info.width
        height = map_msg.info.height

        # Reshape the data to match the map dimensions
        map_data = np.reshape(map_data, (height, width))

        # Get the robot position in the map frame
        robot_x = self.robot_pose.pose.position.x
        robot_y = self.robot_pose.pose.position.y

        # Calculate the map indices corresponding to the robot position
        robot_map_x = int((robot_x - map_msg.info.origin.position.x) / resolution)
        robot_map_y = int((robot_y - map_msg.info.origin.position.y) / resolution)

        # Set the robot position as an obstacle
        map_data[robot_map_y, robot_map_x] = 100

        # Set the unknown cells as obstacles
        unknown_indices = np.where(map_data == -1)
        map_data[unknown_indices] = 100

        # Calculate the frontier cells using the Dijkstra algorithm
        frontier_cells = self.dijkstra_frontier_search(map_data, robot_map_x, robot_map_y)

        # Convert the frontier cells to frontier points
        self.frontier_points = self.cells_to_points(frontier_cells, map_msg.info.origin.position, resolution)

        # Filter frontier points close to walls
        filtered_points = filter_frontier_points(self.frontier_points, 0.3)
        self.frontier_points = filtered_points

    def dijkstra_frontier_search(self, map_data, start_x, start_y):
        # Define the movement costs for each direction
        move_costs = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])

        # Initialize the distance map with large values
        distance_map = np.full_like(map_data, 100, dtype=np.float32)

        # Set the distance of the start cell to 0
        distance_map[start_y, start_x] = 0

        # Create a list to store the frontier cells
        frontier_cells = []

        # Iterate until the frontier cells are found
        while True:
            # Find the cell with the minimum distance
            min_distance = np.min(distance_map)
            min_indices = np.where(distance_map == min_distance)
            current_x, current_y = min_indices[1][0], min_indices[0][0]

            # If the minimum distance is infinity, break the loop
            if min_distance == 100:
                break

            # Add the current cell to the frontier cells
            frontier_cells.append((current_x, current_y))

            # Update the distances of the neighboring cells
            for move_y in range(-1, 2):
                for move_x in range(-1, 2):
                    # Calculate the new cell indices
                    new_x = current_x + move_x
                    new_y = current_y + move_y

                    # Check if the new cell is within the map boundaries
                    if 0 <= new_x < map_data.shape[1] and 0 <= new_y < map_data.shape[0]:
                        # Calculate the cost of moving to the new cell
                        cost = move_costs[move_y + 1, move_x + 1]

                        # Calculate the new distance
                        new_distance = distance_map[current_y, current_x] + cost

                        # Update the distance if the new distance is smaller
                        if new_distance < distance_map[new_y, new_x]:
                            distance_map[new_y, new_x] = new_distance

            # Set the distance of the current cell to infinity
            distance_map[current_y, current_x] = 100

        return frontier_cells

    def cells_to_points(self, cells, map_origin, resolution):
        # Convert the map origin to meters
        origin_x = map_origin.position.x
        origin_y = map_origin.position.y

        # Convert the cells to points
        points = []
        for cell in cells:
            x = (cell[0] * resolution) + origin_x + (resolution / 2)
            y = (cell[1] * resolution) + origin_y + (resolution / 2)
            point = PointStamped()
            point.header.frame_id = 'map'
            point.point.x = x
            point.point.y = y
            point.point.z = 0
            points.append(point)

        return points

    def cluster_frontier_points(self, points, epsilon, min_samples):
        # Extract the x and y coordinates of the points
        xy = [(point.point.x, point.point.y) for point in points]

        # Perform DBSCAN clustering on the points
        dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        labels = dbscan.fit_predict(xy)

        # Create a list to store the clustered points
        cluster_points = []

        # Iterate over the unique labels
        unique_labels = np.unique(labels)
        for label in unique_labels:
            # Get the points with the current label
            cluster_indices = np.where(labels == label)[0]
            cluster = [points[i] for i in cluster_indices]

            # Add the cluster to the list
            cluster_points.append(cluster)

        return cluster_points

    def visualize_clusters(self, clusters):
        # Create a MarkerArray message
        marker_array = MarkerArray()

        # Iterate over each cluster
        for i, cluster in enumerate(clusters):
            # Create a Marker message for the current cluster
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = 'clusters'
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.color.a = 1.0

            # Set the cluster points as marker points
            marker.points = [point.point for point in cluster]

            # Add the marker to the marker array
            marker_array.markers.append(marker)

        # Publish the marker array
        self.marker_publisher.publish(marker_array)

    def calculate_gain_info(self, clusters):
        # TODO: Implement Rao Blackwellization for gain information calculation
        # You can use the frontier clusters and their corresponding points to calculate gain information
        # for each cluster. The calculation depends on the specific problem you are trying to solve.

        pass

    def select_best_frontier(self):
        # TODO: Implement the selection of the best frontier cluster for each robot
        # You can use the calculated gain information to determine the best frontier cluster for each robot.

        pass

    def send_move_base_goals(self):
        # TODO: Implement the sending of MoveBaseGoal messages to the selected frontier clusters
        # You can use the MoveBaseGoal messages to navigate the robots to their respective frontier clusters.

        pass

    def tf_callback_robot(self, tf_message):
        # Get the transform from the map frame to the robot frame
        try:
            transform = self.tf_buffer.lookup_transform('map', self.robot_namespace + '/base_link', rospy.Time())
            self.robot_pose = transform.transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass


if __name__ == '__main__':
    mfe = MultiFrontierExploration()

    # Spin the node
    rospy.spin()