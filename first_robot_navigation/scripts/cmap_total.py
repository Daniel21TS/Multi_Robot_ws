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
def callback(cmap_total: OccupancyGrid):

    data = list(cmap_total.data)
    for y in range(cmap_total.info.height):
        for x in range(cmap_total.info.width):
            i = x + (cmap_total.info.height - 1 - y) * cmap_total.info.width
            if data[i] >= M:  
                data[i] = data[i]
            elif (data[i] >= 0) and (data[i] < N):  # free
                data[i] = data[i]
            else:  # unknown
                data[i] = data[i]
    cmap_total.data = tuple(data)
    pub.publish(cmap_total)

if __name__ == '__main__':
    rospy.init_node('map_merge_to_cmap_total')
    pub = rospy.Publisher('cmap_total', OccupancyGrid, queue_size=10)
    rospy.Subscriber('/map_merge', OccupancyGrid, callback)
    rospy.spin()