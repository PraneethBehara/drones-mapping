#!/usr/bin/env python
import rospy
from math import floor
from nav_msgs.msg import OccupancyGrid
from drones_move.msg import CostandGrid
from message_filters import Subscriber, ApproximateTimeSynchronizer, TimeSynchronizer

import config

class MapCombiner:
    def __init__(self, map1, map2, map_explore):
        self.map1 = map1
        self.map2 = map2
        self.pub = rospy.Publisher(map_explore, OccupancyGrid, queue_size=10)

    def run(self):
        sync = ApproximateTimeSynchronizer([ Subscriber(self.map1, OccupancyGrid)
                                           , Subscriber(self.map2, OccupancyGrid) ], 10, 1)
        sync.registerCallback(self.sync_callback)
        rospy.spin()

    def sync_callback(self, grid1, grid2):
        # msg = CostandGrid()
        # msg.header = grid2.header
        # msg.info = grid2.info
        # msg.data = grid2.data
        # msg.cost_data = grid1.data
        self.pub.publish(grid2)

def main():
    rospy.init_node(config.map_combiner['name'], anonymous=True)
    MapCombiner(config.rtabmap['pub']['grid_map'], config.rtabmap['pub']['proj_map'], config.map_combiner['pub']['map_explore']).run()

if __name__ == '__main__':
    main()