#!/usr/bin/env python
import rospy
from math import floor
from nav_msgs.msg import OccupancyGrid
from message_filters import Subscriber, ApproximateTimeSynchronizer, TimeSynchronizer

import config

class MapCombiner:
    def __init__(self, map1, map2, map_explore):
        self.map1 = map1
        self.map2 = map2
        self.pub = rospy.Publisher(map_explore, CostandGrid, queue_size=10)

    def run(self):
        sync = ApproximateTimeSynchronizer([ Subscriber(self.map1, OccupancyGrid)
                                           , Subscriber(self.map2, OccupancyGrid) ], 10, 1)
        sync.registerCallback(self.sync_callback)
        rospy.spin()

    def sync_callback(self, grid1, grid2):
        self.pub.publish(grid1)

def main():
    rospy.init_node(config.map_combiner['name'], anonymous=True)
    MapCombiner(config.move_base['pub']['cost_map']['costmap'], config.rtabmap['pub']['proj_map'], config.map_combiner['pub']['map_explore']).run()

if __name__ == '__main__':
    main()
