#!/usr/bin/env python
import rospy
import tf

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from drones_move.srv import *
from move_commands import MoveCommands as MC

import config

class Velocity:

    def __init__(self, linear=[0, 0, 0], angular=[0, 0, 0]):
        self.linear = linear
        self.angular = angular

class ReadyState:
    READY = 'READY'
    BUSY = 'BUSY'

class BotState:
    STATIONARY = 'STATIONARY'
    RPREPARING = 'RPREPARING'
    RSTARTING  = 'RSTARTING'
    ROTATING   = 'ROTATING'
    MOVING     = 'MOVING'

class BotCache:

    def __init__(self):
        self.odom = None
        self.move = None
        self.rstart_pose = None
        self.state = BotState.STATIONARY
        self.velocity = Velocity()
        self.command = MC.STOP

class Publishers:

    def __init__(self, cmdvel=None, movegoal=None, readystate=None):
        self.cmdvel = cmdvel
        self.movegoal = movegoal
        self.readystate = readystate

class Subscribers:

    def __init__(self, subscriptions=[]):
        for s in subscriptions:
            rospy.Subscriber(s[0], s[1], s[2])

class Services:

    def __init__(self, services):
        for s in services:
            rospy.Service(s[0], s[1], s[2])

class TurtlebotMover:
    
    def __init__(self):
        rospy.init_node(config.bot_mover['name'], anonymous=True)
        self.rate = rospy.Rate(10)
        self.cache = BotCache()
        self.pub = None

    def run(self):
        Subscribers(subscriptions = [ (config.turtlebot['pub']['odom'], Odometry, self.odom_callback )
                                    , (config.move_base['pub']['status'], GoalStatusArray, self.goal_status_callback ) ])
        Services(services = [ (config.bot_mover['srv']['move_bot'], MoveTurtlebot, self.handle_move) ])
        self.pub = Publishers( cmdvel     = rospy.Publisher(config.bot_mover['pub']['cmd_vel'], Twist, queue_size=10)
                             , movegoal   = rospy.Publisher(config.bot_mover['pub']['move_goal'], PoseStamped, queue_size=10) 
                             , readystate = rospy.Publisher(config.bot_mover['pub']['ready_state'], String, queue_size=10) )
        self.loop()

    def goal_status_callback(self, data):
        if self.cache.command == MC.MOVETO and self.is_movement_complete(data):
            self.log('I: {0} | {1}'.format(self.cache.state, self.cache.command))
            self.complete_movement()
        else:
            self.log('E: {0} | {1}'.format(self.cache.command, self.is_movement_complete(data)))

    def is_movement_complete(self, data):
        result = False

        if len(data.status_list) == 1:
            if data.status_list[0].status in [2, 3, 4, 5, 8]:
                result = True

        return result

    def complete_movement(self):
        self.cache.command = MC.STOP
        self.cache.state = BotState.STATIONARY

    def handle_move(self, request):
        if request.command == MC.THREE60:
            self.cache.command = MC.THREE60
            self.cache.state = BotState.RPREPARING
        elif request.command == MC.MOVETO:
            self.cache.command = MC.MOVETO
            self.cache.state = BotState.MOVING
            self.publish10(self.pub.movegoal, self.get_pose_stamped(request.pose))
        return MoveTurtlebotResponse('ok')

    def odom_callback(self, data):
        self.cache.odom = data

        if self.cache.state in [BotState.RPREPARING, BotState.RSTARTING, BotState.ROTATING]:
            if self.cache.state == BotState.RPREPARING:
                self.prepare_rotation(data)
            elif self.cache.state == BotState.RSTARTING:
                self.begin_rotation(data)
            elif self.cache.state == BotState.ROTATING:
                self.complete_rotation(data)

    def prepare_rotation(self, data):
        self.cache.rstart_pose = data.pose.pose
        self.cache.state = BotState.RSTARTING

    def begin_rotation(self, data):
        if self.is_rotation_started(data.pose.pose, self.cache.rstart_pose):
            self.cache.state = BotState.ROTATING
        else:
            self.cache.velocity.angular[2] = 0.5

    def complete_rotation(self, data):
        if self.is_rotation_complete(data.pose.pose, self.cache.rstart_pose):
            self.cache.velocity.angular[2] = 0.0
            self.cache.state = BotState.STATIONARY
            self.cache.command = MC.STOP

    def is_rotation_started(self, current, target):
        current_e = tf.transformations.euler_from_quaternion(( current.orientation.x
                                                             , current.orientation.y
                                                             , current.orientation.z
                                                             , current.orientation.w ))
        target_e = tf.transformations.euler_from_quaternion(( target.orientation.x
                                                            , target.orientation.y
                                                            , target.orientation.z
                                                            , target.orientation.w ))
        return False if self.are_angles_almost_equal(current_e[2], target_e[2]) else True

    def is_rotation_complete(self, current, target):
        current_e = tf.transformations.euler_from_quaternion(( current.orientation.x
                                                             , current.orientation.y
                                                             , current.orientation.z
                                                             , current.orientation.w ))
        target_e = tf.transformations.euler_from_quaternion(( target.orientation.x
                                                            , target.orientation.y
                                                            , target.orientation.z
                                                            , target.orientation.w ))
        return True if self.are_angles_almost_equal(current_e[2], target_e[2]) else False

    def are_angles_almost_equal(self, one, two):
        return True if abs(two - one) < 0.005 else False

    def loop(self):
        while not rospy.is_shutdown():
            if self.cache.command == MC.THREE60:
                self.publish( [ (self.pub.cmdvel
                                , self.make_twist( linear  = self.cache.velocity.linear
                                                 , angular = self.cache.velocity.angular )) ] )
            self.publish( [ (self.pub.readystate, self.get_ready_state()) ] )
            self.rate.sleep()

    def get_ready_state(self):
        return ReadyState.READY if self.cache.state == BotState.STATIONARY else ReadyState.BUSY

    def publish(self, actions):
        for a in actions:
            a[0].publish(a[1])

    def publish10(self, publisher, message):
        for i in range(0, 10):
            publisher.publish(message)
            self.rate.sleep()

    def make_twist(self, linear=[0, 0, 0], angular=[0, 0, 0]):
        t = Twist()
        t.linear.x = linear[0]
        t.linear.y = linear[1]
        t.linear.z = linear[2]

        t.angular.x = angular[0]
        t.angular.y = angular[1]
        t.angular.z = angular[2]

        return t

    def log(self, message):
        rospy.loginfo(message)

    def get_pose_stamped(self, pose):
            p = PoseStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = 'map'
            p.pose = pose
            return p

def main():
    TurtlebotMover().run()

if __name__ == '__main__':
    main()
