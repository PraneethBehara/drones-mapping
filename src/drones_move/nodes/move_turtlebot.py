#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from drones_move.srv import *
from move_commands import MoveCommands as MC

class TurtlebotMover:
    def __init__(self):
        rospy.init_node('TurtlebotMover', anonymous=True)
        self.pub_cmdvel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.pub_movebase = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(10)

        self.cached_odom = None
        self.state = 'still'
        self.rotate_start_pose = None

        self.command = MC.STOP
        self.lvel = [0, 0, 0]
        self.avel = [0, 0, 0]
        
    def run(self):
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Service('move_turtlebot', MoveTurtlebot, self.handle_move_turtlebot)
        self.keep_moving(self.lvel, self.avel)

    def handle_move_turtlebot(self, request):
        if request.command == MC.THREE60:
            self.command = MC.THREE60
            self.state = 'rotate'
        elif request.command == MC.MOVETO:
            self.log(request)
            self.command = MC.MOVETO
            self.publish_long(self.pub_movebase, self.get_pose_stamped(request.pose))

        return MoveTurtlebotResponse('ok')

    def odom_callback(self, data):
        self.cached_odom = data

        if self.state in ['rotate', 'rotate_kicked', 'rotate_started']:
            if self.state == 'rotate':
                self.rotate_start_pose = data.pose.pose
                self.state = 'rotate_kicked'
            elif self.state == 'rotate_kicked':
                if self.is_rotation_started(data.pose.pose, self.rotate_start_pose):
                    self.state = 'rotate_started'
                else:
                    self.avel[2] = 0.5
            elif self.state == 'rotate_started':
                if self.is_rotation_complete(data.pose.pose, self.rotate_start_pose):
                    self.avel[2] = 0.0
                    self.state = 'still'
                    self.comand = MC.MOVETO

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

    def keep_moving(self, linear, angular):
        while not rospy.is_shutdown():
            if self.command == MC.THREE60:
                self.publish(self.pub_cmdvel, self.make_twist(linear=linear, angular=angular))

    def publish(self, publisher, message):
        publisher.publish(message)
        self.rate.sleep()

    def publish_long(self, publisher, message):
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
