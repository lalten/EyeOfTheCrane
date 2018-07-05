#! /usr/bin/python

import serial
import rospy
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from math import pi, sin, cos, isnan, isinf, sqrt, atan
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy
from scipy.stats import linregress
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Int8
from jsk_rviz_plugins.msg import OverlayText
from serial import Serial
import socket
import time


class CraneAligner:
    def __init__(self):
        self._get_new_scan = False
        
        # we use the middle 15deg of the scan as input for line fitting
        self._window_size = 15/180.0*pi 

        self._pub_pts = rospy.Publisher("/pts",Marker, queue_size=1)
        self._pub_angle = rospy.Publisher("/wall_angle", Float32, queue_size=1)
        self._pub_max_dist = rospy.Publisher("/wall_max_dist", Float32, queue_size=1)
        self._pub_wall_range = rospy.Publisher("/wall_range", Float32, queue_size=1)

        self._pub_basket_command = rospy.Publisher("/basket_command", Float32, queue_size=1)
        self._pub_range_command = rospy.Publisher("/range_command", Float32, queue_size=1)


        rospy.sleep(0.5)
        self._sub_scan = rospy.Subscriber('/scan', LaserScan, self._scan_cb, queue_size=1)
        self._center_points = list()

        self._sub_angle = rospy.Subscriber("/compass_yaw", Float32, self._angle_cb, queue_size=1)

        self._min_dist = 0.3  # minimal range for laser
        self.iteration_cnt = 0


        # parameters for bang-bang controllers:
        self._current_basket_command = 0
        self._basket_dead_angle = 2  ## deadzone (no command sent to crane)
        self._current_basket_angle_deg = 0

        self._range_command = 0
        self._range_dead_zone = 0.1  ## DEAD
        self._range_goal = 0.5
        self._current_range = 0

        self.yaw_command = 0
        self._yaw_dead_zone_deg = 10
        self._yaw_goal = 0
        self._current_yaw = 0

        # TCP-connection to second laptop that passes the command to the arduino that
        # then controls the crane
        TCP_IP = '192.168.43.3'
        TCP_PORT = 5005
        BUFFER_SIZE = 1024
        MESSAGE = "Hello, World!"

        self.tcp_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_s.connect((TCP_IP, TCP_PORT))

    def update_commands(self, ):
        # Creating of digital input commands:
        # if value is close to goal, don't move axis, otherwise move in corresponding direction
        
        
        new_goal = rospy.get_param("/yaw_goal", 0)
        if abs(new_goal-self._yaw_goal) > 1e-2:
            rospy.logwarn("Updated yaw goal to %f", new_goal)
            self._yaw_goal = new_goal

        # Basket: control to zero
        if abs(self._current_basket_angle_deg) < self._basket_dead_angle:
            cmd = 0
        else:
            cmd = -1 if self._current_basket_angle_deg < 0 else 1
        self._current_basket_command = cmd

        self._pub_basket_command.publish(self._current_basket_command)

        # compute yaw command: control to _current_yaw
        # if abs(self._yaw_goal-self._current_yaw) < self._yaw_dead_zone_deg:
        #     self.yaw_command = 199
        # else:
        #     self.yaw_command = 100 if self._current_yaw > 0 else 255
        self.yaw_command = rospy.get_param("/yaw_cmd")

        # Range
        if abs(self._range_goal - self._current_range) < self._range_dead_zone:
            self._range_command = 199
        else:
            self._range_command = 100 if self._current_range < self._range_goal else 255

        self._pub_range_command.publish(self._range_command)

    def move(self):
        self.iteration_cnt += 1 
        if self.iteration_cnt % 20:
            return

        # we didn't control elevation and tilt of basket
        cmd = "199,199,%03i,%03i,%i\n" % (self._range_command, self.yaw_command, self._current_basket_command)
        self.tcp_s.send(cmd)
        rospy.loginfo(cmd)

    def _angle_cb(self, msg):
        assert isinstance(msg, Float32)
        # rospy.loginfo("Got Angle from compass: %f", msg.data)
        self._current_yaw = msg.data

    def _scan_cb(self, msg):
        assert isinstance(msg, LaserScan)

        m = Marker()
        m.type = m.SPHERE_LIST
        m.ns = 'filtered'
        m.header = msg.header
        m.color.a = 1
        m.color.r = 1
        m.action = m.ADD
        m.pose.orientation.w = 1
        m.scale.x = m.scale.y = m.scale.z = 0.05

        pnt_cnt = int(self._window_size/msg.angle_increment)

        len_pts = len(msg.ranges)
        self._center_points = list()
        for i in xrange(len_pts/2-pnt_cnt, len_pts/2+pnt_cnt, 1):
            angle = msg.angle_min+i*msg.angle_increment
            r = msg.ranges[i]
            if isnan(r) or isinf(r):
                continue

            if r < self._min_dist:
                continue

            # create cartesian coordinates from distances
            x = cos(angle) * r
            y = sin(angle) * r
            self._center_points.append((x, y))
            m.points.append(Point(x, y, 0))

        self._pub_pts.publish(m)
        self._get_new_scan = False
        self.get_angle_dist()

        self.update_commands()
        self.move()

    def get_angle_dist(self, time_event=None):
        # back to distances :)
        dists = map(lambda (x, y): sqrt(x*x+y*y), self._center_points)
        if not dists:
            return
        max_dist, min_dist = max(dists), min(dists)
        xl, yl = zip(*self._center_points)

        # fit polynom of degree 1 to points: (y = m*x+c)
        res = numpy.polyfit(yl, xl, deg=1, full=False)

        m = res[0]
        c = res[1]

        # minimal distance of origin to line fit
        self._current_range = c/(sqrt(m**2+1)) 
        
        self._pub_wall_range.publish(self._current_range)
        self._current_basket_angle_deg = atan(m)/pi*180

        self._pub_angle.publish(self._current_basket_angle_deg)


if __name__ == "__main__":
    rospy.init_node("Aligner")
    ca = CraneAligner()
    rospy.spin()
