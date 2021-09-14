#!/usr/bin/env python3

# Libraries
import cv2
from cv2 import aruco
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import threading
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from allied_vision_camera_interfaces.msg import Pose
from allied_vision_camera_interfaces.srv import CameraState
from functools import partial
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from std_srvs.srv import Empty
import rclpy.time
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import json
import numpy as np
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Header
import tf2_ros
import geometry_msgs
import math
import time
import sys

from flir_ptu_d46_interfaces.srv import SetPanTiltSpeed, SetPanTilt, GetLimits

from robot_localization.srv import SetPose

class MarkerSnooper(Node):
    def __init__(self):
        super().__init__("marker_snooper")
        self.get_logger().info("Marker Snooper node is awake...")

        self.tilt_static = 0.2

        self.step_snooping = 0.0
        self.current_pan = 0.0

        self.discretization = 12
        self.time_to_sleep = 4

        # Service: stop acquisition
        self.stop_service = self.create_service(Empty, "/marker_snooping/start", self.start_snooping)

        # Clients
        self.client_ptu_speed = self.create_client(SetPanTiltSpeed, '/PTU/set_pan_tilt_speed')
        while not self.client_ptu_speed.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service SetPanTiltSpeed not available, waiting again...')

        self.client_ptu = self.create_client(SetPanTilt, '/PTU/set_pan_tilt')
        while not self.client_ptu.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service SetPanTilt not available, waiting again...')

        self.req_ptu_speed = SetPanTiltSpeed.Request()
        self.req_ptu_speed.pan_speed = 0.5
        self.req_ptu_speed.tilt_speed = 0.2

        self.send_request_ptu_speed()

        self.req_ptu_pos = SetPanTilt.Request()

        self.client_ptu_limits = self.create_client(GetLimits, '/PTU/get_limits')
        while not self.client_ptu_limits.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service GetLimits not available, waiting again...')

        self.req_ptu_get_limits = GetLimits.Request()

        self.send_request_ptu_limits()



    def send_request_ptu_limits(self):
        self.future = self.client_ptu_limits.call_async(self.req_ptu_get_limits)
        self.future.add_done_callback(partial(self.callback_ptu_get_limits))

    # This function is a callback to the client future
    def callback_ptu_get_limits(self, future):
        try:
            response = future.result()
            self.pan_min = response.pan_min
            self.pan_max = response.pan_max
            self.tilt_min = response.tilt_min
            self.tilt_max = response.tilt_max

            print("Max pan: {0}".format(self.pan_max))
            print("Min pan: {0}".format(self.pan_min))

            self.step_snooping = float(self.pan_max - self.pan_min) / self.discretization
            print("Step Pan Snoop: {0}".format(self.step_snooping))

        except Exception as e:
            self.get_logger().info("Service call failed %r" %(e,))

    def start(self):
        self.current_pan = self.pan_min

        self.move_ptu(self.current_pan, self.tilt_static)
        time.sleep(6)

        self.get_logger().info('Marker Snooper Ready...')

        self.look_for_marker()

        # Subscription
        self.marker_sub = self.create_subscription(PoseStamped, "/target_tracking/ptu_to_marker_pose", self.callback_marker, 1)



    # This function stops/enable the acquisition stream
    def start_snooping(self, request, response):
 
        self.marker_in_sight = False
        self.start()

        return response



    def look_for_marker(self):
        if not self.marker_in_sight:
            print("current_pan: {0}".format(self.current_pan))

            self.move_ptu(self.current_pan, self.tilt_static)


    def send_request_ptu_speed(self):
        self.client_ptu_speed.call_async(self.req_ptu_speed)


    def send_request_ptu_pos(self):
        self.future_ptu_pos = self.client_ptu.call_async(self.req_ptu_pos)
        self.future_ptu_pos.add_done_callback(partial(self.callback_return_service))


    # This function is a callback to the client future
    def callback_return_service(self, future):
        try:
            response = future.result()
            
            if not self.marker_in_sight:
                self.sleep_timer = self.create_timer((self.time_to_sleep), self.restart_snoop)

        except Exception as e:
            self.get_logger().info("Service call failed %r" %(e,))

    def restart_snoop(self):
        try:
            self.sleep_timer.destroy()

            self.current_pan = self.current_pan + self.step_snooping

            if self.current_pan > self.pan_max:
                print("Marker not found!")
                self.marker_in_sight = False
            else:
                self.look_for_marker()

        except Exception as e:
            pass



    # This function store the received frame in a class attribute
    def callback_marker(self, msg):

        self.get_logger().info('Marker in sight!')



    def send_request_set_pose(self):
        self.future = self.client_set_pose.call_async(self.req_set_pose)
        self.future.add_done_callback(partial(self.callback_request_set_pose))

    # This function is a callback to the client future
    def callback_request_set_pose(self, future):
        try:
            response = future.result()

            self.get_logger().info('Set Pose Results: {0}'.format(response))


        except Exception as e:
            self.get_logger().info("Service call failed %r" %(e,))



    def move_ptu(self, pan, tilt):
        self.req_ptu_pos.pan = float(pan)

        self.req_ptu_pos.tilt = float(tilt)

        self.send_request_ptu_pos()


# Main loop function
def main(args=None):
    rclpy.init(args=args)
    node = MarkerSnooper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Marker Snooper Node stopped clearly")
    except BaseException:
        print('Exception in Marker Snooper Node:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        node.destroy_node()
        rclpy.shutdown() 


# Main
if __name__ == '__main__':
    main()