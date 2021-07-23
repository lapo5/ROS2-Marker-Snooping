#!/usr/bin/env python3

# Libraries
import cv2
from cv2 import aruco
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import threading
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from allied_vision_camera_interfaces.msg import Pose
from allied_vision_camera_interfaces.srv import CameraState
from functools import partial
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Header
import tf2_ros
import geometry_msgs

from flir_ptu_d46_interfaces.srv import SetPanTiltSpeed, SetPanTilt, GetLimits


class MarkerSnooper(Node):
	def __init__(self):
		super().__init__("marker_snooper")
		self.get_logger().info("Marker Snooper node is awake...")


		# Subscription
		self.frame_sub = self.create_subscription(PoseStamped, "/target_tracking/ptu_to_marker_pose", self.callback_frame, 10)

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
		self.req_ptu_pos.pan = 0.0
		self.req_ptu_pos.tilt = 0.0
		self.go = True
		self.send_request_ptu_pos()


		self.client_ptu_limits = self.create_client(GetLimits, '/PTU/get_limits')
		while not self.client_ptu_limits.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service GetLimits not available, waiting again...')

		self.req_ptu_get_limits = GetLimits.Request()

		self.send_request_ptu_limits()

		self.discretization = 10

		self.discretization = self.discretization * math.pi / 180.0

		self.go = True

		self.get_logger().info('Marker Snooper Ready...')

		self.ptu_move_timer = self.create_timer(0.03, self.look_for_marker)



	def send_request_ptu_limits(self):
		self.future_2 = self.client_ptu_limits.call_async(self.req_ptu_get_limits)
		self.future_2.add_done_callback(partial(self.callback_ptu_get_limits))

	# This function is a callback to the client future
	def callback_ptu_get_limits(self, future):
		try:
			response = future.result()
			self.pan_min = response.pan_min
			self.pan_max = response.pan_max
			self.tilt_min = response.tilt_min
			self.tilt_max = response.tilt_max

		except Exception as e:
			self.get_logger().info("Service call failed %r" %(e,))


	def look_for_marker(self):
		pass

	def send_request_ptu_speed(self):
		self.future = self.client_ptu_speed.call_async(self.req_ptu_speed)


	def send_request_ptu_pos(self):
		if self.go:
			self.go = False
			self.future = self.client_ptu.call_async(self.req_ptu_pos)
			self.future.add_done_callback(partial(self.callback_return_service))

	# This function is a callback to the client future
	def callback_return_service(self, future):
		try:
			response = future.result()
			self.go = True
		except Exception as e:
			self.get_logger().info("Service call failed %r" %(e,))

	def clean_exit(self):
		self.req_ptu_pos.pan = 0.0
		self.req_ptu_pos.tilt = 0.0
		self.go = True
		self.send_request_ptu_pos()


	# This function store the received frame in a class attribute
	def callback_frame(self, msg):


		self.get_logger().info('Marker in sight!')


	def move_ptu(self, pan, tilt):
		print("discretization: {0}".format(self.discretization))
		print("Wanted pan: {0}".format(pan))
		print("Wanted tilt: {0}".format(tilt))

		pan_discret = int(pan / self.discretization)
		if math.abs(pan % self.discretization) > self.discretization / 2.0:
			if pan < 0:
				pan_discret = pan_discret - 1
			else:
				pan_discret = pan_discret + 1

		self.req_ptu_pos.pan = float(pan_discret * self.discretization)


		tilt_discret = int(tilt / self.discretization)
		if math.abs(tilt % self.discretization) > self.discretization / 2.0:
			if tilt < 0:
				tilt_discret = tilt_discret - 1
			else:
				tilt_discret = tilt_discret + 1

		self.req_ptu_pos.tilt = float(tilt_discret * self.discretization)

		print("Result pan: {0}".format(self.req_ptu_pos.pan))
		print("Result tilt: {0}".format(self.req_ptu_pos.tilt))

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