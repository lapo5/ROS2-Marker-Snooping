#!/usr/bin/env python3

# Libraries
import sys
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
import rclpy.time
from rclpy.action import ActionClient, ActionServer

from functools import partial
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Header
from std_srvs.srv import Empty, Trigger

from marker_snooping_interfaces.action import Snooping
from marker_snooping_interfaces.srv import SetTiltStatic
from flir_ptu_d46_interfaces.srv import SetPanTiltSpeed, GetLimits
from flir_ptu_d46_interfaces.action import SetPanTilt


class MarkerSnooper(Node):
    def __init__(self):
        super().__init__("marker_snooping")
        self.get_logger().info("Marker Snooping node is awake...")

        self.declare_parameter("tilt_static", "0.2")
        self.tilt_static = float(self.get_parameter("tilt_static").value)

        self.declare_parameter("discretization", "10")
        self.discretization = int(self.get_parameter("discretization").value)

        if self.discretization <= 0:
            self.get_logger().info("discretization must be integer > 0")
            return None

        self.declare_parameter("time_to_sleep", "3.0")
        self.time_to_sleep = float(self.get_parameter("time_to_sleep").value)

        self.step_snooping = 0.0
        self.current_pan = 0.0

        self.current_step = 0

        self.operating = False
        self.sleep_timer = None

        self.ptu_arrived = False
        self.started = False

        self.declare_parameter("subscribers.marker_in_sight", "/target_tracking/ptu_to_marker_transform")
        self.marker_in_sight_topic_name = self.get_parameter("subscribers.marker_in_sight").value

        self.declare_parameter("services.set_tilt_static", "/marker_snooping/set_tilt_static")
        self.set_tilt_static_service_name = self.get_parameter("services.set_tilt_static").value
        self.set_tilt_static_srv = self.create_service(SetTiltStatic, self.set_tilt_static_service_name, self.set_tilt_static)

        self.declare_parameter("services_client.set_pan_tilt", "/ptu/set_pan_tilt")
        self.set_pan_tilt_service = self.get_parameter("services_client.set_pan_tilt").value

        self.declare_parameter("services_client.set_pan_tilt_speed", "/ptu/set_pan_tilt_speed")
        self.set_pan_tilt_speed_service = self.get_parameter("services_client.set_pan_tilt_speed").value

        self.declare_parameter("services_client.get_limits", "/ptu/get_limits")
        self.ptu_get_limits_service = self.get_parameter("services_client.get_limits").value

        self.declare_parameter("actions.start", "/marker_snooping/start_action")
        self.start_action = self.get_parameter("actions.start").value

        # Subscription
        self.marker_sub = self.create_subscription(TransformStamped, self.marker_in_sight_topic_name, self.callback_marker, 1)

        # Clients
        self.action_client_ptu = ActionClient(self, SetPanTilt, self.set_pan_tilt_service)
        while not self.action_client_ptu.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action SetPanTilt not available, waiting again...')

        self.req_ptu_pos = SetPanTilt.Goal()

        self.client_ptu_speed = self.create_client(SetPanTiltSpeed, self.set_pan_tilt_speed_service)
        while not self.client_ptu_speed.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service SetPanTiltSpeed not available, waiting again...')

        self.req_ptu_speed = SetPanTiltSpeed.Request()
        self.req_ptu_speed.pan_speed = 0.5
        self.req_ptu_speed.tilt_speed = 0.2

        self.send_request_ptu_speed()

        self.client_ptu_limits = self.create_client(GetLimits, self.ptu_get_limits_service)
        while not self.client_ptu_limits.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service GetLimits not available, waiting again...')

        self.req_ptu_get_limits = GetLimits.Request()

        self.send_request_ptu_limits()



    def set_tilt_static(self, request, response):
        self.get_logger().info('[Marker Snooping] Set Tilt Static')
 
        self.tilt_static = request.tilt_static
        self.move_ptu(self.current_pan, self.tilt_static)

        response.res = True

        return response


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

            self.step_snooping = float(self.pan_max - self.pan_min) / self.discretization

            self.start_snooping_action = ActionServer(
                self,
                Snooping,
                self.start_action,
                self.execute_action_start_callback)
            self.executing_action = False
            
            self.get_logger().info('[Marker Snooping] Ready')

        except Exception as e:
            self.get_logger().info("Service call failed %r" %(e,))


    def execute_action_start_callback(self, goal_handle):
        self.get_logger().info('[Marker Snooping] Executing goal...')

        self.current_step = 1
        self.ptu_arrived = False
        self.operating = True
        self.marker_in_sight = False
        self.executing_action = True
        self.current_pan = self.pan_min

        self.feedback_msg = Snooping.Feedback()
        self.goal_handle = goal_handle

        self.restart_snoop()

        goal_handle.succeed()

        result = Snooping.Result()
        result.ret = self.marker_in_sight
        self.executing_action = False

        return result


    def send_request_ptu_speed(self):
        self.client_ptu_speed.call_async(self.req_ptu_speed)



    # This function is a callback to the client future
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            response = future.result().result
            
            self.feedback_msg.percentage_of_completing = int(self.current_step  * 100.0 / float(self.discretization + 1))
            self.goal_handle.publish_feedback(self.feedback_msg)

            if self.operating:
                self.sleep_timer = self.create_timer((self.time_to_sleep), self.move_ptu_completed)

        except Exception as e:
            self.get_logger().info("Service call failed %r" %(e,))

    def move_ptu_completed(self):
        self.ptu_arrived = True

    def restart_snoop(self):
        self.started = False
        
        if self.executing_action:
            self.feedback_msg.percentage_of_completing = 0
            self.goal_handle.publish_feedback(self.feedback_msg)

        while self.operating:
            try:
                if self.sleep_timer is not None:
                    self.sleep_timer.cancel()

                if self.current_pan > self.pan_max:
                    self.get_logger().info('[Marker Snooping] Marker not found!')
                    self.operating = False
                else:
                    self.move_ptu(self.current_pan, self.tilt_static)
                    self.started = True

                    self.current_pan = self.current_pan + self.step_snooping


                    self.current_step = self.current_step + 1

            except Exception as e:
                pass
                    


    # This function store the received frame in a class attribute
    def callback_marker(self, msg):
        if self.operating and self.started and not self.marker_in_sight:   
            self.marker_in_sight = True
            self.get_logger().info('[Marker Snooping] Marker in sight!')
            self.operating = False


    def move_ptu(self, pan, tilt):
        self.ptu_arrived = False
        self.req_ptu_pos.pan = float(pan)
        self.req_ptu_pos.tilt = float(tilt)

        self.future_ptu_pos = self.action_client_ptu.send_goal_async(self.req_ptu_pos)
        self.future_ptu_pos.add_done_callback(self.goal_response_callback)
        while not self.ptu_arrived:
            rclpy.spin_once(self)


# Main loop function
def main(args=None):
    rclpy.init(args=args)
    node = MarkerSnooper()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node)

    except KeyboardInterrupt:
        print("[Marker Snooping] Node stopped clearly")
    except BaseException:
        print('[Marker Snooping] Exception:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        rclpy.shutdown() 


# Main
if __name__ == '__main__':
    main()