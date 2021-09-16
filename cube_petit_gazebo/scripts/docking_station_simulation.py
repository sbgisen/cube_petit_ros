#!/usr/bin/env python

from math import pi, sqrt
import rospy
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np


class DockingStationSimulation:

    def __init__(self):
        rospy.logdebug('[DockingStationSimulation] Start Initialization')
        self.rate = rospy.Rate(20)
        # Get rosparam
        self.publish_debug_topics = rospy.get_param(
            '~publish_debug_topics', True)
        # Link names
        # Robot link name published by Gazebo plugin
        self.robot_link_name = rospy.get_param(
            '~robot_link_name', 'base')
        # Charging-station link name published by Gazebo plugin
        self.station_link_name = rospy.get_param(
            '~station_link_name', 'station')
        # Docking parameters
        # Distance from robot origin to the charging connector in x-axis
        # direction [m]
        self.robot_connector_offfset = rospy.get_param(
            '~robot_connector_offfset', 0.18)
        # Distance from charging-station origin to the charging connector in
        # the x-axis direction [m]
        self.station_connector_offset = rospy.get_param(
            '~station_connector_offset', 0.22)
        # Radius of the area which publish the charging voltage [m]
        self.docking_area_radious = rospy.get_param(
            '~docking_area_radious', 0.03)
        # Range of angles to publish charging voltage (+-
        # 'publih_voltage_area_angle' [m])
        self.docking_area_angle = rospy.get_param('~docking_area_angle', 0.5)
        # Charging Voltage
        self.voltage = rospy.get_param('~voltage', 30.0)
        # Initialize publishers
        self.output_voltage_pub = rospy.Publisher(
            '~output_voltage', Float32, queue_size=5)
        self.output_current_pub = rospy.Publisher(
            '~output_current', Float32, queue_size=5)
        if self.publish_debug_topics:
            self.robot_connector_pose_pub = rospy.Publisher(
                '~debug/robot_pose', PoseStamped, queue_size=5)
            self.station_connector_pose_pub = rospy.Publisher(
                '~debug/station_pose', PoseStamped, queue_size=5)
            self.targeet_pose_pub = rospy.Publisher(
                '~debug/target_pose', PoseStamped, queue_size=5)
            self.docking_flag_pub = rospy.Publisher(
                '~debug/docking_flag', Bool, queue_size=5)
        # Wait for gazebo plugin topics
        rospy.logdebug('[DockingStationSimulation] Waiting for Gazebo topics')
        self.gazebo_robot_odom = rospy.wait_for_message(
            '~robot_odom', Odometry)
        self.gazebo_station_odom = rospy.wait_for_message(
            '~station_odom', Odometry)
        # Initialize subscribers
        rospy.Subscriber('~robot_odom', Odometry,
                         self.robot_pose_subscriber_callback)
        rospy.Subscriber('~station_odom', Odometry,
                         self.station_pose_subscriber_callback)
        rospy.logdebug('[DockingStationSimulation] Finish Initialization')

    def publish_data_loop(self):
        self.rate.sleep()
        docking_flag = self.validate_docking(
            self.get_connector_deviation())
        voltage_message = Float32()
        current_message = Float32()
        if docking_flag:
            voltage_message.data = self.voltage
            current_message.data = 5.0
        else:
            voltage_message.data = 0.0
            current_message.data = -3.0
        self.output_voltage_pub.publish(voltage_message)
        self.output_current_pub.publish(current_message)

    def get_connector_deviation(self):
        # Calculate position (robot to station) on x-y plane
        quaternion_world2robot = np.array([
            self.gazebo_robot_odom.pose.pose.orientation.x,
            self.gazebo_robot_odom.pose.pose.orientation.y,
            self.gazebo_robot_odom.pose.pose.orientation.z,
            self.gazebo_robot_odom.pose.pose.orientation.w])
        yaw_world2robot = self.quaternion2yaw(quaternion_world2robot)
        rotation_matrix_world2robot = self.yaw2matrix(yaw_world2robot)
        rotation_matrix_world2robot_inv = self.yaw2matrix(- yaw_world2robot)
        vector_world2robot = np.array([
            self.gazebo_robot_odom.pose.pose.position.x,
            self.gazebo_robot_odom.pose.pose.position.y])
        vector_world2station = np.array([
            self.gazebo_station_odom.pose.pose.position.x,
            self.gazebo_station_odom.pose.pose.position.y])
        vector_robot2station = vector_world2station - vector_world2robot
        rotated_robot2station = np.dot(
            rotation_matrix_world2robot_inv, vector_robot2station)
        # Calculate orientation (robot to station) on x-y plane
        quaternion_world2station = np.array([
            self.gazebo_station_odom.pose.pose.orientation.x,
            self.gazebo_station_odom.pose.pose.orientation.y,
            self.gazebo_station_odom.pose.pose.orientation.z,
            self.gazebo_station_odom.pose.pose.orientation.w])
        yaw_world2station = self.quaternion2yaw(quaternion_world2station)
        rotation_matrix_world2station = self.yaw2matrix(yaw_world2station)
        yaw_robot2station = yaw_world2station - yaw_world2robot
        # rotation_matrix_robot2station = self.yaw2matrix(yaw_robot2station)

        if self.publish_debug_topics:
            quaternion_robot2station = self.yaw2quaternion(yaw_robot2station)
            # Set ROS topic (PoseStamped)
            robot2station = PoseStamped()
            robot2station.header = self.gazebo_station_odom.header
            robot2station.pose = self.gazebo_station_odom.pose.pose
            robot2station.header.frame_id = self.gazebo_robot_odom.child_frame_id
            robot2station.pose.position.x = rotated_robot2station[0]
            robot2station.pose.position.y = rotated_robot2station[1]
            robot2station.pose.position.z = 0
            robot2station.pose.orientation.x = quaternion_robot2station[0]
            robot2station.pose.orientation.y = quaternion_robot2station[1]
            robot2station.pose.orientation.z = quaternion_robot2station[2]
            robot2station.pose.orientation.w = quaternion_robot2station[3]
            robot2robot = PoseStamped()
            robot2robot.header = self.gazebo_robot_odom.header
            robot2robot.pose = self.gazebo_robot_odom.pose.pose
            robot2robot.header.frame_id = self.gazebo_robot_odom.child_frame_id
            robot2robot.pose.position.x = 0
            robot2robot.pose.position.y = 0
            robot2robot.pose.position.z = 0
            robot2robot.pose.orientation.x = 0
            robot2robot.pose.orientation.y = 0
            robot2robot.pose.orientation.z = 0
            robot2robot.pose.orientation.w = 1
            self.robot_connector_pose_pub.publish(robot2robot)
            self.station_connector_pose_pub.publish(robot2station)
        # Calculate connector offsets
        vector_robot2connector = np.array(
            [self.robot_connector_offfset, 0])
        vector_world2robot_con = vector_world2robot + \
            np.dot(rotation_matrix_world2robot, vector_robot2connector)
        vector_station2connector = np.array(
            [self.station_connector_offset, 0])
        vector_world2station_con = vector_world2station + \
            np.dot(rotation_matrix_world2station, vector_station2connector)
        vector_robot2station_con = vector_world2station_con - vector_world2robot_con
        rotated_robot2station_con = np.dot(
            rotation_matrix_world2robot_inv, vector_robot2station_con)
        # Calculate target angle (opposite side of the charging station)
        # rotation_matrix_robot2target = self.yaw2matrix(yaw_robot2station + pi)
        quaternion_robot2target = self.yaw2quaternion(yaw_robot2station + pi)
        # Set ROS topic (PoseStamped)
        robot2station_con = PoseStamped()
        robot2robot.header = self.gazebo_robot_odom.header
        robot2robot.pose = self.gazebo_robot_odom.pose.pose
        robot2station_con.header.frame_id = self.gazebo_robot_odom.child_frame_id
        robot2station_con.pose.position.x = rotated_robot2station_con[0]
        robot2station_con.pose.position.y = rotated_robot2station_con[1]
        robot2station_con.pose.position.z = 0
        robot2station_con.pose.orientation.x = quaternion_robot2target[0]
        robot2station_con.pose.orientation.y = quaternion_robot2target[1]
        robot2station_con.pose.orientation.z = quaternion_robot2target[2]
        robot2station_con.pose.orientation.w = quaternion_robot2target[3]
        if self.publish_debug_topics:
            self.targeet_pose_pub.publish(robot2station_con)
        return robot2station_con

    def validate_docking(self, robot2station_connectors):
        translation_diff = sqrt(
            robot2station_connectors.pose.position.x ** 2
            + robot2station_connectors.pose.position.y ** 2)
        anguler_diff_quaternion = np.array([
            robot2station_connectors.pose.orientation.x,
            robot2station_connectors.pose.orientation.y,
            robot2station_connectors.pose.orientation.z,
            robot2station_connectors.pose.orientation.w])
        anguler_diff = abs(self.quaternion2yaw(anguler_diff_quaternion))
        docking_flag = False
        if translation_diff < self.docking_area_radious and anguler_diff > self.docking_area_angle:
            docking_flag = True
        if self.publish_debug_topics:
            docking_flag_msg = Bool()
            docking_flag_msg.data = docking_flag
            self.docking_flag_pub.publish(docking_flag_msg)
        return docking_flag

    def quaternion2yaw(self, quaternion=[0, 0, 0, 1]):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]
        yaw = np.arctan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)
        return yaw

    def yaw2matrix(self, yaw=0):
        cos = np.cos(yaw)
        sin = np.sin(yaw)
        rot = np.array([[cos, -sin], [sin, cos]])
        return rot

    def yaw2quaternion(self, yaw=0):
        cos = np.cos(yaw / 2.0)
        sin = np.sin(yaw / 2.0)
        quaternion = [0, 0, sin, cos]
        return quaternion

    def robot_pose_subscriber_callback(self, data):
        rospy.logdebug_throttle(
            1,
            '[DockingStationSimulation] robot_pose_subscriber_callback called (show every 1 second)')
        self.gazebo_robot_odom = data

    def station_pose_subscriber_callback(self, data):
        rospy.logdebug_throttle(
            1,
            '[DockingStationSimulation] station_pose_subscriber_callback called (show every 1 second)')
        self.gazebo_station_odom = data


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('charging_station_simulation', log_level=rospy.INFO)
    charging_station_simulation = DockingStationSimulation()
    while not rospy.is_shutdown():
        charging_station_simulation.publish_data_loop()
