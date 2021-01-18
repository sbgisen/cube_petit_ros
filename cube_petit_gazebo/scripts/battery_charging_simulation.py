#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from brass_gazebo_battery.srv import SetCharging


class BatteryChargingSimulation:
    def __init__(self):
        rospy.logdebug('[BatteryChargingSimulation] Start Initialization')
        self.rate = rospy.Rate(20)
        # Get rosparam
        # if 'docking_station_voltage_in' > 'voltage_threshold', start 'set_charging' service.
        self.brass_battery_service_name = rospy.get_param(
            '~set_charging', '/gazebo/battery_simulation/cube_petit/set_charging')
        self.voltage_threshold = rospy.get_param('~voltage_threshold', 0.1)
        # Start Subscriber
        self.docking_station_voltage = rospy.wait_for_message(
            '~docking_station_voltage_in', Float32)
        # Subscriber
        rospy.Subscriber(
            '~docking_station_voltage_in', Float32,
            self.voltage_subscriber_callback)
        self.current_charging_state = False
        self.docking_flag = False
        rospy.logdebug('[BatteryChargingSimulation] Finish Initialization')

    def check_docking_loop(self):
        self.rate.sleep()
        if self.docking_flag and not self.current_charging_state:
            # rospy.loginfo('---battery_charging_simulation---check_docking_loop_true1')
            self.set_charging_client(True)
            self.current_charging_state = True
            # rospy.loginfo('---battery_charging_simulation---check_docking_loop_true2')
        elif not self.docking_flag and self.current_charging_state:
            #rospy.loginfo('---battery_charging_simulation---check_docking_loop_false1')
            self.set_charging_client(False)
            self.current_charging_state = False
            # rospy.loginfo('---battery_charging_simulation---check_docking_loop_false2')

    def voltage_subscriber_callback(self, data):
        # rospy.loginfo('---battery_charging_simulation---voltage_subscriber')
        rospy.logdebug_throttle(
            1,
            '[BatteryChargingSimulation] voltage_subscriber_callback called (show every 1 second)')
        self.docking_flag = bool(data.data > self.voltage_threshold)
        # rospy.loginfo('---battery_charging_simulation---docking flag %d', self.docking_flag)

    def set_charging_client(self, charging_flag=True):
        # rospy.loginfo('---battery_charging_simulation---set_charge_client 1')
        rospy.loginfo(self.brass_battery_service_name)
        rospy.wait_for_service(self.brass_battery_service_name)
        # rospy.loginfo('---battery_charging_simulation---set_charge_client 2')
        try:
            # rospy.loginfo('---battery_charging_simulation---set_charge_client 3')
            call_charging_service = rospy.ServiceProxy(
                self.brass_battery_service_name, SetCharging)
            service_result = call_charging_service(charging_flag)
            # rospy.loginfo('---battery_charging_simulation---set_charge_client 4')
            return service_result
        except rospy.ServiceException as err:
            rospy.logerr('Service call failed: %s' % err)


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('battery_charging_simulation', log_level=rospy.INFO)
    # rospy.loginfo('---battery_charging_simulation---')
    charging_station_simulation = BatteryChargingSimulation()
    while not rospy.is_shutdown():
        charging_station_simulation.check_docking_loop()
