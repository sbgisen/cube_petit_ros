#! /usr/bin/env python
# -*- encoding: utf-8 -*-

from __future__ import print_function

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the ar_docking action, including the
# goal message and the result message.
import ar_docking.msg
from std_msgs.msg import Float32
from std_msgs.msg import Int8

# import ar_docking.msg
from bzrlib.commands import Command
# import ar_docking_server


def ar_docking_action_client(goal='ar_marker_0'):
    # Creates the SimpleActionClient, passing the type of the action
    # (ARDockingAction) to the constructor.
    client = actionlib.SimpleActionClient('ar_docking_action', ar_docking.msg.ARDockingAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Creates a goal to send to the action server.
    _goal = ar_docking.msg.ARDockingGoal(marker_id=goal)
    # Sends the goal to the action server.
    client.send_goal(_goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    #return client.get_result()  # A ARDockingResult
    #メモリ解放
    return_tmp = client.get_state()
#     del client
#     del _goal
    return return_tmp
    
class doChargeSubscriber: #ドッキング動作開始コマンド(1でドッキング開始,2でアンドック)
    def __init__(self):
        self._status = 0 #0:何もなし, 1:ドッキング開始, 2:アンドック
        self._sub = rospy.Subscriber('/do_charge', Int8, self._callback)
    def _callback(self, data):
#         rospy.loginfo("I heard %d", data.data)
        self._status = data.data
    def set_status(self, status):
        self._status = status
    def get_status(self):
        _tmp = self._status
        self._status = 0
        return _tmp

class arDockingActionClient: #ドッキング動作開始コマンド(1でドッキング開始,2でアンドック)
    def __init__(self):
        #reference: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29
        self.client = actionlib.SimpleActionClient('ar_docking_action', ar_docking.msg.ARDockingAction)
        self.client.wait_for_server()
        self.do_training = False # True: ぶつかり稽古
        rospy.sleep(1)
    def do_charge(self):
        if self.do_training == False:
            goal = ar_docking.msg.ARDockingGoal(marker_id='ar_marker_0',Command="charge")
        else:
            goal = ar_docking.msg.ARDockingGoal(marker_id='ar_marker_0',Command="recharge")
        self.client.send_goal(goal)
        self.client.wait_for_result()
    def do_undock(self):
        goal = ar_docking.msg.ARDockingGoal(marker_id='ar_marker_0',Command="undock")
        self.client.send_goal(goal)
        self.client.wait_for_result()

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.      
    rospy.init_node('ar_docking_action_client')
    #reference: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29
    client = arDockingActionClient()
    
    do_charge_subscriber = doChargeSubscriber()
    do_training = False # True: ぶつかり稽古
    doCharge = 0 #0:何もなし, 1:ドッキング開始, 2:アンドック
    doChargeOld = 0 #0:何もなし, 1:ドッキング開始, 2:アンドック
    dock_or_undock_state = 0 #0:not docked, 1:docked
    rospy.sleep(1)

    rospy.loginfo("ar_client:start up")
    while not rospy.is_shutdown():
        rospy.sleep(1)
        #待機
        doCharge = do_charge_subscriber.get_status()
        if doChargeOld != doCharge:
            rospy.loginfo("ar_client:command received")
            if doCharge == 0:
                rospy.sleep(1)
            elif doCharge == 1:
                client.do_charge()
            elif doCharge == 2:
                client.do_undock()
        doChargeOld = doCharge
