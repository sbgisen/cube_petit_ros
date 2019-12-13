#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
File: move_base_sub.py
Description: 
    move_base client Topic subscriber utility tools
    rotate_in_place_vel.pyにて走行状態を取得する目的で使用
    subscribe topic: 
      move_base/status, move_base/feedback, move_base/result, local_costmap/costmap_updates
'''


# move_base client TopicのSubscriber utility tools
# move_base/status, move_base/feedback, move_base/result

import rospy
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult
from map_msgs.msg import OccupancyGridUpdate


class moveBaseResultSubscriber(object):
    """
    /move_base/resultをSubscribeするClass
    """

    def __init__(self):
        rospy.Subscriber("move_base/result", MoveBaseActionResult,
                         self._result_callback, queue_size=1)
        self.subscribe_cancel = False
        rospy.loginfo('Init move_base result subscriber')

    def _result_callback(self, result):
        if result.status.status is 2:
            self.subscribe_cancel = True
        else:
            self.subscribe_cancel = False

    @property
    def exist_cancel(self):
        return self.subscribe_cancel

    @exist_cancel.setter
    def exist_cancel(self, input_result):
        if type(input_result) is bool:
            self.subscribe_cancel = input_result
        else:
            self.subscribe_cancel = False


class moveBaseStatusSubscriber(object):
    """
    /move_base/statusをSubscribeするClass
    statusによって走行中か判別
    actionlib_msgs/GoalStatusArray:
    """
    def __init__(self):
        rospy.Subscriber("move_base/status", GoalStatusArray,
                         self._status_callback, queue_size=1)
        self.is_cancel = False
        rospy.loginfo('Init move_base status subscriber')

    def _status_callback(self, status):
        self.statusArray = status.status_list
        for status in self.statusArray:
            if status.status is 0 or status.status is 1:  # PREEMPTED
                self.cancel = False
                break
            else:
                self.cancel = True

    @property
    def status(self):  # return Bool
        return self.cancel


class moveBaseFeedbackSubscriber(object):
    """
    /move_base/feedbackをSubscribeするClass
    move_baseが動いている間はFeedbackがPublishされるため,動作中かの判定に使用
    recover_behaivor中にFeedbackは出ない
    """
    def __init__(self):
        rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback,
                         self._feedback_callback, queue_size=1)
        self.feedback_time = rospy.Time(0)
        rospy.loginfo('Init move_base feedback subscriber')

    def _feedback_callback(self, feedback):
        self.feedback_time = feedback.header.stamp

    def get_feedbacktime(self):  # feedbackの時刻[s]を返す
        return self.feedback_time.to_sec()


class localCostmapSubscriber(object):
    """
    local costmap subscriber
    robotのfootprintの中心点周辺の4点の内cost最大の点を評価
    """
    def __init__(self):
        self.pose_cost = 0
        # subscribe local costmap
        rospy.Subscriber("/move_base/local_costmap/costmap_updates",
                         OccupancyGridUpdate,
                         self._local_costmap_cb,
                         queue_size=1)
        rospy.loginfo('Init local costmap subscriber')

    # local costmapでロボットの座標と前後左右1セルのコストの最大値
    # (height - 1) * width /2 がrobot原点
    def _local_costmap_cb(self, costmap):
        cost_array = []
        cells = [-1, 0]
        for i in cells:
            cost_array.append(costmap.data[(costmap.height - 1) *
                              costmap.width / 2 + i])
            cost_array.append(costmap.data[(costmap.height + 1) *
                              costmap.width / 2 + i])
        self.pose_cost = max(cost_array)
        rospy.logdebug('local costmap cost %d' % self.pose_cost)

    @property
    def max_cost(self):  # return max cost at robot center position
        return self.pose_cost
