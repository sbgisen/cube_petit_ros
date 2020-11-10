#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import rospy

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import actionlib
import tf

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path

from math import pi, atan2, cos


class moveBaseCurrentGoal(object):
    # move_base/current_goal Subscriber
    # 外部ノードでmove_base/current_goalを更新された時に新ゴールとして登録する

    def __init__(self):
        self.current_goalpose = None
        rospy.Subscriber("/move_base/current_goal", PoseStamped, self._current_goal_cb, queue_size=1)

    def _current_goal_cb(self, posestamped):
        self.current_goalpose = posestamped

    def get_current_goal(self):
        return self.current_goalpose


class globalPathSubscriber(object):
    # global path subscriber
    # global pathの/map座標系における向きの算出および
    # global pathが読み込まれているかの判定を行う

    def __init__(self):
        self.calculate_global_path_dir = False
        self.current_goalpose = None
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self._global_path_cb, queue_size=2)

    # global pathの最初の二点からMap座標系における角度を計算
    def _global_path_cb(self, path):
        # TODO global pathのどの地点のベクトルを算出すればいいかの検討xy_tolelanceと同じくらいか
        if len(path.poses) > 10:  # 10はfootprintより小さい程度の距離として設定
            first_pose = [path.poses[8].pose.position.x, path.poses[8].pose.position.y]
            second_pose = [path.poses[9].pose.position.x, path.poses[9].pose.position.y]
            self.progress_dir_rad = atan2(second_pose[1] - first_pose[1],
                                          second_pose[0] - first_pose[0])
            self.current_goalpose = path.poses[-1]
            rospy.logdebug('goal update')
            self.calculate_global_path_dir = True
        else:  # 近い距離の時はrotateしない
            self.calculate_global_path_dir = False

    def get_state(self):  # return bool
        return self.calculate_global_path_dir

    def get_progress_dir_rad(self):  # return path direction[rad]
        return self.progress_dir_rad

    def get_current_goalpose(self):  # return geometry_msgs/pose
        return self.current_goalpose


class rotate_in_place(object):
    # Global pathが反対方向に行く時に方向転換してから移動する

    def __init__(self):
        self.calculate_global_path_dir = False
        self.move_base_client = moveBaseActionClient()
        self.global_path_sub = globalPathSubscriber()
        # subscribe the robot position at /map frame
        rospy.Subscriber("robot_pose", PoseStamped,
                         self._robot_pose_callback,
                         queue_size=1)
        self.tf1_listener = tf.TransformListener()

    # robotの現在の座標と向きの取得
    def _robot_pose_callback(self, robot_pose):
        self.robot_pose = robot_pose
        robot_orientation_q = robot_pose.pose.orientation
        self.robot_orientation_rad = euler_from_quaternion((robot_orientation_q.x, robot_orientation_q.y, robot_orientation_q.z, robot_orientation_q.w))

    def pose_to_goal(self, pose_stamped=PoseStamped()):  # PoseStamped型からMoveBaseGoal型に変換する関数
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose_stamped.pose.position.x
        goal.target_pose.pose.position.y = pose_stamped.pose.position.y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.x = pose_stamped.pose.orientation.x
        goal.target_pose.pose.orientation.y = pose_stamped.pose.orientation.y
        goal.target_pose.pose.orientation.z = pose_stamped.pose.orientation.z
        goal.target_pose.pose.orientation.w = pose_stamped.pose.orientation.w
        return goal

    def _turn_in_place(self, tf_robot='base_link', theta_ofs=0):
        rospy.logdebug("cos theta %f" % cos(self.progress_dir_rad - self.robot_orientation_rad[2]))
        # global pathの向きとロボットの向きの比較
        cos_theta = cos(self.progress_dir_rad - self.robot_orientation_rad[2])
        if cos_theta < cos(100.0 / 180.0 * pi):
            # self.calculate_global_path_dir = True
            rospy.logdebug('rotate here!!')
            self.move_base_client.cancel_goals()
            rospy.sleep(1)
            target_from_map = PoseStamped()
            target_from_map.header.stamp = rospy.Time(0)
            target_from_map.header.frame_id = 'map'
            target_from_map.pose.position.x = self.robot_pose.pose.position.x
            target_from_map.pose.position.y = self.robot_pose.pose.position.y
            target_from_map.pose.position.z = self.robot_pose.pose.position.z
            tmp_orientation = quaternion_from_euler(0, 0, self.progress_dir_rad)
            target_from_map.pose.orientation.x = tmp_orientation[0]
            target_from_map.pose.orientation.y = tmp_orientation[1]
            target_from_map.pose.orientation.z = tmp_orientation[2]
            target_from_map.pose.orientation.w = tmp_orientation[3]
            # マップ座標系に変換
            _goal = self.pose_to_goal(target_from_map)
            # move_baseのゴール地点を送信
            if self.move_base_client.send_goal_to_action_server(duration=30, goal=_goal):

                _current_goal = self.pose_to_goal(self.current_goalpose)
                self.move_base_client.send_goal_to_action_server(duration=30, goal=_current_goal)
                return
            else:
                return

    def main_loop(self):  # mainの中でループ実行
        self.calculate_global_path_dir = False
        self.calculate_global_path_dir = self.global_path_sub.get_state()
        if self.calculate_global_path_dir is True:
            self.current_goalpose = self.global_path_sub.get_current_goalpose()
            self.progress_dir_rad = self.global_path_sub.get_progress_dir_rad()
            self._turn_in_place(theta_ofs=self.progress_dir_rad)
            rospy.sleep(0.1)
        else:
            rospy.sleep(0.1)


class moveBaseActionClient:  # move_baseに走行動作要求をするクライアント
    # from ar_docking/scripts/ar_docking_server.py
    def __init__(self):
        self.move_base_ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        while not self.move_base_ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo('Waiting for the move_base action server to come up')
        rospy.loginfo('move_base action server comes up')
        self.move_base_goal = MoveBaseGoal()

    def _set_next_goal(self, goal=MoveBaseGoal()):  # goal:MoveBaseGoal()
        self.move_base_goal.target_pose.header.frame_id = 'map'
        self.move_base_goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base_goal.target_pose.pose.position.x = goal.target_pose.pose.position.x
        self.move_base_goal.target_pose.pose.position.y = goal.target_pose.pose.position.y
        self.move_base_goal.target_pose.pose.position.z = 0
        self.move_base_goal.target_pose.pose.orientation.x = goal.target_pose.pose.orientation.x
        self.move_base_goal.target_pose.pose.orientation.y = goal.target_pose.pose.orientation.y
        self.move_base_goal.target_pose.pose.orientation.z = goal.target_pose.pose.orientation.z
        self.move_base_goal.target_pose.pose.orientation.w = goal.target_pose.pose.orientation.w

    def _send_goal(self, duration=30):
        rospy.loginfo("Sending goal to navigation via actionlib")
        rospy.logdebug(self.move_base_goal)
        # self.move_base_ac.cancel_all_goals()
        self.move_base_ac.send_goal(self.move_base_goal)
        succeeded = self.move_base_ac.wait_for_result(rospy.Duration(duration))
        state = self.move_base_ac.get_state()
        if (state == 3) and succeeded:  # state:3 <- navigation reached goal
            rospy.loginfo("Navigation Succeeded")
            return 1
        elif(state == 2):
            rospy.logwarn("Another goal is set, state: " + str(state))
            return 0
        else:
            rospy.logwarn("Navigation Timeout, state: " + str(state))
            return 0
            # return succeeded

    def send_goal_to_action_server(self, duration=30, goal=MoveBaseGoal()):  # 成功時1，失敗時0を返す
        self._set_next_goal(goal)
        succeeded = self._send_goal(duration)
        return succeeded

    def cancel_goals(self):
        self.move_base_ac.cancel_all_goals()
        self.move_base_ac.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('rotate_to_progress')
    rotate = rotate_in_place()
    while not rospy.is_shutdown():
        rotate.main_loop()
    rospy.spin()
