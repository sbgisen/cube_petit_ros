#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
File: rotate_in.py
Description:
    move_base時に進行方向とGlobal pathが逆方向の時ににその場旋回するNode
'''
import rospy
from tf.transformations import euler_from_quaternion

from util.movebase_sub import moveBaseResultSubscriber,\
    moveBaseFeedbackSubscriber,\
    localCostmapSubscriber
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from math import atan2, cos, sin


class globalPathSubscriber(object):
    """
    global path subscriber
    global pathの/map座標系における向きの算出および
    global pathが読み込まれているかの判定を行う
    """

    def __init__(self):
        self.exist_path_dir = False
        self.progress_dir_rad = None
        self.robot_pose_sub = robotPoseSubscriber()
        rospy.Subscriber("/move_base/NavfnROS/plan", Path,
                         self._global_path_cb, queue_size=2)
        rospy.loginfo('Init global path subscriber')

    # global pathの最初の二点からMap座標系における角度を計算
    def _global_path_cb(self, path):
        # TODO global pathのどの地点のベクトルを算出すればいいかの検討xy_tolelanceと同じくらいか
        if len(path.poses) > 10:  # 10はfootprintより小さい程度の距離として設定
            first_pose = [path.poses[8].pose.position.x,
                          path.poses[8].pose.position.y]
            second_pose = [path.poses[9].pose.position.x,
                           path.poses[9].pose.position.y]
            self.progress_dir_rad = atan2(second_pose[1] - first_pose[1],
                                          second_pose[0] - first_pose[0])
            rospy.logdebug('goal update')
            self.exist_path_dir = True
        else:  # 近い距離の時はrotateしない
            self.exist_path_dir = False

    @property  # Read only
    def cos_btw_path_and_pose(self):
        path_dir_rad = self.progress_dir_rad
        robot_dir_rad = self.robot_pose_sub.robot_dir_rad[2]
        cos_theta = cos(path_dir_rad - robot_dir_rad)
        rospy.logdebug('cos %lf' % cos_theta)
        return cos_theta

    @property  # Read only
    def sin_btw_path_and_pose(self):
        path_dir_rad = self.progress_dir_rad
        robot_dir_rad = self.robot_pose_sub.robot_dir_rad[2]
        sin_theta = sin(path_dir_rad - robot_dir_rad)
        rospy.logdebug('sin %lf' % sin_theta)
        return sin_theta


class robotPoseSubscriber(object):
    """
    /map上の向き[rad]の取得
    """

    def __init__(self):
        self.robot_orientation_rad = None
        rospy.Subscriber("robot_pose", PoseStamped,
                         self._robot_pose_callback,
                         queue_size=1)
        rospy.loginfo('Init robot_pose subscriber')

    # robotの現在の座標と向きの取得[rad]
    def _robot_pose_callback(self, robot_pose):
        robot_orientation_q = robot_pose.pose.orientation
        self.robot_orientation_rad = \
            euler_from_quaternion((robot_orientation_q.x,
                                   robot_orientation_q.y,
                                   robot_orientation_q.z,
                                   robot_orientation_q.w))

    @property  # Read only
    def robot_dir_rad(self):
        return self.robot_orientation_rad


class rotate_in_place(object):
    '''
    現在の向きとGlobal pathの向きを比較してその場旋回するクラス
    publish:round_vel(twist)
    '''

    def __init__(self):
        self.get_global_path_dir = False
        self._init_pub_sub()

    def _init_pub_sub(self):
        self.global_path_sub = globalPathSubscriber()
        self.local_cost_sub = localCostmapSubscriber()
        self.mv_result_sub = moveBaseResultSubscriber()
        self.mv_feedback_sub = moveBaseFeedbackSubscriber()
        # cmd_vel publisher
        self.rotate_vel_pub = rospy.Publisher("rotate_vel",
                                              Twist, queue_size=1)
        self.twist_msg = Twist()
        rospy.loginfo('[rotate_in_place] Init publisher and subscriber')

    def _publish_rotate_vel(self, vel_x=0.0, vel_z=0.0):
        # 回転速度のPublish
        self.twist_msg.linear.x = vel_x
        self.twist_msg.angular.z = vel_z
        self.rotate_vel_pub.publish(self.twist_msg)

    def _is_canceled(self):
        # rotate中にキャンセルされたら止まるためのメソッド
        # exist_cancelをWhile前で初期化しないと走行中にキャンセルされてもbreakしてしまう
        if self.mv_result_sub.exist_cancel:  # task is cancelled
            rospy.logwarn('[rotate_in_place] move_base task is cancelled.')
            self.mv_result_sub.exist_cancel = False
            return True
        else:
            return False

    def _is_stopped(self):
        # 時刻によるfeedbackが止まっているかの判定
        feedback_time_s = self.mv_feedback_sub.get_feedbacktime()
        if feedback_time_s < rospy.Time.now().to_sec() - 1:
            rospy.logwarn('[rotate_in_place] Feedback is stopped.')
            return True
        else:
            return False

    def _estimate_cost(self):
        # costmapの判定
        # costmapの値の参考値
        # "inscribed" cost is 99 "possibly circumscribed" cost is 50
        # 人が目の前に立ったら発生する値が80~90なので安全に振って75をしきい値とした
        safety_cost = rospy.get_param("~safety_cost", 75)
        rospy.logdebug('local costmap cost %d' % self.local_cost_sub.max_cost)
        if self.local_cost_sub.max_cost > safety_cost:
            rospy.logerr("[rotate_in_place] can't rotate in place becase there is a potential collision. Cost: % d" % self.local_cost_sub.max_cost)
            return True
        else:
            return False

    def _turn_in_place_by_cmd_vel(self, tf_robot='base_link'):
        # rosparam get min_in_place_rotational_vel
        rotate_vel_param = \
            "/move_base/TrajectoryPlannerROS/min_in_place_rotational_vel"
        rotate_vel = rospy.get_param(rotate_vel_param, 0.6)
        rotate_begin_rad = \
            rospy.get_param("~rotate_begin_threshold", 1.74)
        rotate_end_rad = \
            rospy.get_param("~rotate_end_threshold", 0.0873)
        self.mv_result_sub.exist_cancel = False  # exist_cancelの初期化

        if self.global_path_sub.cos_btw_path_and_pose < cos(rotate_begin_rad):
            rospy.loginfo('[rotate_in_place] Rotate here.')
            while(True):
                # 停止中だったりコストが高い時はrotateしない
                if self._is_stopped():
                    break
                if self._is_canceled():
                    self._publish_rotate_vel(vel_z=0)  # キャンセルされたら止まる
                    break
                if self._estimate_cost():
                    self._publish_rotate_vel(vel_z=0)  # ぶつからないように止まる
                    break
                if self.global_path_sub.cos_btw_path_and_pose > cos(rotate_end_rad):
                    rospy.loginfo('[rotate_in_place] Rotated.')
                    self._publish_rotate_vel(vel_z=0)  # 行き過ぎ防止
                    break

                # publish rotate_vel
                # sinの符号で回転方向を変える
                if self.global_path_sub.sin_btw_path_and_pose > 0:
                    self._publish_rotate_vel(vel_z=rotate_vel)
                else:
                    self._publish_rotate_vel(vel_z=-rotate_vel)
                rospy.sleep(0.2)
            return

    def run(self):  # mainの中でループ実行
        if self.global_path_sub.exist_path_dir is True:
            self._turn_in_place_by_cmd_vel()
            # reset exist_path_dir
            self.global_path_sub.exist_path_dir = False
            rospy.sleep(0.3)
        else:
            rospy.sleep(0.3)


if __name__ == '__main__':
    rospy.init_node('rotate_in_place')
    rotate = rotate_in_place()
    while not rospy.is_shutdown():
        rotate.run()
    rospy.spin()
