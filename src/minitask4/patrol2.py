#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib


class Pose:
    """表示机器人当前位姿的类"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0


class Waypoint:
    """表示目标点的类"""
    def __init__(self, x, y, w=1.0):
        self.x = x
        self.y = y
        self.w = w


class RobotPosition:
    """订阅`/odom`主题以获取机器人当前位姿"""
    def __init__(self):
        self.pose = Pose()
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        """Odometry消息的回调函数，提取位姿信息"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.pose.x = position.x
        self.pose.y = position.y
        self.pose.theta = yaw
    
    def log_position(self):
        """打印当前机器人位姿"""
        rospy.loginfo(f"Robot Position -> X: {self.pose.x}, Y: {self.pose.y}, Theta: {self.pose.theta}")


class NavigationClient:
    """导航控制器，使用move_base发送导航目标"""
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.robot_position = RobotPosition()
        self.client.wait_for_server()
            
    def send_goal(self, waypoint):
        """向move_base发送目标"""
        rospy.loginfo(f"Sending goal -> X: {waypoint.x}, Y: {waypoint.y}")

        # 等待服务器连接
        self.client.wait_for_server()

        # 设置导航目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint.x
        goal.target_pose.pose.position.y = waypoint.y
        goal.target_pose.pose.orientation.w = waypoint.w

        self.client.send_goal(goal)
        if not self.client.wait_for_result(timeout=rospy.Duration(60)):
            rospy.logerr("Failed to reach goal within time limit. Retrying...")
            self.client.cancel_goal()
            return False

        state = self.client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
            return True
        else:
            rospy.logwarn(f"Goal not reached, state: {state}")
            return False
    
    # def is_at_goal(self, waypoint, tolerance=0.5):
    #     """检查机器人是否已到达目标点，判断阈值为tolerance"""
    #     dist = math.sqrt((self.robot_position.pose.x - waypoint.x)**2 + (self.robot_position.pose.y - waypoint.y)**2)
    #     return dist < tolerance

    def waypoints_patrol(self):
        """导航巡逻任务"""
        # 定义一组导航目标点
        waypoints = [
            Waypoint(0.82, 1.68, 1.0),
            Waypoint(3.36, 1.29, 1.0),
            Waypoint(4.44, -1.2, 1.0),
            Waypoint(2.8, -1.4, 1.0),
            Waypoint(2.34, 0.3, 1.0),
            Waypoint(-0.03, -1.72, 1.0)
        ]
        for waypoint in waypoints:
            if self.send_goal(waypoint):
                self.robot_position.log_position()
                rospy.loginfo("Waiting for 1 second before moving to the next waypoint...")
                rospy.sleep(1)  # 等待 1 秒钟
            else:
                rospy.logwarn(f"Skipping waypoint -> X: {waypoint.x}, Y: {waypoint.y}")
            
            # 直到目标位置完全到达，再继续执行下一步
            # while not self.is_at_goal(waypoint):
            #     rospy.loginfo(f"Robot is still moving towards X: {waypoint.x}, Y: {waypoint.y}.")
            #     rospy.sleep(1)  # 每0.5秒检查一次

if __name__ == '__main__':
    try:
        rospy.init_node('robot_navigation')
        nav_client = NavigationClient()
        nav_client.waypoints_patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation task interrupted.")
