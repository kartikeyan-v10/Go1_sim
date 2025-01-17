#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.30
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

def main():

    rclpy.init()
    nav = BasicNavigator()

    #wait for nav2
    #nav.waitUntilNav2Active()

    goal_pose1 = create_pose_stamped(nav, 4.7, 3.0, 1.57)
    goal_pose2 = create_pose_stamped(nav, 2.5, -2.15, 3.14)
    goal_pose3 = create_pose_stamped(nav, 7.0, -2.0, 0.0)
    goal_pose4 = create_pose_stamped(nav, 6.5, 0.75, 1.57)
    goal_pose5 = create_pose_stamped(nav, -2.0, 3.0, 3.14)
    goal_pose6 = create_pose_stamped(nav, 0.75, -2.30, 0.00)
    goal_pose7 = create_pose_stamped(nav, 0.00, 0.00, 0.00)

    waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose4,
                 goal_pose5, goal_pose6, goal_pose7]
    for i in range(7):
        nav.goToPose(waypoints[i])

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            #print(feedback)

    #get resut
    print(nav.getResult())

    rclpy.shutdown()

if __name__ == '__main__':
    main()