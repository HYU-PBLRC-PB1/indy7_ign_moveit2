#!/usr/bin/env python3

from moveit2 import MoveIt2Interface
import rclpy
import threading

from rclpy.node import Node
from std_msgs.msg import String

class TopicFollower(Node):

    def __init__(self):
        super().__init__('topic_follower')
        self.get_logger().info('init topic_follower')

        self.subscription = self.create_subscription(
            String,
            '/xyz_pose',
            self.pose_msg_callback,
            10
        )
        
        # Create MoveIt2 interface node
        self.moveit2_ = MoveIt2Interface()
        self.latest_position = [0.25, 0.25, 0.25]
        self.latest_quat = [1.0, 0.0, 0.0, 0.0]

        # Spin up multi-threaded executor
        self.executor_ = rclpy.executors.MultiThreadedExecutor(2)
        self.executor_.add_node(self)
        self.executor_.add_node(self.moveit2_)

        self.executor_.spin()
        
    def pose_msg_callback(self, msg):
        # ex) msg.data == "0.25 0.14 0.32"
        self.get_logger().info('xyz message: "%s"' % msg.data)

        xyz_msg = msg.data.split(' ')
        xyz = [float(elem) for elem in xyz_msg]

        self.get_logger().info(f"x:{xyz[0]}, y:{xyz[1]}, z:{xyz[2]}")
        self.latest_position[0] += xyz[0]       
        self.latest_position[1] += xyz[1]
        self.latest_position[2] += xyz[2]
        
        self.moveit2_.set_pose_goal(self.latest_position, self.latest_quat)
        # Plan and execute
        self.moveit2_.plan_kinematic_path()
        self.moveit2_.execute()

def main(args=None):
    rclpy.init(args=args)

    _topic_follower = TopicFollower()

    rclpy.shutdown()

if __name__ == "__main__":
    main()