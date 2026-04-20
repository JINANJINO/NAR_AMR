#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import time

class MultiGoalNavigator:
    def __init__(self):
        self.navigator = BasicNavigator()

        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.w = 1.0

        self.navigator.setInitialPose(self.initial_pose)

        # self.goals_relative = [
        #     {'x': 3.0, 'y': 0.0, 'theta': math.radians(0)},
        #     {'x': 3.0, 'y': 0.0, 'theta': math.radians(-90)},
        #     {'x': 3.0, 'y': -3.0, 'theta': math.radians(-90)},
        #     {'x': 3.0, 'y': -3.0, 'theta': math.radians(-180)},
        #     {'x': 0.0, 'y': -3.0, 'theta': math.radians(-180)},
        #     {'x': 0.0, 'y': -3.0, 'theta': math.radians(90)},
        #     {'x': 0.0, 'y': 0.0, 'theta': math.radians(90)},
        #     {'x': 0.0, 'y': 0.0, 'theta': math.radians(0)},
        #     {'x': 0.0, 'y': 0.0, 'theta': math.radians(0)},
        # ]

        self.goals_relative = [
            {'x': 0.0, 'y': 0.0, 'theta': math.radians(0)},
            {'x': -3.0, 'y': 0.0, 'theta': math.radians(0)},
            {'x': -3.0, 'y': -1.0, 'theta': math.radians(0)},
            {'x': 0.0, 'y': -1.0, 'theta': math.radians(0)},
            {'x': 0.0, 'y': 0.0, 'theta': math.radians(0)},
        ]

        self.goal_poses = [self.calc_goal_abs(goal) for goal in self.goals_relative]

    def calc_goal_abs(self, goal_relative):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = goal_relative['x']
        pose.pose.position.y = goal_relative['y']

        # Calculate quaternion
        pose.pose.orientation.z = math.sin(goal_relative['theta'] / 2)
        pose.pose.orientation.w = math.cos(goal_relative['theta'] / 2)

        print(
            f'Goal: x={goal_relative["x"]}, y={goal_relative["y"]}, '
            f'theta={math.degrees(goal_relative["theta"])}°'
        )

        return pose

    def navigate(self):
        try:
            # Wait until Nav2 is active
            self.navigator.waitUntilNav2Active()

            # Follow waypoints
            self.navigator.followWaypoints(self.goal_poses)

            # Check task completion
            while not self.navigator.isTaskComplete():
                # Spin the navigator node to process events
                rclpy.spin_once(self.navigator, timeout_sec=0.1)
                time.sleep(0.1)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Navigation completed successfully')
            else:
                print('Navigation failed')

            # Shutdown navigator lifecycle and destroy the node
            # self.navigator.lifecycleShutdown()
            self.navigator.destroyNode()

        except Exception as e:
            print(f'Navigation error: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        navigator = MultiGoalNavigator()
        navigator.navigate()
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        # Destroy the navigator object and shutdown rclpy
        del navigator
        rclpy.shutdown()

if __name__ == '__main__':
    main()