---
title: "Chapter 8: Advanced Manipulation and Grasping"
sidebar_position: 8
---

# Chapter 8: Advanced Manipulation and Grasping

## Humanoid Hand Control

Humanoid robots need sophisticated hand control to interact with objects in the same way humans do. Unlike simple grippers that can only open and close, humanoid hands have multiple fingers with several joints each, allowing for complex grasps and fine manipulation.

### Hand Anatomy and Degrees of Freedom

A typical humanoid hand has:
- **Thumb**: 4 joints (opposition, flexion, abduction, and additional joint)
- **Index finger**: 3 joints (metacarpal, proximal, distal)
- **Middle finger**: 3 joints
- **Ring finger**: 3 joints
- **Pinky finger**: 3 joints

This gives us up to 16 degrees of freedom in a single hand, though some simplified designs use fewer joints to reduce complexity and cost.

### Types of Grasps

Humanoid robots can perform various types of grasps:

**Power Grasps**: Used for holding heavy or large objects with maximum force
- Cylindrical grasp: Wrapping fingers around a cylindrical object
- Spherical grasp: Surrounding a spherical object with all fingers
- Hook grasp: Using just the fingers to carry handles or straps

**Precision Grasps**: Used for delicate manipulation of small objects
- Tip pinch: Using thumb and finger tips to pick up small items
- Lateral pinch: Using thumb and side of index finger
- Tripod grasp: Using thumb, index, and middle fingers

[Diagram: Different types of humanoid hand grasps]

### Hand Control Systems

Humanoid hands can be controlled in several ways:
- **Position control**: Moving each joint to a specific angle
- **Force control**: Applying specific forces at contact points
- **Impedance control**: Controlling the hand's stiffness and compliance

## Object Interaction and Manipulation

### Object Recognition and Localization

Before a robot can manipulate an object, it must first recognize and locate it in 3D space:

**Computer Vision**: Cameras capture images that are processed to identify objects
- Object detection: Finding where objects are in the image
- Object recognition: Identifying what type of object it is
- Pose estimation: Determining the object's 3D position and orientation

**3D Reconstruction**: Creating a 3D model of the object and its environment
- Point cloud generation from depth sensors
- Surface normal estimation for grasp planning
- Collision detection with environment

### Grasp Planning

Grasp planning involves determining the best way to grasp an object:

**Geometric Approaches**: Analyze object shape to find stable grasp points
- Find parallel surfaces for pinch grasps
- Identify convex regions for wrap grasps
- Calculate contact points that ensure force closure

**Learning-Based Approaches**: Use machine learning to predict good grasp points
- Train on thousands of successful and failed grasps
- Consider object properties like weight and fragility
- Adapt to different object shapes and materials

### Manipulation Strategies

**Pre-grasp Planning**: Planning the approach to the object
- Path planning to avoid collisions
- Approach angle optimization
- Hand configuration before contact

**In-grasp Control**: Managing the grasp once contact is made
- Force control to avoid crushing fragile objects
- Slip detection and compensation
- Grasp adjustment during manipulation

**Post-grasp Manipulation**: Moving the object after grasping
- Trajectory planning for object transport
- Dynamic balancing during manipulation
- Coordinated arm and body movement

## ROS 2 Nodes for Grasping Tasks

### Action-Based Architecture

ROS 2 uses an action-based system for grasping tasks, which is perfect for long-running operations:

- **GraspAction**: High-level action that handles the entire grasping process
- **MoveToPoseAction**: For positioning the hand near the object
- **ApproachAction**: For the final approach to the object
- **LiftAction**: For lifting the object after grasp
- **TransportAction**: For moving the object to destination

### Key Components

**Object Perception Node**: Processes sensor data to identify objects
- Subscribes to camera topics
- Publishes object poses
- Maintains object map

**Grasp Planner Node**: Plans how to grasp objects
- Receives object information
- Generates grasp candidates
- Evaluates grasp quality

**Motion Controller Node**: Executes the grasping motion
- Receives grasp commands
- Controls arm and hand joints
- Monitors success/failure

**Grasp Manager Node**: Coordinates the entire process
- Orchestrates other nodes
- Handles error recovery
- Manages grasp database

## Code Example: Object Manipulation with ROS 2

Here's a complete example of a ROS 2 node that handles object grasping:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
import math

class GraspingController(Node):
    def __init__(self):
        super().__init__('grasping_controller')

        # Publishers
        self.joint_cmd_publisher = self.create_publisher(
            JointState, '/joint_states', 10)
        self.status_publisher = self.create_publisher(
            String, '/grasping_status', 10)

        # Subscribers
        self.object_pose_sub = self.create_subscription(
            Pose, '/detected_object_pose', self.object_pose_callback, 10)

        # Action clients for MoveIt
        self.move_group_client = ActionClient(
            self, MoveGroup, 'move_group')
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')

        # Internal state
        self.current_object_pose = None
        self.robot_joints = {}
        self.grasp_in_progress = False

        # Robot-specific parameters
        self.arm_joints = [
            'shoulder_pan_joint', 'shoulder_lift_joint',
            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.hand_joints = [
            'thumb_joint', 'index_finger_joint', 'middle_finger_joint',
            'ring_finger_joint', 'pinky_finger_joint'
        ]

        # Grasp parameters
        self.approach_distance = 0.1  # 10cm from object
        self.grasp_height_offset = 0.05  # 5cm above object center
        self.gripper_closed_position = 0.0  # Fully closed
        self.gripper_open_position = 0.8   # Fully open

        self.get_logger().info('Grasping Controller initialized')

    def object_pose_callback(self, msg):
        """Callback for detected object pose"""
        self.current_object_pose = msg
        self.get_logger().info(f'Object detected at position: ({msg.position.x}, {msg.position.y}, {msg.position.z})')

        # Start grasping sequence
        if not self.grasp_in_progress:
            self.initiate_grasp_sequence()

    def initiate_grasp_sequence(self):
        """Start the complete grasping sequence"""
        if self.current_object_pose is None:
            self.get_logger().warn('No object detected for grasping')
            return

        self.grasp_in_progress = True
        self.publish_status('Starting grasp sequence')

        # Step 1: Move to approach position
        self.get_logger().info('Planning approach to object')
        approach_pose = self.calculate_approach_pose(self.current_object_pose)

        # Step 2: Execute approach movement
        future = self.move_to_pose(approach_pose)
        if future:
            future.add_done_callback(self.on_approach_completed)

    def calculate_approach_pose(self, object_pose):
        """Calculate safe approach pose above the object"""
        approach_pose = Pose()

        # Approach from above, at a safe distance
        approach_pose.position.x = object_pose.position.x
        approach_pose.position.y = object_pose.position.y
        approach_pose.position.z = object_pose.position.z + self.grasp_height_offset + self.approach_distance

        # Orient gripper to approach vertically
        approach_pose.orientation = self.calculate_approach_orientation(object_pose)

        return approach_pose

    def calculate_approach_orientation(self, object_pose):
        """Calculate gripper orientation for approach"""
        # For simple objects, approach from above with gripper facing down
        # This would typically involve more complex calculations for real objects
        orientation = Quaternion()

        # Simple orientation: gripper facing down (Z-axis down)
        orientation.x = 0.0
        orientation.y = 0.707  # sqrt(2)/2 for 90-degree rotation
        orientation.z = 0.0
        orientation.w = 0.707  # sqrt(2)/2 for 90-degree rotation

        return orientation

    def move_to_pose(self, target_pose):
        """Send MoveGroup action to reach target pose"""
        if not self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('MoveGroup action server not available')
            return None

        goal_msg = MoveGroup.Goal()

        # Set up the move group goal
        goal_msg.request.group_name = 'manipulator'  # Name of the robot arm group
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 10.0

        # Define target pose constraint
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import TransformStamped

        # Add position and orientation constraints
        constraints = Constraints()

        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'ee_link'  # End effector link name
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        pos_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))  # 1cm tolerance
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = 'base_link'
        orient_constraint.link_name = 'ee_link'
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)

        goal_msg.request.path_constraints = constraints

        return self.move_group_client.send_goal_async(goal_msg)

    def on_approach_completed(self, future):
        """Callback when approach movement is completed"""
        goal_result = future.result()

        if goal_result.successful:
            self.get_logger().info('Approach completed successfully')
            self.publish_status('Approach completed, moving to grasp position')

            # Step 3: Move down to object
            grasp_pose = self.calculate_grasp_pose(self.current_object_pose)
            future = self.move_to_pose(grasp_pose)
            if future:
                future.add_done_callback(self.on_grasp_position_reached)
        else:
            self.get_logger().error('Approach failed')
            self.publish_status('Approach failed')
            self.grasp_in_progress = False

    def calculate_grasp_pose(self, object_pose):
        """Calculate final grasp pose at the object"""
        grasp_pose = Pose()

        # Position at object center
        grasp_pose.position.x = object_pose.position.x
        grasp_pose.position.y = object_pose.position.y
        grasp_pose.position.z = object_pose.position.z + self.grasp_height_offset

        # Same orientation as approach
        grasp_pose.orientation = self.calculate_approach_orientation(object_pose)

        return grasp_pose

    def on_grasp_position_reached(self, future):
        """Callback when grasp position is reached"""
        goal_result = future.result()

        if goal_result.successful:
            self.get_logger().info('Grasp position reached')
            self.publish_status('Grasp position reached, closing gripper')

            # Step 4: Close gripper
            self.close_gripper()
            timer = self.create_timer(1.0, self.on_gripper_closed)  # Wait 1 second for grip
        else:
            self.get_logger().error('Grasp position approach failed')
            self.publish_status('Grasp position approach failed')
            self.grasp_in_progress = False

    def close_gripper(self):
        """Send command to close the gripper"""
        self.get_logger().info('Closing gripper')

        # Send gripper close command
        joint_state = JointState()
        joint_state.name = self.hand_joints
        joint_state.position = [self.gripper_closed_position] * len(self.hand_joints)

        self.joint_cmd_publisher.publish(joint_state)

    def on_gripper_closed(self):
        """Callback after gripper is closed"""
        self.get_logger().info('Gripper closed, attempting to lift object')
        self.publish_status('Gripper closed, lifting object')

        # Remove the timer
        timer = self.timers[0]  # Assuming this is the only timer
        timer.destroy()

        # Step 5: Lift object slightly
        lift_pose = self.calculate_lift_pose(self.current_object_pose)
        future = self.move_to_pose(lift_pose)
        if future:
            future.add_done_callback(self.on_lift_completed)

    def calculate_lift_pose(self, object_pose):
        """Calculate pose after lifting the object"""
        lift_pose = Pose()

        # Lift 5cm above grasp position
        lift_pose.position.x = object_pose.position.x
        lift_pose.position.y = object_pose.position.y
        lift_pose.position.z = object_pose.position.z + self.grasp_height_offset + 0.05  # Lift 5cm

        # Maintain same orientation
        lift_pose.orientation = self.calculate_approach_orientation(object_pose)

        return lift_pose

    def on_lift_completed(self, future):
        """Callback when lift is completed"""
        goal_result = future.result()

        if goal_result.successful:
            self.get_logger().info('Object lifted successfully')
            self.publish_status('Object grasped and lifted successfully')
        else:
            self.get_logger().error('Lift failed - object may have slipped')
            self.publish_status('Lift failed - object may have slipped')

        # Reset for next grasp
        self.grasp_in_progress = False

    def publish_status(self, status):
        """Publish current status"""
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = GraspingController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Interactive Exercises

**[Interactive Exercise: Grasp Planning Challenge]**
Imagine a humanoid robot needs to pick up a glass of water without spilling it. List the steps the robot should take, considering:
1. How should the robot approach the glass?
2. Which fingers should be used and where should they make contact?
3. How much force should be applied to prevent dropping but avoid breaking the glass?
4. What safety measures should be in place in case the grasp fails?

**[Interactive Exercise: Object Manipulation Scenario]**
Design a manipulation task where a humanoid robot needs to stack blocks of different sizes. Create a step-by-step plan that includes:
1. How the robot will identify and locate each block
2. The sequence of grasps needed to pick up and place each block
3. How the robot will maintain balance while manipulating objects
4. What to do if a block slips during the task

[Interactive Element: Grasp Simulator - Visualize different grasp types on various object shapes]

## Summary

Advanced manipulation and grasping require sophisticated integration of perception, planning, and control systems. Humanoid robots must be able to recognize objects, plan stable grasps, execute precise movements, and adapt to changing conditions during manipulation. The ROS 2 framework provides the necessary tools and architecture to coordinate these complex tasks, enabling robots to perform dexterous manipulation in real-world environments.