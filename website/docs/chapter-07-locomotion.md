---
title: "Chapter 7: Advanced Humanoid Locomotion"
sidebar_position: 7
---

# Chapter 7: Advanced Humanoid Locomotion

## Bipedal Walking and Balance Control

Humanoid robots face unique challenges when it comes to walking on two legs. Unlike wheeled robots that can simply rotate their wheels, humanoid robots must maintain balance while shifting their weight from one leg to another. This requires sophisticated control algorithms and careful coordination of multiple body parts.

### The Challenge of Bipedal Walking

Walking with two legs is something humans do naturally, but it's actually quite complex from an engineering perspective. When we walk, we're constantly falling forward and catching ourselves with our other leg. Humanoid robots must replicate this process while maintaining stability.

Key challenges include:
- **Maintaining Center of Mass**: The robot's center of mass must stay within the support polygon created by its feet
- **Dynamic Balance**: Unlike standing still, walking involves constant motion and requires dynamic balance control
- **Terrain Adaptation**: The robot must adjust its gait for different surfaces and obstacles
- **Energy Efficiency**: Walking should be as energy-efficient as possible to maximize battery life

### Zero Moment Point (ZMP) Control

One of the most important concepts in humanoid walking is the Zero Moment Point (ZMP). This is a point on the ground where the sum of all moments (rotational forces) caused by ground reaction forces equals zero.

In simple terms, if the robot's center of pressure stays within the area of its feet, it won't fall over. The ZMP controller continuously calculates where the robot's center of mass should be to maintain balance.

[Diagram: ZMP concept showing center of mass and support polygon]

### Walking Pattern Generation

Humanoid robots typically use pre-computed walking patterns that are adjusted in real-time based on sensor feedback. These patterns define:
- Foot placement positions
- Body trajectory (how the torso moves)
- Joint angles for each step
- Timing of each phase of the walking cycle

## Humanoid Kinematics and Dynamics

### Forward and Inverse Kinematics

Kinematics is the study of motion without considering the forces that cause it. For humanoid robots, we need to understand:

**Forward Kinematics**: Given the joint angles, where is the foot positioned? This is useful for predicting where the robot will place its foot.

**Inverse Kinematics**: Given where we want the foot to be, what joint angles are needed? This is essential for planning foot placement during walking.

For a humanoid leg, we typically have:
- Hip joints (3 degrees of freedom: roll, pitch, yaw)
- Knee joint (1 degree of freedom: pitch)
- Ankle joints (2 degrees of freedom: pitch, roll)

[Diagram: Humanoid leg with joint labels and degrees of freedom]

### Dynamics and Control

Dynamics considers the forces and torques needed to create motion. For walking, we need to calculate:
- Joint torques required to move the legs
- Forces needed to maintain balance
- How to distribute weight between both legs during walking

### Center of Mass Control

The center of mass (CoM) is crucial for balance. In humanoid robots, the CoM is typically located around the pelvis area. Controllers work to keep the CoM within safe boundaries by:
- Adjusting the robot's posture
- Modifying step timing and placement
- Using arm movements for balance assistance

## Code Example: Simple Walking Controller

Here's a simplified example of a walking controller using ROS 2 and the concept of ZMP:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')

        # Publishers for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)

        # Subscriber for IMU data (for balance feedback)
        self.imu_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Walking parameters
        self.step_length = 0.2  # meters
        self.step_height = 0.05  # meters
        self.step_time = 1.0  # seconds per step
        self.current_phase = 0.0  # 0.0 to 1.0

        # Initialize walking pattern
        self.timer = self.create_timer(0.01, self.walk_callback)

        # Robot kinematic parameters (simplified)
        self.hip_offset = 0.05  # distance from body to hip
        self.thigh_length = 0.35  # length of thigh
        self.shin_length = 0.35  # length of shin

        self.get_logger().info('Walking Controller initialized')

    def joint_state_callback(self, msg):
        """Callback for joint state feedback"""
        # Process joint positions for balance adjustment
        pass

    def calculate_foot_trajectory(self, phase, leg_side):
        """
        Calculate desired foot position based on walking phase
        phase: 0.0 to 1.0 representing progress through step cycle
        leg_side: 'left' or 'right'
        """
        # Determine which leg is swing leg vs stance leg
        if leg_side == 'left':
            swing_phase = phase
        else:  # right leg
            swing_phase = (phase + 0.5) % 1.0

        # Calculate step trajectory
        x_offset = self.step_length * (phase - 0.5)  # Forward/back motion
        y_offset = 0.0 if leg_side == 'left' else 0.2  # Lateral separation

        # Calculate step height (parabolic trajectory for swing leg)
        if swing_phase < 0.5:
            z_offset = self.step_height * (1 - np.cos(2 * np.pi * swing_phase))
        else:
            z_offset = self.step_height * (1 - np.cos(2 * np.pi * (1 - swing_phase)))

        return np.array([x_offset, y_offset, z_offset])

    def inverse_kinematics(self, target_position, leg_side):
        """
        Simple inverse kinematics for a 3-DOF leg
        target_position: [x, y, z] desired foot position
        leg_side: 'left' or 'right'
        """
        x, y, z = target_position

        # Calculate hip to ankle vector
        hip_to_ankle_x = x
        hip_to_ankle_y = y - (0.1 if leg_side == 'left' else -0.1)  # Hip lateral offset
        hip_to_ankle_z = z + 0.8  # Hip height (assuming 0.8m)

        # Distance from hip to ankle
        d = np.sqrt(hip_to_ankle_x**2 + hip_to_ankle_y**2 + hip_to_ankle_z**2)

        # Check if target is reachable
        leg_length = self.thigh_length + self.shin_length
        if d > leg_length:
            # Normalize to reachable position
            hip_to_ankle = np.array([hip_to_ankle_x, hip_to_ankle_y, hip_to_ankle_z])
            hip_to_ankle = hip_to_ankle * leg_length / d
            hip_to_ankle_x, hip_to_ankle_y, hip_to_ankle_z = hip_to_ankle

        # Calculate knee angle using law of cosines
        cos_knee = (self.thigh_length**2 + self.shin_length**2 - d**2) / (2 * self.thigh_length * self.shin_length)
        cos_knee = np.clip(cos_knee, -1, 1)  # Clamp to valid range
        knee_angle = np.pi - np.arccos(cos_knee)

        # Calculate hip angles
        hip_pitch = np.arctan2(-hip_to_ankle_z, np.sqrt(hip_to_ankle_x**2 + hip_to_ankle_y**2))
        hip_roll = np.arctan2(hip_to_ankle_y, np.sqrt(hip_to_ankle_x**2 + hip_to_ankle_z**2))
        hip_yaw = np.arctan2(hip_to_ankle_x, hip_to_ankle_z)

        return {
            'hip_pitch': hip_pitch,
            'hip_roll': hip_roll,
            'hip_yaw': hip_yaw,
            'knee_angle': knee_angle,
            'ankle_pitch': -hip_pitch,  # Simple ankle compensation
            'ankle_roll': -hip_roll
        }

    def walk_callback(self):
        """Main walking control callback"""
        # Update walking phase
        self.current_phase = (self.current_phase + 0.01/self.step_time) % 1.0

        # Calculate desired foot positions
        left_foot_pos = self.calculate_foot_trajectory(self.current_phase, 'left')
        right_foot_pos = self.calculate_foot_trajectory((self.current_phase + 0.5) % 1.0, 'right')

        # Calculate joint angles using inverse kinematics
        left_joints = self.inverse_kinematics(left_foot_pos, 'left')
        right_joints = self.inverse_kinematics(right_foot_pos, 'right')

        # Prepare joint command message
        cmd_msg = Float64MultiArray()

        # Pack joint angles into message (example for 6 joints per leg)
        joint_commands = [
            left_joints['hip_pitch'], left_joints['hip_roll'], left_joints['hip_yaw'],
            left_joints['knee_angle'], left_joints['ankle_pitch'], left_joints['ankle_roll'],
            right_joints['hip_pitch'], right_joints['hip_roll'], right_joints['hip_yaw'],
            right_joints['knee_angle'], right_joints['ankle_pitch'], right_joints['ankle_roll']
        ]

        cmd_msg.data = joint_commands
        self.joint_cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = WalkingController()

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

## Interactive Questions

**[Interactive Question: Balance Challenge]**
If a humanoid robot is walking and suddenly encounters a small step up in the ground, how should its control system respond to maintain balance? Consider both immediate reactions (within 0.1 seconds) and longer-term adjustments (over the next few steps).

**[Interactive Exercise: Gait Analysis]**
Watch a video of human walking and observe how the center of mass moves during a walking cycle. The human center of mass moves in a wave-like pattern up and down. Why is it important for a humanoid robot to mimic this motion pattern rather than keeping its body perfectly level?

[Interactive Element: Walking Pattern Simulator - Adjust step length, height, and timing to see effects on robot stability]

## Summary

Advanced humanoid locomotion combines principles of physics, control theory, and robotics engineering to create stable, efficient walking patterns. By understanding concepts like Zero Moment Point control, inverse kinematics, and dynamic balance, we can create robots that walk naturally and safely in human environments. The key to successful locomotion lies in the robot's ability to continuously adapt to changing conditions while maintaining its center of mass within safe boundaries.