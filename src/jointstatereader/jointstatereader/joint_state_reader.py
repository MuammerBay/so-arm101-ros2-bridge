#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import serial
import time
import struct
import math
import numpy as np

class JointStateReader(Node):
    def __init__(self):
        super().__init__('joint_state_reader')
        
        # Publisher for SO100 robot joint states - Hardware Driver
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # NEW: Publisher for pose deltas (Twist) - IK Teleoperation
        self.twist_pub = self.create_publisher(Twist, '/robot/cmd_pose', 10)
        
        # Joint names for SO100 robot (matching Isaac Lab convention)
        self.joint_names = [
            'Rotation',      # Base rotation
            'Pitch',         # Shoulder pitch  
            'Elbow',         # Elbow
            'Wrist_Pitch',   # Wrist pitch
            'Wrist_Roll',    # Wrist roll
            'Jaw'            # Gripper
        ]
        
        # NEW: SO-ARM101 DH parameters for forward kinematics
        # These are approximate - adjust based on your robot's actual dimensions
        self.dh_params = [
            # [a, alpha, d, theta_offset] - standard DH parameters
            [0.0, -math.pi/2, 0.0, 0.0],      # Base to shoulder
            [0.0, math.pi/2, 0.0, 0.0],       # Shoulder to upper arm
            [0.2, 0.0, 0.0, 0.0],             # Upper arm to forearm
            [0.0, -math.pi/2, 0.0, 0.0],      # Forearm to wrist
            [0.0, math.pi/2, 0.0, 0.0],       # Wrist to gripper
        ]
        
        # NEW: Previous joint positions for delta calculation
        self.prev_joint_positions = [0.0] * len(self.joint_names)
        self.prev_ee_pose = None
        
        # NEW: Pose delta calculation parameters
        self.pose_delta_scale = 0.1  # Scale factor for pose deltas
        self.min_delta_threshold = 0.001  # Minimum change to publish
        
        # Connect to SO100 robot hardware
        self.serial_port = None
        self.connect_to_robot()
        
        # Timer to read and publish joint states at 20Hz (more stable)
        self.timer = self.create_timer(0.05, self.read_and_publish)  # 20Hz
        
        # Cache for faster processing and change detection
        self.last_positions = [0.0] * len(self.joint_names)
        self.last_raw_ticks = [2048] * len(self.joint_names)  # Center position
        
        # Error tracking
        self.read_errors = [0] * len(self.joint_names)
        self.total_reads = 0
        self.consecutive_errors = 0
        
        # Movement detection (reduced sensitivity)
        self.movement_threshold = 0.05  # radians (~3 degrees)
        
        self.get_logger().info("🚀 SO100 Hardware Driver started - publishing to /joint_states at 20Hz")
        self.get_logger().info("✅ Direct hardware interface - reads from physical robot")
        self.get_logger().info("🔄 NEW: IK teleoperation support - publishing to /robot/cmd_pose")
        self.get_logger().info("Using official STS3215 protocol with proper error handling")
        
    def connect_to_robot(self):
        """Connect to the SO100 robot hardware"""
        try:
            # Connect to SO100 robot with optimized timeout for stability
            self.serial_port = serial.Serial('/dev/ttyACM0', 1000000, timeout=0.1)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            time.sleep(0.1)  # Allow port to stabilize
            self.get_logger().info("Connected to SO100 robot on /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to robot: {e}")
            self.serial_port = None
    
    def read_servo_position(self, servo_id):
        """Read position from STS3215 servo using official protocol"""
        if not self.serial_port:
            return None
            
        try:
            # Official STS3215 position read command
            # Read 2 bytes from PRESENT_POSITION_L (0x38)
            length = 4
            instruction = 0x02  # Read data instruction
            address = 0x38      # PRESENT_POSITION_L register
            read_length = 0x02  # Read 2 bytes (position is 16-bit)
            
            # Calculate checksum
            checksum = (~(servo_id + length + instruction + address + read_length)) & 0xFF
            
            # Build command packet
            cmd = bytes([0xFF, 0xFF, servo_id, length, instruction, address, read_length, checksum])
            
            # Clear buffers before communication
            self.serial_port.reset_input_buffer()
            self.serial_port.write(cmd)
            
            # Wait for response (8 bytes expected)
            time.sleep(0.002)  # Small delay for servo response
            response = self.serial_port.read(8)
            
            if len(response) >= 7:
                # Validate response header
                if response[0] != 0xFF or response[1] != 0xFF:
                    return None
                    
                # Validate servo ID
                if response[2] != servo_id:
                    return None
                    
                # Extract position from response bytes 5-6 (little endian)
                pos = struct.unpack('<H', response[5:7])[0]
                
                # Validate position range (0-4095 for STS3215)
                if 0 <= pos <= 4095:
                    return pos
                else:
                    self.get_logger().debug(f"Servo {servo_id}: Invalid position {pos} (out of range 0-4095)")
                    return None
            else:
                self.get_logger().debug(f"Servo {servo_id}: Short response ({len(response)} bytes)")
                return None
                
        except Exception as e:
            # Track communication errors
            self.read_errors[servo_id - 1] += 1
            if self.read_errors[servo_id - 1] % 50 == 1:  # Log every 50th error
                self.get_logger().warn(f"Servo {servo_id}: Communication error #{self.read_errors[servo_id - 1]}: {e}")
        
        return None
    
    def ticks_to_radians(self, ticks, joint_idx):
        """Convert servo ticks (0-4095) to radians (-π to π)"""
        if ticks is None:
            return self.last_positions[joint_idx]  # Use last known position
        
        # Convert to normalized position (-1 to 1)
        normalized = (ticks - 2048) / 2048.0
        
        # Convert to radians (-π to π)
        return normalized * 3.14159
    
    def dh_transform_matrix(self, a, alpha, d, theta):
        """Compute DH transformation matrix"""
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, joint_positions):
        """Compute end-effector pose from joint positions"""
        # Use only first 5 joints (exclude gripper)
        joints = joint_positions[:5]
        
        # Start with identity matrix
        T = np.eye(4)
        
        # Apply DH transformations for each joint
        for i, (joint_angle, dh_param) in enumerate(zip(joints, self.dh_params)):
            a, alpha, d, theta_offset = dh_param
            theta = joint_angle + theta_offset
            T_i = self.dh_transform_matrix(a, alpha, d, theta)
            T = T @ T_i
        
        # Extract position and orientation
        position = T[:3, 3]
        
        # Convert rotation matrix to Euler angles (simplified)
        # This is a simplified approach - for more accuracy, use proper quaternion conversion
        rx = math.atan2(T[2, 1], T[2, 2])
        ry = math.atan2(-T[2, 0], math.sqrt(T[2, 1]**2 + T[2, 2]**2))
        rz = math.atan2(T[1, 0], T[0, 0])
        
        return position, [rx, ry, rz]
    
    def compute_pose_delta(self, current_joints, prev_joints):
        """Compute pose delta from joint position changes"""
        if prev_joints is None:
            return None
            
        # Compute current and previous end-effector poses
        current_pos, current_rot = self.forward_kinematics(current_joints)
        prev_pos, prev_rot = self.forward_kinematics(prev_joints)
        
        # Compute position delta
        pos_delta = current_pos - prev_pos
        
        # Compute rotation delta (simplified)
        rot_delta = [current_rot[i] - prev_rot[i] for i in range(3)]
        
        # Normalize rotation deltas to [-π, π]
        for i in range(3):
            while rot_delta[i] > math.pi:
                rot_delta[i] -= 2 * math.pi
            while rot_delta[i] < -math.pi:
                rot_delta[i] += 2 * math.pi
        
        return pos_delta, rot_delta
    
    def read_and_publish(self):
        """Read all joint positions and publish as JointState and Twist"""
        if not self.serial_port:
            # Try to reconnect
            self.connect_to_robot()
            return
            
        self.total_reads += 1
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = []
        
        new_raw_ticks = []
        successful_reads = 0
        
        # Read each servo (IDs 1-6) with proper delays
        for i in range(len(self.joint_names)):
            servo_id = i + 1
            ticks = self.read_servo_position(servo_id)
            radians = self.ticks_to_radians(ticks, i)
            msg.position.append(radians)
            
            # Store raw ticks for debugging
            if ticks is not None:
                new_raw_ticks.append(ticks)
                successful_reads += 1
            else:
                new_raw_ticks.append(self.last_raw_ticks[i])
            
            # Proper delay between servo reads (official driver uses 10ms)
            time.sleep(0.01)
        
        # Track consecutive errors
        if successful_reads == 0:
            self.consecutive_errors += 1
            if self.consecutive_errors > 10:
                self.get_logger().error("Too many consecutive read failures - attempting reconnection")
                self.serial_port = None
                self.consecutive_errors = 0
                return
        else:
            self.consecutive_errors = 0
        
        # Update cache
        self.last_positions = list(msg.position)
        self.last_raw_ticks = new_raw_ticks.copy()
        
        # Publish joint states to ROS2 /joint_states topic
        self.joint_pub.publish(msg)
        
        # NEW: Compute and publish pose deltas for IK teleoperation
        if self.prev_joint_positions is not None:
            pose_delta = self.compute_pose_delta(msg.position, self.prev_joint_positions)
            
            if pose_delta is not None:
                pos_delta, rot_delta = pose_delta
                
                # Check if delta is significant enough to publish
                pos_magnitude = np.linalg.norm(pos_delta)
                rot_magnitude = np.linalg.norm(rot_delta)
                
                if pos_magnitude > self.min_delta_threshold or rot_magnitude > self.min_delta_threshold:
                    # Create Twist message
                    twist_msg = Twist()
                    twist_msg.linear.x = pos_delta[0] * self.pose_delta_scale
                    twist_msg.linear.y = pos_delta[1] * self.pose_delta_scale
                    twist_msg.linear.z = pos_delta[2] * self.pose_delta_scale
                    twist_msg.angular.x = rot_delta[0] * self.pose_delta_scale
                    twist_msg.angular.y = rot_delta[1] * self.pose_delta_scale
                    twist_msg.angular.z = rot_delta[2] * self.pose_delta_scale
                    
                    # Publish Twist for IK teleoperation
                    self.twist_pub.publish(twist_msg)
        
        # Update previous joint positions for next iteration
        self.prev_joint_positions = list(msg.position)
        
        # Status logging every 5 seconds
        if self.total_reads % 100 == 0:  # Every 5 seconds at 20Hz
            pos_str = [f'{p:.3f}' for p in msg.position]
            tick_str = [f'{t}' for t in new_raw_ticks]
            error_str = [f'{e}' for e in self.read_errors]
            
            self.get_logger().info(f"=== Status Report (Read #{self.total_reads}) ===")
            self.get_logger().info(f"📡 Publishing to /joint_states")
            self.get_logger().info(f"🔄 Publishing to /robot/cmd_pose (IK teleop)")
            self.get_logger().info(f"Joint positions (rad): {pos_str}")
            self.get_logger().info(f"Raw servo ticks: {tick_str}")
            self.get_logger().info(f"Read errors per servo: {error_str}")
            self.get_logger().info(f"Successful reads: {successful_reads}/{len(self.joint_names)}")

def main():
    rclpy.init()
    
    try:
        reader = JointStateReader()
        rclpy.spin(reader)
    except KeyboardInterrupt:
        print("\nShutting down joint state reader...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 