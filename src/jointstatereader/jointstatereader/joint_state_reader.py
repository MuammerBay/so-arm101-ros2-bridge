#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import struct
import math

class JointStateReader(Node):
    def __init__(self):
        super().__init__('joint_state_reader')
        
        # Publisher for SO100 robot joint states - Hardware Driver
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Joint names for SO100 robot (matching Isaac Lab convention)
        self.joint_names = [
            'Rotation',      # Base rotation
            'Pitch',         # Shoulder pitch  
            'Elbow',         # Elbow
            'Wrist_Pitch',   # Wrist pitch
            'Wrist_Roll',    # Wrist roll
            'Jaw'            # Gripper
        ]
        
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
    
    def read_and_publish(self):
        """Read all joint positions and publish as JointState"""
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
        
        # Status logging every 5 seconds
        if self.total_reads % 100 == 0:  # Every 5 seconds at 20Hz
            pos_str = [f'{p:.3f}' for p in msg.position]
            tick_str = [f'{t}' for t in new_raw_ticks]
            error_str = [f'{e}' for e in self.read_errors]
            
            self.get_logger().info(f"=== Status Report (Read #{self.total_reads}) ===")
            self.get_logger().info(f"📡 Publishing to /joint_states")
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