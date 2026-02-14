#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math

# --- STATES ---
STATE_SEARCH = 0      
STATE_APPROACH = 1    
STATE_ALIGNING = 2    
STATE_CLIMBING = 3    
STATE_FINISHED = 4

class PrecisionLevelManager(Node):
    def __init__(self):
        super().__init__('precision_level_manager')
        self.get_logger().info('--- R2KRISHNA: DARK BLOCK ISOLATION & ANTI-SHAKE ---')

        # --- STABILITY SETTINGS ---
        self.linear_speed = 0.20       
        self.search_speed = 0.20       # Slower search prevents overshooting
        self.steering_gain = 0.0045    # Extremely low gain to stop all shaking
        self.deadzone = 25             # Center-point tolerance in pixels
        self.smoothing = 0.92          # Heavy smoothing for fluid movement
        self.prev_turn = 0.0
        
        # --- SEQUENCE CONFIG ---
        self.objectives = [200, 400, 600]
        self.current_obj_idx = 0
        self.state = STATE_SEARCH
        
        self.br = CvBridge()
        self.block_x = 0
        self.valid_target_locked = False 
        self.lidar_min_dist = 99.9
        self.lidar_error = 0.0
        self.current_pitch = 0.0
        self.align_start_time = 0.0

        # Subscriptions
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publishers
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_front = self.create_publisher(Twist, '/cmd_vel_front', 10)

        self.create_timer(0.05, self.control_loop)
        cv2.namedWindow("Robot Vision", cv2.WINDOW_NORMAL)

    def get_hsv_range(self):
        """Strictly isolated ranges based on block brightness (Value)."""
        obj = self.objectives[self.current_obj_idx]
        
        if obj == 200: 
            # 200mm: Dark Olive Green (Diffuse 0.16 0.32 0.06) 
            # Value (Brightness) capped at 90 to ignore bright 600mm blocks.
            return np.array([30, 40, 5]), np.array([80, 255, 90])
        
        elif obj == 400: 
            # 400mm: Forest Green (Diffuse 0.165 0.44 0.22) [cite: 13]
            # Your provided values: 49, 80, 57
            return np.array([20, 60, 40]), np.array([35, 255, 150])
            
        elif obj == 600: 
            # 600mm: Bright Yellow-Green (Diffuse 0.60 0.65 0.31) 
            # Your provided values: 93, 96, 67. Value set high (100+).
            return np.array([40, 70, 100]), np.array([60, 255, 255])
            
        return np.array([0,0,0]), np.array([180,255,255])

    def imu_callback(self, msg):
        q = msg.orientation
        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.current_pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        self.lidar_min_dist = np.min(ranges) if len(ranges) > 0 else 99.9
        mid = len(ranges) // 2
        # Alignment check [cite: 301]
        self.lidar_error = np.mean(ranges[mid+5:mid+25]) - np.mean(ranges[mid-25:mid-5])

    def image_callback(self, msg):
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            lower, upper = self.get_hsv_range()
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            self.valid_target_locked = False
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 300:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        self.block_x = int(M["m10"] / M["m00"])
                        self.valid_target_locked = True
                        cv2.circle(frame, (self.block_x, 240), 12, (0, 255, 0), -1)

            status = f"TARGET: {self.objectives[self.current_obj_idx]}mm | STATE: {self.state}"
            cv2.putText(frame, status, (15, 35), 0, 0.7, (255, 255, 255), 2)
            cv2.imshow("Robot Vision", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Perception Error: {e}")

    def drive(self, fwd, rot, engage_6wd=False):
        # Heavy Smoothing filter to eliminate shaking
        actual_rot = (self.smoothing * self.prev_turn) + ((1 - self.smoothing) * rot)
        self.prev_turn = actual_rot

        cmd = Twist()
        cmd.linear.x = float(fwd)
        cmd.angular.z = float(actual_rot)
        self.pub_vel.publish(cmd)
        if engage_6wd:
            self.pub_front.publish(cmd)
        else:
            self.pub_front.publish(Twist())

    def control_loop(self):
        if self.state == STATE_FINISHED:
            self.drive(0, 0); return

        if self.state == STATE_SEARCH:
            if self.valid_target_locked:
                self.state = STATE_APPROACH
            else:
                self.drive(0.0, self.search_speed)

        elif self.state == STATE_APPROACH:
            if not self.valid_target_locked:
                self.state = STATE_SEARCH; return
            
            err = 320 - self.block_x
            if abs(err) < self.deadzone:
                turn = 0.0
            else:
                turn = self.steering_gain * err * -1
            
            if self.lidar_min_dist < 0.40:
                self.state = STATE_ALIGNING
                self.align_start_time = time.time()
                self.get_logger().info("Target reached. Finalizing alignment...")
            else:
                self.drive(self.linear_speed, turn)

        elif self.state == STATE_ALIGNING:
            # 8 second settle time for precise climbing alignment
            if time.time() - self.align_start_time > 8.0:
                self.state = STATE_CLIMBING
                self.get_logger().info("Starting 6WD Climb.")
            else:
                # Square up using lidar error
                self.drive(0.0, self.lidar_error * 3.5)

        elif self.state == STATE_CLIMBING:
            self.drive(0.85, 0.0, engage_6wd=True)
            if abs(self.current_pitch) < 0.03 and self.lidar_min_dist > 2.0:
                self.current_obj_idx += 1
                if self.current_obj_idx >= 3:
                    self.state = STATE_FINISHED
                    self.get_logger().info("ALL LEVELS COMPLETE")
                else:
                    self.state = STATE_SEARCH

if __name__ == '__main__':
    rclpy.init()
    node = PrecisionLevelManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
