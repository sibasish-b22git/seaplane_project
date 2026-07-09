#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandLong
from rclpy.qos import qos_profile_sensor_data

class ImuServoController(Node):
    def __init__(self):
        super().__init__('imu_servo_controller')
        
        # --- CONFIGURATION (Change these instead of hardcoding in logic) ---
        self.PITCH_THRESHOLD = 15.0          # Degrees
        self.SERVO_CENTER_PWM = 1500.0       # Neutral position
        self.OFFSET_MULTIPLIER = 10.0        # How aggressively the servo moves
        # ------------------------------------------------------------------
        
        self.cli = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MAVROS command service...')
        
        self.subscription = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, qos_profile_sensor_data
        )
        
        self.last_pwm = self.SERVO_CENTER_PWM
        self.get_logger().info(f'Controller active. Threshold set to {self.PITCH_THRESHOLD}°')

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        _, pitch, _ = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        pitch_deg = math.degrees(pitch)
        
        self.get_logger().info(f'Current Pitch: {pitch_deg:>7.2f}°')
        
        target_pwm = self.SERVO_CENTER_PWM
        
        # Calculate offset logic
        if pitch_deg > self.PITCH_THRESHOLD:
            offset = pitch_deg - self.PITCH_THRESHOLD
            target_pwm = self.SERVO_CENTER_PWM - (offset * self.OFFSET_MULTIPLIER)
        elif pitch_deg < -self.PITCH_THRESHOLD:
            offset = pitch_deg + self.PITCH_THRESHOLD
            target_pwm = self.SERVO_CENTER_PWM - (offset * self.OFFSET_MULTIPLIER)
        
        # --- THE SAFETY CLAMP ---
        # Forces the value to stay between 1000 and 2000
        target_pwm = max(1000.0, min(2000.0, target_pwm))
            
        if abs(target_pwm - self.last_pwm) > 2.0:
            self.send_servo_command(5, target_pwm)
            self.send_servo_command(6, target_pwm)
            self.last_pwm = target_pwm
            self.get_logger().info(f'>> Servo clamped to PWM: {target_pwm}')

    def send_servo_command(self, pin, pwm):
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(pin)
        req.param2 = float(pwm)
        self.cli.call_async(req)

    def quaternion_to_euler(self, x, y, z, w):
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2.0, sinp)
        return 0.0, pitch, 0.0 

def main(args=None):
    rclpy.init(args=args)
    node = ImuServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()