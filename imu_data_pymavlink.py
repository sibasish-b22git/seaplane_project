#!/usr/bin/env python3
import math
import time
from pymavlink import mavutil

class PymavlinkServoController:
    def __init__(self, connection_string, baud_rate=115200):
        # --- CONFIGURATION ---
        self.PITCH_THRESHOLD = 15.0          # Degrees
        self.SERVO_CENTER_PWM = 1500.0       # Neutral position
        self.OFFSET_MULTIPLIER = 10.0        # How aggressively the servo moves
        self.SERVO_PINS = [5, 6]             # Crossflight pins to control
        # ---------------------
        
        self.last_pwm = self.SERVO_CENTER_PWM
        
        print(f"Connecting to Flight Controller on {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
        
        # Wait for the first heartbeat to establish system/component IDs
        self.master.wait_heartbeat()
        print(f"Heartbeat received from System {self.master.target_system}, Component {self.master.target_component}!")
        print(f"Controller active. Threshold set to {self.PITCH_THRESHOLD}°")
        
        # Request data stream for ATTITUDE (if not already streaming automatically)
        self.request_message_stream(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, rate_hz=20)

    def request_message_stream(self, msg_id, rate_hz):
        """Requests the flight controller to stream a specific message at a target frequency."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,                     # param1: Message ID
            int(1000000 / rate_hz),     # param2: Interval in microseconds
            0, 0, 0, 0, 0               # param3-7: Unused
        )

    def send_servo_command(self, pin, pwm):
        """Sends MAV_CMD_DO_SET_SERVO to move a specific hardware pin."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,              # Confirmation
            int(pin),       # param1: Servo pin number
            int(pwm),       # param2: PWM value (1000 to 2000)
            0, 0, 0, 0, 0   # param3-7: Unused
        )

    def run(self):
        """Main loop that listens for IMU/Attitude messages and updates servos."""
        try:
            while True:
                # Wait and grab only the ATTITUDE message from the stream
                msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
                
                if not msg:
                    continue
                
                # MAVLink ATTITUDE pitch is given directly in radians
                pitch_deg = math.degrees(msg.pitch)
                print(f"Current Pitch: {pitch_deg:>7.2f}°")
                
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
                
                # Check if change is significant (> 2 PWM) to avoid flooding the UART line
                if abs(target_pwm - self.last_pwm) > 2.0:
                    for pin in self.SERVO_PINS:
                        self.send_servo_command(pin, target_pwm)
                    
                    self.last_pwm = target_pwm
                    print(f">> Servos clamped to PWM: {int(target_pwm)}")
                    
        except KeyboardInterrupt:
            print("\nShutting down servo controller safely...")
        finally:
            self.master.close()

if __name__ == '__main__':
    # Change '/dev/ttyACM0' to your actual serial port or UDP stream (e.g., 'udp:localhost:14550')
    CONNECTION_PORT = '/dev/ttyACM0'
    BAUD_RATE = 115200
    
    controller = PymavlinkServoController(CONNECTION_PORT, BAUD_RATE)
    controller.run()
