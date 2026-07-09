#!/usr/bin/env python3  # Shebang for Python 3 execution in Linux terminal
import math             # Math library for trig, degrees/radians, and distance math
import rclpy            # Core ROS 2 client library for runtime communication
import subprocess       # Execute terminal commands for Gazebo service calls
from rclpy.node import Node  # Base class to define our ROS 2 autopilot node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # Prevent thread blocking during service calls
from rclpy.executors import MultiThreadedExecutor                 # Handle topics, timers, and services concurrently
from rclpy.qos import qos_profile_sensor_data                     # Low-latency QoS profile for sensor data

from rcl_interfaces.msg import ParameterValue, ParameterType  # Data types for setting MAVROS parameters
from mavros_msgs.msg import State, AttitudeTarget             # Autopilot state and attitude target messages
from mavros_msgs.srv import CommandBool, SetMode, CommandInt, ParamSetV2  # MAVROS service definitions
from sensor_msgs.msg import NavSatFix, LaserScan              # GPS coordinate and LiDAR scan messages

ORIGIN_LAT = -35.363262  # Gazebo world origin latitude in simulation environment
ORIGIN_LON = 149.165237  # Gazebo world origin longitude in simulation environment

RACETRACK_WAYPOINTS = []  # List to store dynamically generated figure-8 local waypoints
for i in range(48):       # Generate 48 waypoints spaced roughly 30m apart along track
    t = i * (2.0 * math.pi / 48.0)                 # Parametric angle 't' for current waypoint step
    x = 300.0 * math.sin(t)                        # Scale X coordinate by 300m along horizontal axis
    y = 200.0 + 200.0 * math.sin(t) * math.cos(t)  # Scale and offset Y coordinate for figure-8 intersection
    RACETRACK_WAYPOINTS.append((round(x, 1), round(y, 1)))  # Append rounded (X, Y) tuple to waypoint list

DANGER_DISTANCE   = 30.0  # Max LiDAR detection distance in meters; triggers evasion
MAX_BANK_ANGLE    = 35.0  # Max roll/bank angle in degrees commanded during obstacle evasion
STEER_THRESHOLD   = 2.0   # Noise filter threshold; steering commands under 2° are ignored

CESSNA_TARGET_ALT_CRUISE = 1.0  # Target cruise altitude in meters (relative to home)

CESSNA_PARAMS = {            # Dictionary of ArduPilot parameters configured before flight
    'TKOFF_ALT': 3.0,        # Target climb altitude (in meters) for auto-takeoff mode
    'RUDDER_ONLY': 0,        # Disable rudder-only mode (Cessna uses ailerons for turns)
    'SERVO1_FUNCTION': 4,    # Assign Servo Pin 1 as Aileron control surface output
    'TKOFF_THR_MAX': 57.0,   # Limit takeoff throttle to 57% to prevent excessive pitch-up
    'THR_MAX': 40.0,         # Limit max cruise throttle to 40% for stable airspeed
    'AIRSPEED_MIN': 30.0,    # Minimum safe airspeed before stall warning triggers
    'AIRSPEED_MAX': 50.0,    # Maximum allowed airspeed to prevent structural overspeed
    'AIRSPEED_CRUISE': 4000, # Target cruise airspeed (4000 cm/s = 40 m/s)
    'WP_RADIUS': 25.0,       # Acceptance radius (in meters) to consider waypoint reached
    'WP_LOITER_RAD': 5.0     # Radius (in meters) for loiter/orbit holding circles
}                            # End of parameter dictionary configuration

WAYPOINT_RADIUS   = CESSNA_PARAMS['WP_RADIUS']  # Set active acceptance radius from parameter dictionary


class PlaneAutopilot(Node):  # Main ROS 2 node for Cessna takeoff, navigation, and avoidance
    def __init__(self):      # Initialize node, subscriptions, services, and timers
        super().__init__('plane_autopilot')                   # Initialize node with name 'plane_autopilot'
        self._timer_cbg   = MutuallyExclusiveCallbackGroup()  # Dedicated callback group for loop timers
        self._service_cbg = MutuallyExclusiveCallbackGroup()  # Separate callback group for async service calls
        self.state        = State()                           # Store latest MAVROS system state
        self.current_lat  = None                              # Store current GPS latitude
        self.current_lon  = None                              # Store current GPS longitude
        self.current_alt  = None                              # Store current GPS altitude (AMSL)
        self.home_alt     = None                              # Lock in starting altitude upon first GPS fix
        self.phase        = 'WAIT_CONNECTION'                 # Initial phase: wait for MAVROS heartbeat
        self._pending     = False                             # Track active service calls to prevent command spam
        self.deck_deployed = False                            # Ensure invisible deck is only spawned once
        self.params_to_set = []                               # List of parameter key-value pairs to transmit
        self.current_param_idx = 0                            # Index tracker for parameter transmission list
        self.current_wp_idx = 0                               # Index tracker for active target waypoint
        self.target_global_lat = None                         # Target GPS latitude of current waypoint
        self.target_global_lon = None                         # Target GPS longitude of current waypoint
        self.create_subscription(                             # Subscribe to MAVROS system state updates
            State, '/mavros/state',                           # Topic name and message type for state
            self._state_cb, qos_profile_sensor_data)          # Callback handler and high-frequency QoS profile
        self.create_subscription(                             # Subscribe to global GPS position updates
            NavSatFix, '/mavros/global_position/global',      # Topic name and message type for GPS
            self._gps_cb, qos_profile_sensor_data)            # Callback handler and high-frequency QoS profile
        self.create_subscription(                             # Subscribe to 2D LiDAR scan rays for avoidance
            LaserScan, '/scan',                               # Topic name and message type for LiDAR
            self._obstacle_cb, qos_profile_sensor_data)       # Callback handler and high-frequency QoS profile
        self.arm_client       = self.create_client(           # Client to command arming/disarming of motors
            CommandBool, '/mavros/cmd/arming',                # MAVROS arming service name and message definition
            callback_group=self._service_cbg)                 # Assign to service callback group
        self.mode_client      = self.create_client(           # Client to change ArduPilot flight modes
            SetMode, '/mavros/set_mode',                      # MAVROS mode switch service name and definition
            callback_group=self._service_cbg)                 # Assign to service callback group
        self.repo_client      = self.create_client(           # Client for MAVLink command_int repositioning
            CommandInt, '/mavros/cmd/command_int',            # MAVROS command_int service name and definition
            callback_group=self._service_cbg)                 # Assign to service callback group
        self.param_set_client = self.create_client(           # Client to modify ArduPilot parameters over MAVLink
            ParamSetV2, '/mavros/param/set',                  # MAVROS parameter service name and definition
            callback_group=self._service_cbg)                 # Assign to service callback group
        self.attitude_pub = self.create_publisher(            # Publisher for raw attitude targets in GUIDED mode
            AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)  # Topic name, message type, queue size 10
        self.create_timer(0.1, self._loop, callback_group=self._timer_cbg)  # 10Hz control loop timer (every 0.1s)
        self.get_logger().info('Flight Commander started.')   # Log confirmation that autopilot node initialized

    def _state_cb(self, msg: State):  # Callback function updating internal state from MAVROS messages
        self.state = msg              # Store latest MAVROS state message

    def _gps_cb(self, msg: NavSatFix):            # Callback capturing real-time GPS coordinates and altitude
        self.current_lat = msg.latitude           # Update current latitude from GPS
        self.current_lon = msg.longitude          # Update current longitude from GPS
        self.current_alt = msg.altitude           # Update current altitude (AMSL)
        if self.home_alt is None and msg.altitude > 0:  # Lock initial altitude as ground elevation reference
            self.home_alt = msg.altitude                # Save initial altitude as home ground reference
            self.get_logger().info(f'Home alt locked: {self.home_alt:.1f}m AMSL')  # Log locked home altitude

    def _local_to_global(self, x_meters, y_meters):  # Convert Gazebo X/Y Cartesian meters into Global GPS Lat/Lon
        R_earth = 6378137.0                          # Approximate Earth radius in meters
        d_lat = y_meters / R_earth                   # Latitude offset in radians (North/South movement)
        d_lon = x_meters / (R_earth * math.cos(math.radians(ORIGIN_LAT)))  # Longitude offset in radians
        target_lat = ORIGIN_LAT + math.degrees(d_lat)  # Convert radian offset to degrees and add to origin lat
        target_lon = ORIGIN_LON + math.degrees(d_lon)  # Convert radian offset to degrees and add to origin lon
        return target_lat, target_lon                  # Return finalized GPS coordinates tuple

    def _calculate_steering(self, msg: LaserScan) -> float:  # Analyze LiDAR rays to get bank angle (+ Right, - Left)
        num_rays = len(msg.ranges)                           # Get total number of distance rays in LiDAR scan
        if num_rays == 0:                                    # If scan array is empty, no steering required
            return 0.0                                       # Return zero degrees bank angle
        center_idx     = num_rays // 2                       # Array index representing dead-center forward vision
        count_left     = 0                                   # Counter for obstacles detected on left side
        count_right    = 0                                   # Counter for obstacles detected on right side
        min_dist_left  = float('inf')                        # Shortest distance to an obstacle on left side
        min_dist_right = float('inf')                        # Shortest distance to an obstacle on right side
        idx_min_left   = center_idx                          # Ray index of closest obstacle on left
        idx_min_right  = center_idx                          # Ray index of closest obstacle on right
        for i, dist in enumerate(msg.ranges):                # Loop through every ray in LiDAR scan array
            if math.isinf(dist) or math.isnan(dist) or dist > DANGER_DISTANCE:  # Ignore invalid or distant rays
                continue                                     # Skip to next ray
            if i < center_idx:                               # Check if ray is on left half of forward FOV
                count_left += 1                              # Increment left-side obstacle count
                if dist < min_dist_left:                     # Check if ray is closer than previous left detections
                    min_dist_left = dist                     # Update shortest distance on left side
                    idx_min_left  = i                        # Record ray index of closest left obstacle
            else:                                            # Otherwise, ray is on right half of FOV
                count_right += 1                             # Increment right-side obstacle count
                if dist < min_dist_right:                    # Check if ray is closer than previous right detections
                    min_dist_right = dist                    # Update shortest distance on right side
                    idx_min_right  = i                       # Record ray index of closest right obstacle
        if count_left == 0 and count_right == 0:             # If no obstacles detected within danger distance
            return 0.0                                       # Fly straight with zero bank angle
        def steer_mag(target_idx: int) -> float:             # Calculate bank angle based on obstacle proximity to center
            dist_from_center = abs(target_idx - center_idx)  # Ray indices away from dead-center
            weight = 1.0 - (dist_from_center / max(center_idx, 1))  # Severity weight from 1.0 (center) to 0.0 (edge)
            return weight * MAX_BANK_ANGLE                   # Multiply weight by maximum allowed bank angle (35°)
        if count_left > 0 and count_right == 0:              # If obstacles exist only on left, bank right
            return steer_mag(idx_min_left)                   # Return positive bank angle away from left threat
        elif count_right > 0 and count_left == 0:            # If obstacles exist only on right, bank left
            return -steer_mag(idx_min_right)                 # Return negative bank angle away from right threat
        else:                                                # If obstacles detected on BOTH sides simultaneously
            if count_left > count_right:                     # If more obstacle points on left, steer right
                return steer_mag(idx_min_left)               # Return positive bank angle away from left threat
            elif count_right > count_left:                   # If more obstacle points on right, steer left
                return -steer_mag(idx_min_right)             # Return negative bank angle away from right threat
            else:                                            # If both sides equally blocked, default hard left
                self.get_logger().warn(                      # Log warning message about symmetric obstacle
                    'Symmetric obstacle — forcing LEFT evasion!')  # Console warning text for forced left evasion
                return -steer_mag(idx_min_right)             # Return negative bank angle to evade left

    def _send_roll_command(self, roll_deg: float):  # Command target bank angle (roll) to ailerons in GUIDED mode
        roll_rad  = math.radians(-roll_deg)         # Convert degrees to radians (inverted sign for ArduPilot frame)
        pitch_rad = 0.0                             # Keep pitch neutral (0.0 radians) during evasion
        yaw_rad   = 0.0                             # Keep yaw rate neutral (0.0 radians) during evasion
        cy = math.cos(yaw_rad   * 0.5)              # Cosine of half yaw for quaternion conversion
        sy = math.sin(yaw_rad   * 0.5)              # Sine of half yaw for quaternion conversion
        cp = math.cos(pitch_rad * 0.5)              # Cosine of half pitch for quaternion conversion
        sp = math.sin(pitch_rad * 0.5)              # Sine of half pitch for quaternion conversion
        cr = math.cos(roll_rad  * 0.5)              # Cosine of half roll for quaternion conversion
        sr = math.sin(roll_rad  * 0.5)              # Sine of half roll for quaternion conversion
        msg = AttitudeTarget()                      # Initialize raw attitude target ROS 2 message
        msg.header.stamp    = self.get_clock().now().to_msg()  # Stamp message with current system clock time
        msg.header.frame_id = 'base_link'                      # Specify body-relative coordinate frame ('base_link')
        msg.type_mask = (                                      # Ignore rates and throttle; command orientation only
            AttitudeTarget.IGNORE_ROLL_RATE  |                 # Ignore body roll rate setpoint
            AttitudeTarget.IGNORE_PITCH_RATE |                 # Ignore body pitch rate setpoint
            AttitudeTarget.IGNORE_YAW_RATE   |                 # Ignore body yaw rate setpoint
            AttitudeTarget.IGNORE_THRUST                       # Ignore throttle/thrust setpoint
        )                                                      # End of type mask configuration
        msg.orientation.w = cr * cp * cy + sr * sp * sy        # Quaternion w component from Euler angles
        msg.orientation.x = sr * cp * cy - cr * sp * sy        # Quaternion x component from Euler angles
        msg.orientation.y = cr * sp * cy + sr * cp * sy        # Quaternion y component from Euler angles
        msg.orientation.z = cr * cp * sy - sr * sp * cy        # Quaternion z component from Euler angles
        self.attitude_pub.publish(msg)                         # Publish formatted attitude command to MAVROS

    def _obstacle_cb(self, msg: LaserScan):              # Analyze LiDAR scans to override path for obstacles
        if self.phase not in ['GOTO_WAYPOINT', 'EVADING']:  # Only run avoidance if navigating or evading
            return                                       # Ignore scan during takeoff, landing, or setup
        steering = self._calculate_steering(msg)         # Calculate required evasive bank angle from LiDAR scan
        if abs(steering) > STEER_THRESHOLD:              # If steering exceeds noise threshold (2°), evade
            if self.phase != 'EVADING':                  # If previously flying normally, switch to EVADING
                self.get_logger().warn(                  # Log warning message that obstacle was detected
                    f'OBSTACLE DETECTED — steering={steering:.1f}°')  # Include calculated steering angle in log
                self.phase = 'EVADING'                   # Switch active state machine phase to EVADING
            self._send_roll_command(steering)            # Command Cessna ailerons to bank by calculated angle
            self.get_logger().info(                      # Log active roll command to terminal
                f'Roll cmd: {steering:.1f}°',            # Format roll command text
                throttle_duration_sec=0.5)               # Throttle console print to once every 0.5 seconds
        elif self.phase == 'EVADING':                    # If steering dropped below threshold while EVADING
            self.get_logger().info('Path clear — wings level, resuming mission.')  # Log clear path confirmation
            self._send_roll_command(0.0)                 # Command 0 roll (level wings) to stabilize aircraft
            self.phase = 'RESUME_MISSION'                # Switch to RESUME_MISSION phase to request next waypoint

    def _set_mode(self, mode: str, on_done):            # Asynchronously switch ArduPilot flight mode
        if self._pending or not self.mode_client.service_is_ready():  # Ignore if client busy or service offline
            return                                      # Exit function early without calling service
        self._pending = True                            # Lock service client to prevent command spam
        req = SetMode.Request()                         # Initialize flight mode service request message
        req.custom_mode = mode                          # Assign target custom mode string ('13' or 'GUIDED')
        f = self.mode_client.call_async(req)            # Send asynchronous service request to MAVROS
        def _cb(fut):                                   # Callback executed when autopilot responds
            self._pending = False                       # Unlock service client once response received
            try:                                        # Exception handling for service response evaluation
                ok = fut.result() and fut.result().mode_sent  # Check if flight controller accepted mode switch
                label = 'TAKEOFF' if mode == '13' else mode   # Translate numeric mode '13' to label 'TAKEOFF'
                self.get_logger().info(                       # Log mode switch confirmation or rejection
                    f'Mode → {label} ✅' if ok                 # Print success checkmark if accepted
                    else f'Mode {label} rejected')            # Print rejection warning if failed
                on_done(ok)                             # Trigger completion callback with boolean result
            except Exception:                           # Catch communication exceptions or timeouts
                on_done(False)                          # Trigger completion callback with False on failure
        f.add_done_callback(_cb)                        # Attach completion callback to future response

    def _arm(self, on_done):                           # Asynchronously command ArduPilot to ARM motors
        if self._pending or not self.arm_client.service_is_ready():  # Ignore if client busy or service offline
            return                                     # Exit function early without calling service
        self._pending = True                           # Lock service client to prevent command spam
        req = CommandBool.Request()                    # Initialize arming command request message
        req.value = True                               # Set arm value to True (arm motors)
        f = self.arm_client.call_async(req)            # Send asynchronous arming request to MAVROS
        def _cb(fut):                                  # Callback executed when autopilot responds
            self._pending = False                      # Unlock service client once response received
            try:                                       # Exception handling for service response evaluation
                ok = fut.result() and fut.result().success  # Check if flight controller successfully armed
                self.get_logger().info('Armed ✅' if ok else 'Arming failed')  # Log arming status to console
                on_done(ok)                            # Trigger completion callback with boolean result
            except Exception:                          # Catch communication exceptions or timeouts
                on_done(False)                         # Trigger completion callback with False on failure
        f.add_done_callback(_cb)                       # Attach completion callback to future response

    def _set_parameter(self, param_id: str, value, on_done):  # Asynchronously transmit parameter change to ArduPilot
        if self._pending or not self.param_set_client.service_is_ready():  # Ignore if busy or service offline
            return                                            # Exit function early without calling service
        self._pending = True                                  # Lock service client to prevent command spam
        req = ParamSetV2.Request()                            # Initialize parameter set service request
        req.param_id = param_id                               # Assign parameter string ID ('AIRSPEED_CRUISE')
        req.value = ParameterValue()                          # Initialize parameter value structure
        if isinstance(value, int):                            # Check if parameter value is an integer
            req.value.type          = ParameterType.PARAMETER_INTEGER  # Set parameter type flag to integer
            req.value.integer_value = value                            # Assign integer value to message field
        else:                                                 # Otherwise format parameter as floating double
            req.value.type         = ParameterType.PARAMETER_DOUBLE    # Set parameter type flag to double
            req.value.double_value = float(value)                      # Assign floating point value to message
        f = self.param_set_client.call_async(req)             # Transmit parameter asynchronously to autopilot
        def _cb(fut):                                         # Callback executed when autopilot responds
            self._pending = False                             # Unlock service client once response received
            try:                                              # Exception handling for service response evaluation
                res = fut.result()                            # Get result message from service call future
                ok  = res.success if res else False           # Verify parameter was saved successfully
                self.get_logger().info(                       # Log parameter modification status
                    f'Param {param_id}={value} ✅' if ok       # Print checkmark if parameter saved
                    else f'Param {param_id} failed')          # Print warning if parameter rejected
                on_done(ok)                                   # Trigger completion callback with boolean result
            except Exception as e:                            # Catch communication or formatting errors
                self.get_logger().error(f'Param error: {e}')  # Log error message if transmission failed
                on_done(False)                                # Trigger completion callback with False on failure
        f.add_done_callback(_cb)                              # Attach completion callback to future response

    def _send_reposition(self, on_done):                # Asynchronously command GPS reposition (MAV_CMD_DO_REPOSITION)
        if self._pending or not self.repo_client.service_is_ready():  # Ignore if busy or service offline
            return                                      # Exit function early without calling service
        self._pending = True                            # Lock service client to prevent command spam
        req = CommandInt.Request()                      # Initialize integer command request message
        req.broadcast    = False                        # Do not broadcast command to network vehicles
        req.frame        = 6                            # Altitude relative to home (GLOBAL_RELATIVE_ALT_INT)
        req.command      = 192                          # Specify command ID 192 (MAV_CMD_DO_REPOSITION)
        req.current      = 0                            # Current waypoint flag set to 0
        req.autocontinue = 0                            # Auto-continue flag set to 0
        req.param1       = -1.0                         # Transit speed: -1.0 instructs default cruise airspeed
        req.param2       = 0.0                          # Unused parameter 2 set to 0
        req.param3       = 0.0                          # Unused parameter 3 set to 0
        req.param4       = 0.0                          # Unused parameter 4 set to 0
        req.x            = int(self.target_global_lat * 10000000)  # Latitude in ArduPilot int format (deg * 10^7)
        req.y            = int(self.target_global_lon * 10000000)  # Longitude in ArduPilot int format (deg * 10^7)
        req.z            = float(CESSNA_TARGET_ALT_CRUISE)         # Target cruise altitude (1.0 meter AGL)
        f = self.repo_client.call_async(req)            # Transmit reposition command to flight controller
        def _cb(fut):                                   # Callback executed when autopilot responds
            self._pending = False                       # Unlock service client once response received
            try:                                        # Exception handling for service response evaluation
                ok = fut.result() and fut.result().success  # Verify autopilot accepted reposition coordinates
                self.get_logger().info(                     # Log confirmation or rejection message
                    'Reposition sent ✅' if ok else 'Reposition rejected')  # Print status confirmation text
                on_done(ok)                             # Trigger completion callback with boolean result
            except Exception:                           # Catch communication exceptions or timeouts
                on_done(False)                          # Trigger completion callback with False on failure
        f.add_done_callback(_cb)                        # Attach completion callback to future response

    def _distance_to(self, lat, lon) -> float:           # Great-circle distance in meters using Haversine formula
        if self.current_lat is None or lat is None or lon is None:  # Check for missing GPS telemetry data
            return float('inf')                          # Return infinity to prevent false waypoint triggers
        dlat = math.radians(lat - self.current_lat)      # Latitude difference in radians
        dlon = math.radians(lon - self.current_lon)      # Longitude difference in radians
        a    = (math.sin(dlat / 2) ** 2 +                # Haversine chord length calculation part 1
                math.cos(math.radians(self.current_lat)) * # Multiply cosine of current latitude
                math.cos(math.radians(lat)) * # Multiply cosine of target latitude
                math.sin(dlon / 2) ** 2)                 # Haversine chord length calculation part 2
        return 6371000 * 2 * math.asin(math.sqrt(a))     # Earth diameter * arcsin for surface distance

    def _loop(self):                                     # 10Hz control loop timer evaluating flight state machine
        if self.phase == 'WAIT_CONNECTION':              # Phase 1: Wait for MAVLink connection from autopilot
            if self.state.connected:                     # Check if MAVROS is connected to ArduPilot
                self.get_logger().info('MAVROS connected.')  # Log connection confirmation
                self.phase = 'WAIT_GPS'                  # Switch phase to wait for GPS fix
        elif self.phase == 'WAIT_GPS':                   # Phase 2: Wait for valid GPS fix and home altitude
            if self.home_alt is not None:                # Check if home altitude reference locked by GPS callback
                self.get_logger().info('GPS ready.')     # Log GPS readiness confirmation
                active_dict = CESSNA_PARAMS              # Load Cessna parameter dictionary into local variable
                self.params_to_set = list(active_dict.items())  # Convert dictionary to sequential transmission list
                self.current_param_idx = 0               # Reset parameter index tracker to start of list
                self.phase = 'CONFIG_PARAMS'             # Switch phase to begin parameter configuration
        elif self.phase == 'CONFIG_PARAMS':              # Phase 3: Sequentially transmit ArduPilot parameters
            if not self._pending:                        # Proceed only if no async service call is active
                if self.current_param_idx < len(self.params_to_set):  # Check for remaining parameters to send
                    param_id, value = self.params_to_set[self.current_param_idx]  # Get next parameter key/value
                    def param_done_cb(ok):               # Callback executed after parameter transmission
                        if ok:                           # Check if autopilot successfully saved parameter
                            self.current_param_idx += 1  # Advance index to send next parameter in list
                        else:                            # If parameter transmission failed or rejected
                            self.get_logger().error(f"Failed to set {param_id}. Retrying...")  # Log error, retry
                    self._set_parameter(param_id, value, on_done=param_done_cb)  # Send parameter to MAVROS
                else:                                    # When all parameters are confirmed saved by autopilot
                    self.get_logger().info('✅ All parameters configured successfully.')  # Log config completion
                    self.phase = 'SET_TAKEOFF'           # Switch phase to begin auto-takeoff setup
        elif self.phase == 'SET_TAKEOFF':                # Phase 4: Command ArduPlane AUTO-TAKEOFF mode (Mode 13)
            if not self._pending:                        # Proceed only if service client is free
                self._set_mode('13',                     # Request custom flight mode '13' (TAKEOFF)
                    on_done=lambda ok: setattr(          # Inline callback: if mode accepted, switch phase to ARM
                        self, 'phase', 'ARM') if ok else None)  # Advance to ARM phase on success
        elif self.phase == 'ARM':                        # Phase 5: Command autopilot to ARM motors for takeoff
            if not self._pending:                        # Proceed only if service client is free
                self._arm(                               # Transmit motor arming command to autopilot
                    on_done=lambda ok: setattr(          # Inline callback: if armed, switch phase to CLIMBING
                        self, 'phase', 'CLIMBING') if ok else None)  # Advance to CLIMBING monitoring phase
        elif self.phase == 'CLIMBING':                   # Phase 6: Monitor climb until safe altitude reached
            if self.current_alt is not None:             # Ensure altitude telemetry is currently valid
                agl = self.current_alt - self.home_alt   # Calculate current Above Ground Level (AGL) altitude
                self.get_logger().info(                  # Log current climb progress to console
                    f'Climbing... AGL={agl:.1f}m', throttle_duration_sec=2.0)  # Throttle console print to 2s
                if agl >= 0.6 and not self.deck_deployed:  # If aircraft climbs past 0.6m and deck not deployed
                    self.get_logger().info('🛡️ Altitude reached! Deploying invisible hard deck to 0.5m!')  # Log deck
                    subprocess.Popen([                   # Call Gazebo external service via terminal subprocess
                        'gz', 'service', '-s', '/world/runway/set_pose',  # Invoke Gazebo pose modification service
                        '--reqtype', 'gz.msgs.Pose',     # Specify request message type as Pose
                        '--reptype', 'gz.msgs.Boolean',  # Specify response message type as Boolean
                        '--timeout', '2000',             # Set service timeout to 2000 milliseconds
                        '--req', 'name: "invisible_hard_deck", position: {x: 0.0, y: 0.0, z: 0.5}'  # Teleport deck
                    ])                                   # End of subprocess argument list
                    self.deck_deployed = True            # Flag deck as deployed so subprocess triggers once
                target = CESSNA_PARAMS['TKOFF_ALT']      # Get target takeoff altitude from dictionary (3.0m)
                if agl >= 0.3 * target:                  # Consider takeoff established at 30% of target altitude
                    self.get_logger().info('Cruise alt reached → GUIDED')  # Log transition to guided navigation
                    self.phase = 'SET_GUIDED'            # Advance state machine to SET_GUIDED phase
        elif self.phase == 'SET_GUIDED':                 # Phase 7: Switch mode to GUIDED for waypoint navigation
            if not self._pending:                        # Proceed only if service client is free
                self._set_mode('GUIDED',                 # Request flight mode switch to 'GUIDED'
                    on_done=lambda ok: setattr(          # Inline callback: if GUIDED accepted, switch phase
                        self, 'phase', 'SEND_TARGET') if ok else None)  # Advance to SEND_TARGET on success
        elif self.phase == 'SEND_TARGET':                # Phase 8: Transmit next racetrack waypoint coordinates
            if not self._pending:                        # Proceed only if service client is free
                target_x, target_y = RACETRACK_WAYPOINTS[self.current_wp_idx]  # Get X/Y for current waypoint index
                self.target_global_lat, self.target_global_lon = self._local_to_global(target_x, target_y)  # Lat/Lon
                self.get_logger().info(                  # Log waypoint transmission details
                    f'Transmitting WP {self.current_wp_idx + 1}/{len(RACETRACK_WAYPOINTS)}: '  # Waypoint index log
                    f'X={target_x}m, Y={target_y}m')     # Include target local coordinate meters in log
                self._send_reposition(                   # Send reposition command to MAVROS
                    on_done=lambda ok: setattr(          # Inline callback: if accepted, switch to GOTO_WAYPOINT
                        self, 'phase', 'GOTO_WAYPOINT') if ok else None)  # Advance to transit monitoring phase
        elif self.phase == 'GOTO_WAYPOINT':              # Phase 9: Monitor transit toward commanded waypoint
            dist = self._distance_to(self.target_global_lat, self.target_global_lon)  # Distance to active waypoint
            self.get_logger().info(                      # Log remaining distance to waypoint
                f'En route to WP {self.current_wp_idx + 1} → dist={dist:.0f}m', throttle_duration_sec=2.0)  # Log
            agl = self.current_alt - self.home_alt       # Calculate Above Ground Level (AGL) altitude in transit
            self.get_logger().info(                      # Log current AGL altitude
                    f'AGL={agl:.1f}m', throttle_duration_sec=2.0)  # Throttle console print to 2s
            if dist < WAYPOINT_RADIUS:                   # If distance drops below acceptance radius (25m), reached
                self.get_logger().info(f'✅ Waypoint {self.current_wp_idx + 1} reached!')  # Log waypoint completion
                self.current_wp_idx = (self.current_wp_idx + 1) % len(RACETRACK_WAYPOINTS)  # Advance index; loop track
                self.phase = 'SEND_TARGET'               # Switch to SEND_TARGET phase to transmit next coordinate
        elif self.phase == 'EVADING':                    # Phase 10: State managed by obstacle callback during evasion
            if self.current_alt is not None:             # Ensure altitude telemetry is currently valid
                agl = self.current_alt - self.home_alt   # Calculate Above Ground Level (AGL) altitude while evading
                self.get_logger().info(                  # Log evasion status and altitude
                    f'EVADING... AGL={agl:.1f}m', throttle_duration_sec=1.0)  # Throttle console print to 1s
        elif self.phase == 'RESUME_MISSION':             # Phase 11: Cleanly resume navigation after clearing obstacle
            if not self._pending:                        # Proceed only if service client is free
                self._set_mode('GUIDED',                 # Re-assert GUIDED mode to clear attitude overrides
                    on_done=lambda ok: setattr(          # Inline callback: if GUIDED accepted, switch phase
                        self, 'phase', 'SEND_TARGET') if ok else None)  # Advance to SEND_TARGET on success


def main(args=None):                   # Main execution entry point when script launched via terminal or ROS 2
    rclpy.init(args=args)              # Initialize ROS 2 communications context
    node = PlaneAutopilot()            # Instantiate our PlaneAutopilot node class
    executor = MultiThreadedExecutor() # Create MultiThreadedExecutor for concurrent timers and callbacks
    executor.add_node(node)            # Add our autopilot node to the executor
    try:                               # Begin exception handling block for main execution loop
        executor.spin()                # Keep node spinning (processing callbacks and timers) indefinitely
    except KeyboardInterrupt:          # Catch Ctrl+C terminal interrupts cleanly without stack traces
        pass                           # Do nothing on keyboard interrupt to allow clean shutdown
    finally:                           # Clean-up block executed upon exit
        node.destroy_node()            # Destroy node object to clean up memory and release subscriptions
        try:                           # Begin exception handling block for ROS 2 shutdown
            rclpy.shutdown()           # Shut down the ROS 2 runtime environment
        except Exception:              # Catch shutdown exceptions if ROS 2 already terminated
            pass                       # Ignore secondary exceptions during exit


if __name__ == '__main__':             # Execute main function only when script run directly (not imported)
    main()                             # Call main execution function
