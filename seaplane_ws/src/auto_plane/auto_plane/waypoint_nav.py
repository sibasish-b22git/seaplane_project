#!/usr/bin/env python3  # Shebang line for Python 3 execution in Linux terminal
import math             # Math library for trig, degrees/radians, and distance calculations
import rclpy            # Core ROS 2 client library for runtime communication
import subprocess       # Execute terminal commands for Gazebo service calls
from rclpy.node import Node  # Base class to define our ROS 2 autopilot node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # Prevent thread blocking during service calls
from rclpy.executors import MultiThreadedExecutor                 # Handle topics, timers, and services concurrently
from rclpy.qos import qos_profile_sensor_data                     # Low-latency QoS profile for sensor data

from rcl_interfaces.msg import ParameterValue, ParameterType  # Data types for setting MAVROS parameters
from mavros_msgs.msg import State, AttitudeTarget, Waypoint   # MAVROS state, attitude, and waypoint messages
from mavros_msgs.srv import CommandBool, SetMode, CommandInt, ParamSetV2, WaypointPush, WaypointClear  # MAVROS services
from sensor_msgs.msg import NavSatFix, LaserScan              # GPS coordinate and LiDAR scan messages

# ── Gazebo World Origin (From your .world file) ──────────────
ORIGIN_LAT = -35.363262  # Gazebo world origin latitude in simulation environment
ORIGIN_LON = 149.165237  # Gazebo world origin longitude in simulation environment

# ── Racetrack Local Coordinates ──────────────────────────────
RACETRACK_WAYPOINTS = []  # List to store dynamically generated figure-8 local waypoints
for i in range(48):       # Generate 48 waypoints spaced roughly 30m apart along track
    t = i * (2.0 * math.pi / 48.0)                 # Parametric angle 't' for current waypoint step
    x = 300.0 * math.sin(t)                        # Scale X coordinate by 300m along horizontal axis
    y = 200.0 + 200.0 * math.sin(t) * math.cos(t)  # Scale and offset Y coordinate for figure-8 intersection
    RACETRACK_WAYPOINTS.append((round(x, 1), round(y, 1)))  # Append rounded (X, Y) tuple to waypoint list

# ── Avoidance parameters ─────────────────────────────────────
DANGER_DISTANCE   = 30.0  # Max LiDAR detection distance in meters; triggers evasion
MAX_BANK_ANGLE    = 35.0  # Max roll/bank angle in degrees commanded during obstacle evasion
STEER_THRESHOLD   =  2.0  # Noise filter threshold; steering commands under 2° are ignored

# ── Vehicle type & Script Internal Params ────────────────────
SEAPLANE_MODE              = False  # Toggle switch between Cessna and Seaplane dynamics
SEAPLANE_TARGET_ALT_CRUISE = 0.5    # Target cruise altitude in meters for Seaplane
CESSNA_TARGET_ALT_CRUISE   = 1.0    # Target cruise altitude in meters for Cessna

# ── MAVROS Parameter Dictionaries ────────────────────────────
SEAPLANE_PARAMS = {            # Dictionary of ArduPilot parameters configured for Seaplane
    'TKOFF_ALT':       0.5,    # Target climb altitude (in meters) for auto-takeoff
    'RUDDER_ONLY':     0,      # Disable rudder-only mode (uses ailerons for turns)
    'SERVO1_FUNCTION': 4,      # Assign Servo Pin 1 as Aileron control surface output
    'TKOFF_THR_MAX':   100.0,  # Limit takeoff throttle to 100%
    'THR_MAX':         100.0,  # Limit max cruise throttle to 100%
    'AIRSPEED_MIN':    30.0,   # Minimum safe airspeed before stall warning triggers
    'AIRSPEED_MAX':    50.0,   # Maximum allowed airspeed to prevent structural overspeed
    'AIRSPEED_CRUISE': 4000,   # Target cruise airspeed (4000 cm/s = 40 m/s)
    'WP_RADIUS':       20.0,   # Acceptance radius (in meters) to consider waypoint reached
    'WP_LOITER_RAD':   15.0,   # Radius (in meters) for loiter/orbit holding circles
}

CESSNA_PARAMS = {              # Dictionary of ArduPilot parameters configured for Cessna
    'TKOFF_ALT':       3.0,    # Target climb altitude (in meters) for auto-takeoff
    'RUDDER_ONLY':     0,      # Disable rudder-only mode
    'SERVO1_FUNCTION': 4,      # Assign Servo Pin 1 as Aileron control surface output
    'TKOFF_THR_MAX':   57.0,   # Limit takeoff throttle to 57% to prevent excessive pitch-up
    'THR_MAX':         40.0,   # Limit max cruise throttle to 40% for stable airspeed
    'AIRSPEED_MIN':    30.0,   # Minimum safe airspeed
    'AIRSPEED_MAX':    50.0,   # Maximum allowed airspeed
    'AIRSPEED_CRUISE': 4000,   # Target cruise airspeed
    'WP_RADIUS':       25.0,   # Acceptance radius to consider waypoint reached
    'WP_LOITER_RAD':   5.0,    # Radius for loiter holding circles
}

WAYPOINT_RADIUS = SEAPLANE_PARAMS['WP_RADIUS'] if SEAPLANE_MODE else CESSNA_PARAMS['WP_RADIUS']  # Set active WP radius
CRUISE_ALT      = SEAPLANE_TARGET_ALT_CRUISE if SEAPLANE_MODE else CESSNA_TARGET_ALT_CRUISE      # Set active cruise alt
# ────────────────────────────────────────────────────────────


class PlaneAutopilot(Node):  # Main ROS 2 node for aircraft takeoff, navigation, and avoidance

    def __init__(self):      # Initialize node, subscriptions, services, and timers
        super().__init__('plane_autopilot')  # Initialize node with name 'plane_autopilot'

        self._timer_cbg   = MutuallyExclusiveCallbackGroup()  # Dedicated callback group for loop timers
        self._service_cbg = MutuallyExclusiveCallbackGroup()  # Separate callback group for async service calls

        self.state        = State()             # Store latest MAVROS system state
        self.current_lat  = None                # Store current GPS latitude
        self.current_lon  = None                # Store current GPS longitude
        self.current_alt  = None                # Store current GPS altitude (AMSL)
        self.home_alt     = None                # Lock in starting altitude upon first GPS fix
        self.phase        = 'WAIT_CONNECTION'   # Initial phase: wait for MAVROS heartbeat
        self._pending     = False               # Track active service calls to prevent command spam
        self.deck_deployed = False              # Ensure invisible deck is only spawned once

        # Parameter Configuration State
        self.params_to_set     = []             # List of parameter key-value pairs to transmit
        self.current_param_idx = 0              # Index tracker for parameter transmission list

        # Mission tracking (for logging/awareness — AUTO mode handles actual transitions)
        self.current_wp_idx    = 0                                 # Index tracker for active target waypoint
        self.wp_reached        = [False] * len(RACETRACK_WAYPOINTS)  # Array tracking completed waypoints
        self.target_global_lat = None                              # Target GPS latitude of current waypoint
        self.target_global_lon = None                              # Target GPS longitude of current waypoint

        # Subscriptions
        self.create_subscription(                                  # Subscribe to MAVROS system state updates
            State, '/mavros/state',                                # Topic name and message type for state
            self._state_cb, qos_profile_sensor_data)               # Callback handler and QoS profile
        self.create_subscription(                                  # Subscribe to global GPS position updates
            NavSatFix, '/mavros/global_position/global',           # Topic name and message type for GPS
            self._gps_cb, qos_profile_sensor_data)                 # Callback handler and QoS profile
        self.create_subscription(                                  # Subscribe to 2D LiDAR scan rays for avoidance
            LaserScan, '/scan',                                    # Topic name and message type for LiDAR
            self._obstacle_cb, qos_profile_sensor_data)            # Callback handler and QoS profile

        # Service clients
        self.arm_client       = self.create_client(                # Client to command arming/disarming of motors
            CommandBool,   '/mavros/cmd/arming',                   # MAVROS arming service name and message definition
            callback_group=self._service_cbg)                      # Assign to service callback group
        self.mode_client      = self.create_client(                # Client to change ArduPilot flight modes
            SetMode,       '/mavros/set_mode',                     # MAVROS mode switch service
            callback_group=self._service_cbg)                      # Assign to service callback group
        self.repo_client      = self.create_client(                # Client for MAVLink command_int repositioning
            CommandInt,    '/mavros/cmd/command_int',              # MAVROS command_int service
            callback_group=self._service_cbg)                      # Assign to service callback group
        self.param_set_client = self.create_client(                # Client to modify ArduPilot parameters over MAVLink
            ParamSetV2,    '/mavros/param/set',                    # MAVROS parameter service
            callback_group=self._service_cbg)                      # Assign to service callback group
        self.wp_clear_client  = self.create_client(                # Client to clear MAVLink mission waypoints
            WaypointClear, '/mavros/mission/clear',                # MAVROS mission clear service
            callback_group=self._service_cbg)                      # Assign to service callback group
        self.wp_push_client   = self.create_client(                # Client to upload new MAVLink mission waypoints
            WaypointPush,  '/mavros/mission/push',                 # MAVROS mission push service
            callback_group=self._service_cbg)                      # Assign to service callback group

        self.attitude_pub = self.create_publisher(                 # Publisher for raw attitude targets in GUIDED mode
            AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)   # Topic name, message type, queue size 10

        self.create_timer(0.1, self._loop, callback_group=self._timer_cbg)  # 10Hz control loop timer (every 0.1s)
        self.get_logger().info('Flight Commander started.')        # Log confirmation that autopilot node initialized

    # ── Subscribers ──────────────────────────────────────────

    def _state_cb(self, msg: State):  # Callback function updating internal state from MAVROS messages
        self.state = msg              # Store latest MAVROS state message

    def _gps_cb(self, msg: NavSatFix):                # Callback capturing real-time GPS coordinates and altitude
        self.current_lat = msg.latitude               # Update current latitude from GPS
        self.current_lon = msg.longitude              # Update current longitude from GPS
        self.current_alt = msg.altitude               # Update current altitude (AMSL)
        if self.home_alt is None and msg.altitude > 0:  # Lock initial altitude as ground elevation reference
            self.home_alt = msg.altitude              # Save initial altitude as home ground reference
            self.get_logger().info(f'Home alt locked: {self.home_alt:.1f}m AMSL')  # Log locked home altitude

    # ── Local Coordinate Conversion ──────────────────────────

    def _local_to_global(self, x_meters, y_meters):  # Convert Gazebo X/Y Cartesian meters into Global GPS Lat/Lon
        R_earth = 6378137.0                          # Approximate Earth radius in meters
        d_lat = y_meters / R_earth                   # Latitude offset in radians (North/South movement)
        d_lon = x_meters / (R_earth * math.cos(math.radians(ORIGIN_LAT)))  # Longitude offset in radians
        return (ORIGIN_LAT + math.degrees(d_lat),    # Convert radian offset to degrees and add to origin lat
                ORIGIN_LON + math.degrees(d_lon))    # Convert radian offset to degrees and add to origin lon

    # ── Mission upload ────────────────────────────────────────

    def _clear_mission(self, on_done):               # Asynchronously wipe existing flight plan from flight controller
        if self._pending or not self.wp_clear_client.service_is_ready():  # Ignore if client busy or service offline
            return                                   # Exit function early without calling service
        self._pending = True                         # Lock service client to prevent command spam
        f = self.wp_clear_client.call_async(WaypointClear.Request())  # Send async mission clear request

        def _cb(fut):                                # Callback executed when autopilot responds
            self._pending = False                    # Unlock service client once response received
            try:                                     # Exception handling for service response evaluation
                self.get_logger().info('Mission cleared ✅')  # Log confirmation that mission was wiped
                on_done(True)                        # Trigger completion callback with True
            except Exception:                        # Catch communication exceptions or timeouts
                on_done(False)                       # Trigger completion callback with False on failure

        f.add_done_callback(_cb)                     # Attach completion callback to future response

    def _push_mission(self, on_done):
        """
        Upload all racetrack waypoints as a MAVLink mission.
        AUTO mode uses L1 look-ahead — plane curves naturally
        through each waypoint without needing to hit it precisely.
        """
        if self._pending or not self.wp_push_client.service_is_ready():  # Ignore if client busy or service offline
            return                                   # Exit function early without calling service
        self._pending = True                         # Lock service client to prevent command spam

        waypoints = []                               # Initialize empty list to hold structured waypoints

        # Item 0: Home (required by ArduPilot as first mission item)
        home = Waypoint()                                 # Initialize home waypoint message
        home.frame        = Waypoint.FRAME_GLOBAL_REL_ALT # Altitude relative to home elevation
        home.command      = 16                            # MAV_CMD_NAV_WAYPOINT identifier
        home.is_current   = False                         # Not the currently active waypoint
        home.autocontinue = True                          # Continue to next item automatically
        home.x_lat        = ORIGIN_LAT                    # Set home latitude
        home.y_long       = ORIGIN_LON                    # Set home longitude
        home.z_alt        = 0.0                           # Set home altitude to 0.0 (ground level)
        waypoints.append(home)                            # Append home waypoint to list

        # Items 1..N: racetrack waypoints
        for i, (x, y) in enumerate(RACETRACK_WAYPOINTS):  # Loop through generated local coordinates
            lat, lon = self._local_to_global(x, y)        # Convert Cartesian X/Y to global Lat/Lon
            wp = Waypoint()                               # Initialize standard waypoint message
            wp.frame        = Waypoint.FRAME_GLOBAL_REL_ALT  # Altitude relative to home elevation
            wp.command      = 16                   # MAV_CMD_NAV_WAYPOINT identifier
            wp.is_current   = (i == 0)             # Make the first uploaded waypoint the active one
            wp.autocontinue = True                 # Automatically advance to next waypoint upon reach
            wp.param1       = 0.0                  # Hold time at waypoint (0 = fly straight through)
            wp.param2       = WAYPOINT_RADIUS      # Acceptance radius — enables smooth L1 look-ahead cornering
            wp.param3       = 0.0                  # Pass-through distance (0 = pass directly through)
            wp.param4       = float('nan')         # Target yaw angle (NaN = let autopilot handle yaw)
            wp.x_lat        = lat                  # Target GPS latitude
            wp.y_long       = lon                  # Target GPS longitude
            wp.z_alt        = CRUISE_ALT           # Target altitude above ground
            waypoints.append(wp)                   # Append formatted waypoint to list

        req = WaypointPush.Request()               # Initialize mission push service request
        req.start_index = 0                        # Start writing at index 0 in flight controller memory
        req.waypoints   = waypoints                # Attach populated waypoint list to request
        f = self.wp_push_client.call_async(req)    # Transmit mission asynchronously to autopilot

        def _cb(fut):                              # Callback executed when autopilot responds
            self._pending = False                  # Unlock service client once response received
            try:                                   # Exception handling for service response evaluation
                res = fut.result()                 # Extract response from service future
                if res and res.success:            # Verify flight controller accepted the mission points
                    self.get_logger().info(        # Log success and total waypoints uploaded
                        f'Mission uploaded ✅ ({len(waypoints)} waypoints)')
                    on_done(True)                  # Trigger completion callback with True
                else:                              # If autopilot rejected the mission upload
                    self.get_logger().error('Mission push failed')  # Log failure warning
                    on_done(False)                 # Trigger completion callback with False
            except Exception as e:                 # Catch communication or parsing errors
                self.get_logger().error(f'Mission push error: {e}')  # Log specific exception error
                on_done(False)                     # Trigger completion callback with False

        f.add_done_callback(_cb)                   # Attach completion callback to future response

    # ── Avoidance: steering calculation ──────────────────────

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
        elif count_left > count_right:                       # If more obstacle points on left, steer right
            return steer_mag(idx_min_left)                   # Return positive bank angle away from left threat
        elif count_right > count_left:                       # If more obstacle points on right, steer left
            return -steer_mag(idx_min_right)                 # Return negative bank angle away from right threat
        else:                                                # If both sides equally blocked, default hard left
            self.get_logger().warn('Symmetric obstacle — forcing LEFT evasion!')  # Warn about symmetric obstacle
            return -steer_mag(idx_min_right)                 # Return negative bank angle to evade left

    # ── Avoidance: attitude commands ──────────────────────────

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
            AttitudeTarget.IGNORE_THRUST)                      # Ignore throttle/thrust setpoint
        msg.orientation.w = cr * cp * cy + sr * sp * sy        # Quaternion w component from Euler angles
        msg.orientation.x = sr * cp * cy - cr * sp * sy        # Quaternion x component from Euler angles
        msg.orientation.y = cr * sp * cy + sr * cp * sy        # Quaternion y component from Euler angles
        msg.orientation.z = cr * cp * sy - sr * sp * cy        # Quaternion z component from Euler angles
        self.attitude_pub.publish(msg)                         # Publish formatted attitude command to MAVROS

    def _send_yaw_rate_command(self, yaw_rate_deg_s: float):  # Command yaw rotation rate (used for seaplane dynamics)
        msg = AttitudeTarget()                                # Initialize raw attitude target ROS 2 message
        msg.header.stamp    = self.get_clock().now().to_msg() # Stamp message with current system clock time
        msg.header.frame_id = 'base_link'                     # Specify body-relative coordinate frame
        msg.type_mask = (                                     # Mask to command yaw RATE, not specific angles
            AttitudeTarget.IGNORE_ROLL_RATE  |                # Ignore body roll rate setpoint
            AttitudeTarget.IGNORE_PITCH_RATE |                # Ignore body pitch rate setpoint
            AttitudeTarget.IGNORE_THRUST)                     # Ignore throttle/thrust setpoint
        msg.body_rate.x = 0.0                                 # Zero out roll rate
        msg.body_rate.y = 0.0                                 # Zero out pitch rate
        msg.body_rate.z = math.radians(-yaw_rate_deg_s)       # Set target yaw rotation rate in rad/s
        msg.orientation.w = 1.0                               # Set default neutral quaternion w
        msg.orientation.x = 0.0                               # Set default neutral quaternion x
        msg.orientation.y = 0.0                               # Set default neutral quaternion y
        msg.orientation.z = 0.0                               # Set default neutral quaternion z
        self.attitude_pub.publish(msg)                        # Publish formatted yaw rate command to MAVROS

    # ── Avoidance: obstacle callback ──────────────────────────

    def _obstacle_cb(self, msg: LaserScan):          # Analyze LiDAR scans to override path for obstacles
        # Active only during AUTO flight or while already evading
        if self.phase not in ['FLYING_AUTO', 'EVADING']:  # Ignore scans if taking off or configuring parameters
            return

        steering = self._calculate_steering(msg)     # Calculate required evasive steering/bank angle from LiDAR

        if abs(steering) > STEER_THRESHOLD:          # If steering exceeds noise threshold (2°), evade
            # ── OBSTACLE DETECTED ──
            if self.phase != 'EVADING':              # If previously flying normally, switch to EVADING
                self.get_logger().warn(              # Log warning that obstacle was detected
                    f'OBSTACLE DETECTED — steering={steering:.1f}° switching to GUIDED')
                self.phase = 'EVADING'               # Switch active state machine phase to EVADING
                # Emergency mode switch — bypasses _pending intentionally for instant response
                if self.mode_client.service_is_ready():    # Ensure mode switch service is active
                    req = SetMode.Request()                # Initialize mode switch request
                    req.custom_mode = 'GUIDED'             # Request GUIDED mode to allow attitude overrides
                    self.mode_client.call_async(req)       # Fire async mode change without waiting

            if SEAPLANE_MODE:                        # If operating in seaplane dynamics mode
                yaw_rate = steering * 0.5            # Scale steering angle to a yaw rotation rate
                self._send_yaw_rate_command(yaw_rate)  # Command seaplane to skid/yaw away
                self.get_logger().info(              # Log active yaw command to terminal
                    f'Yaw rate cmd: {yaw_rate:.1f} deg/s', throttle_duration_sec=0.5)
            else:                                    # If operating in standard fixed-wing Cessna mode
                self._send_roll_command(steering)    # Command Cessna ailerons to bank away
                self.get_logger().info(              # Log active roll command to terminal
                    f'Roll cmd: {steering:.1f}°', throttle_duration_sec=0.5)

        elif self.phase == 'EVADING':                # If steering dropped below threshold while EVADING
            # ── PATH CLEAR — return to AUTO ──
            self.get_logger().info('Path clear — resuming AUTO mission.')  # Log clear path confirmation
            if SEAPLANE_MODE:                        # Depending on active vehicle mode
                self._send_yaw_rate_command(0.0)     # Zero out seaplane yaw rate
            else:
                self._send_roll_command(0.0)         # Command 0 roll (level wings) to stabilize Cessna
            self.phase = 'RESUME_AUTO'               # Switch to RESUME_AUTO phase to re-engage mission sequencer

    # ── Service helpers ───────────────────────────────────────

    def _set_mode(self, mode: str, on_done):        # Asynchronously switch ArduPilot flight mode
        if self._pending or not self.mode_client.service_is_ready():  # Ignore if busy or service offline
            return                                  # Exit function early
        self._pending = True                        # Lock service client
        req = SetMode.Request()                     # Initialize mode switch request message
        req.custom_mode = mode                      # Assign target custom mode string (e.g., 'AUTO')
        f = self.mode_client.call_async(req)        # Send async service request

        def _cb(fut):                               # Callback executed when autopilot responds
            self._pending = False                   # Unlock service client
            try:                                    # Exception handling block
                ok = fut.result() and fut.result().mode_sent  # Verify flight controller accepted mode switch
                label = 'TAKEOFF' if mode == '13' else mode   # Translate numeric mode '13' to label
                self.get_logger().info(             # Log mode switch status
                    f'Mode → {label} ✅' if ok else f'Mode {label} rejected')
                on_done(ok)                         # Trigger completion callback with boolean result
            except Exception:                       # Catch communication errors
                on_done(False)                      # Trigger completion callback with False

        f.add_done_callback(_cb)                    # Attach completion callback to future response

    def _arm(self, on_done):                        # Asynchronously command ArduPilot to ARM motors
        if self._pending or not self.arm_client.service_is_ready():  # Ignore if busy or service offline
            return                                  # Exit function early
        self._pending = True                        # Lock service client
        req = CommandBool.Request()                 # Initialize arming command request message
        req.value = True                            # Set arm value to True (arm motors)
        f = self.arm_client.call_async(req)         # Send async arming request

        def _cb(fut):                               # Callback executed when autopilot responds
            self._pending = False                   # Unlock service client
            try:                                    # Exception handling block
                ok = fut.result() and fut.result().success  # Check if armed successfully
                self.get_logger().info('Armed ✅' if ok else 'Arming failed')  # Log arming status
                on_done(ok)                         # Trigger completion callback
            except Exception:                       # Catch errors
                on_done(False)                      # Trigger completion callback with False

        f.add_done_callback(_cb)                    # Attach completion callback to future response

    def _set_parameter(self, param_id: str, value, on_done):  # Asynchronously transmit parameter change to ArduPilot
        if self._pending or not self.param_set_client.service_is_ready():  # Ignore if busy or service offline
            return                                            # Exit function early
        self._pending = True                                  # Lock service client
        req = ParamSetV2.Request()                            # Initialize parameter set service request
        req.param_id = param_id                               # Assign parameter string ID
        req.value = ParameterValue()                          # Initialize parameter value structure
        if isinstance(value, int):                            # If parameter value is an integer
            req.value.type          = ParameterType.PARAMETER_INTEGER  # Set parameter type flag to integer
            req.value.integer_value = value                            # Assign integer value
        else:                                                 # Otherwise format as double
            req.value.type         = ParameterType.PARAMETER_DOUBLE    # Set parameter type flag to double
            req.value.double_value = float(value)                      # Assign floating point value
        f = self.param_set_client.call_async(req)             # Transmit parameter asynchronously

        def _cb(fut):                                         # Callback executed when autopilot responds
            self._pending = False                             # Unlock service client
            try:                                              # Exception handling block
                res = fut.result()                            # Get result from future
                ok  = res.success if res else False           # Verify parameter was saved
                self.get_logger().info(                       # Log parameter modification status
                    f'Param {param_id}={value} ✅' if ok else f'Param {param_id} failed')
                on_done(ok)                                   # Trigger completion callback
            except Exception as e:                            # Catch transmission errors
                self.get_logger().error(f'Param error: {e}')  # Log error message
                on_done(False)                                # Trigger completion callback with False

        f.add_done_callback(_cb)                              # Attach completion callback to future response

    def _distance_to(self, lat, lon) -> float:               # Great-circle distance in meters using Haversine formula
        if self.current_lat is None or lat is None or lon is None:  # Check for missing GPS telemetry
            return float('inf')                              # Return infinity to prevent false triggers
        dlat = math.radians(lat - self.current_lat)          # Latitude difference in radians
        dlon = math.radians(lon - self.current_lon)          # Longitude difference in radians
        a    = (math.sin(dlat / 2) ** 2 +                    # Haversine calculation part 1
                math.cos(math.radians(self.current_lat)) * # Haversine calculation part 2
                math.cos(math.radians(lat)) * # Haversine calculation part 3
                math.sin(dlon / 2) ** 2)                     # Haversine calculation part 4
        return 6371000 * 2 * math.asin(math.sqrt(a))         # Earth diameter * arcsin for surface distance

    # ── State machine (10 Hz) ─────────────────────────────────

    def _loop(self):                                         # 10Hz control loop evaluating flight state machine

        if self.phase == 'WAIT_CONNECTION':                  # Phase: Wait for MAVLink connection
            if self.state.connected:                         # Check if MAVROS is connected
                self.get_logger().info('MAVROS connected.')  # Log connection confirmation
                self.phase = 'WAIT_GPS'                      # Switch phase to wait for GPS fix

        elif self.phase == 'WAIT_GPS':                       # Phase: Wait for valid GPS fix
            if self.home_alt is not None:                    # Check if home altitude reference is locked
                self.get_logger().info('GPS ready.')         # Log GPS readiness
                active_dict = SEAPLANE_PARAMS if SEAPLANE_MODE else CESSNA_PARAMS  # Select correct param dict
                self.params_to_set     = list(active_dict.items())  # Convert dict to sequential list
                self.current_param_idx = 0                   # Reset parameter index tracker
                self.phase = 'CONFIG_PARAMS'                 # Switch phase to configure parameters

        # ── Parameter sequencing ──────────────────────────────
        elif self.phase == 'CONFIG_PARAMS':                  # Phase: Sequentially transmit parameters
            if not self._pending:                            # Proceed if service client is free
                if self.current_param_idx < len(self.params_to_set):  # Check for remaining parameters
                    param_id, value = self.params_to_set[self.current_param_idx]  # Get next parameter

                    def param_done_cb(ok):                   # Callback after parameter transmission
                        if ok:                               # If parameter saved successfully
                            self.current_param_idx += 1      # Advance index for next parameter
                        else:                                # If parameter rejected
                            self.get_logger().error(         # Log error and retry
                                f'Failed to set {param_id}. Retrying...')

                    self._set_parameter(param_id, value, on_done=param_done_cb)  # Send parameter to MAVROS
                else:                                        # When all parameters are saved
                    self.get_logger().info('✅ All parameters configured — clearing mission.')  # Log completion
                    self.phase = 'CLEAR_MISSION'             # Switch phase to clear old missions

        # ── Mission upload ────────────────────────────────────
        elif self.phase == 'CLEAR_MISSION':                  # Phase: Wipe existing flight plan
            if not self._pending:                            # Proceed if service client is free
                self._clear_mission(                         # Send clear command
                    on_done=lambda ok: setattr(              # Inline callback: switch to PUSH_MISSION on success
                        self, 'phase', 'PUSH_MISSION') if ok else None)

        elif self.phase == 'PUSH_MISSION':                   # Phase: Upload new flight plan
            if not self._pending:                            # Proceed if service client is free
                self._push_mission(                          # Send push command
                    on_done=lambda ok: setattr(              # Inline callback: switch to SET_TAKEOFF on success
                        self, 'phase', 'SET_TAKEOFF') if ok else None)

        # ── Flight Sequence ───────────────────────────────────
        elif self.phase == 'SET_TAKEOFF':                    # Phase: Command AUTO-TAKEOFF mode (Mode 13)
            if not self._pending:                            # Proceed if service client is free
                self._set_mode('13',                         # Request mode '13' (TAKEOFF)
                    on_done=lambda ok: setattr(              # Inline callback: switch to ARM on success
                        self, 'phase', 'ARM') if ok else None)

        elif self.phase == 'ARM':                            # Phase: Command motors to ARM
            if not self._pending:                            # Proceed if service client is free
                self._arm(                                   # Send arm command
                    on_done=lambda ok: setattr(              # Inline callback: switch to CLIMBING on success
                        self, 'phase', 'CLIMBING') if ok else None)

        elif self.phase == 'CLIMBING':                       # Phase: Monitor climb until safe altitude
            if self.current_alt is not None:                 # Ensure altitude telemetry is valid
                agl = self.current_alt - self.home_alt       # Calculate Above Ground Level (AGL)
                self.get_logger().info(                      # Log climb progress
                    f'Climbing... AGL={agl:.1f}m', throttle_duration_sec=2.0)

                # ── POP-UP DECK LOGIC (unchanged from original) ──
                if agl >= 0.6 and not self.deck_deployed:    # If climbed past 0.6m and deck not deployed
                    self.get_logger().info(                  # Log deck deployment
                        '🛡️ Altitude reached! Deploying invisible hard deck to 0.5m!')
                    subprocess.Popen([                       # Call Gazebo external service via subprocess
                        'gz', 'service', '-s', '/world/runway/set_pose',
                        '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '2000',
                        '--req',
                        'name: "invisible_hard_deck", '
                        'position: {x: 0.0, y: 0.0, z: 0.5}'
                    ])
                    self.deck_deployed = True                # Flag deck as deployed to prevent looping
                # ─────────────────────────────────────────────────

                target = SEAPLANE_PARAMS['TKOFF_ALT'] if SEAPLANE_MODE else CESSNA_PARAMS['TKOFF_ALT']  # Get takeoff alt
                if agl >= 0.3 * target:                      # Consider takeoff established at 30% of target
                    self.get_logger().info('Cruise alt reached → switching to AUTO')  # Log transition
                    self.phase = 'SET_AUTO'                  # Advance to SET_AUTO phase

        # ── Switch to AUTO — mission begins ───────────────────
        elif self.phase == 'SET_AUTO':                       # Phase: Switch mode to AUTO for mission execution
            if not self._pending:                            # Proceed if service client is free
                self._set_mode('AUTO',                       # Request 'AUTO' mode
                    on_done=lambda ok: setattr(              # Inline callback: switch to FLYING_AUTO on success
                        self, 'phase', 'FLYING_AUTO') if ok else None)

        # ── AUTO mission running ──────────────────────────────
        # obstacle_cb handles switching to GUIDED when obstacle detected
        elif self.phase == 'FLYING_AUTO':                    # Phase: Monitor transit along mission path
            if self.current_alt is not None and self.home_alt is not None:  # Ensure telemetry is valid
                agl = self.current_alt - self.home_alt       # Calculate AGL

                # Track waypoint progress for logging
                if self.current_wp_idx < len(RACETRACK_WAYPOINTS):  # Verify active waypoint index is valid
                    lat, lon = self._local_to_global(               # Convert target local WP to global Lat/Lon
                        *RACETRACK_WAYPOINTS[self.current_wp_idx])
                    dist = self._distance_to(lat, lon)              # Calculate distance to active waypoint
                    self.get_logger().info(                         # Log remaining distance and altitude
                        f'En route to WP {self.current_wp_idx + 1}/'
                        f'{len(RACETRACK_WAYPOINTS)} → '
                        f'dist={dist:.0f}m AGL={agl:.1f}m',
                        throttle_duration_sec=2.0)                  # Throttle print to 2 seconds

                    if dist < WAYPOINT_RADIUS:                      # If distance drops below acceptance radius
                        self.wp_reached[self.current_wp_idx] = True # Mark waypoint as completed
                        self.get_logger().info(                     # Log waypoint completion
                            f'✅ Waypoint {self.current_wp_idx + 1} passed!')
                        self.current_wp_idx = (                     # Advance index; loop track back to 0 if at end
                            self.current_wp_idx + 1) % len(RACETRACK_WAYPOINTS)

        # ── EVADING — obstacle_cb publishes attitude cmds ─────
        elif self.phase == 'EVADING':                        # Phase: State managed by obstacle callback during evasion
            if self.current_alt is not None:                 # Ensure altitude telemetry is valid
                agl = self.current_alt - self.home_alt       # Calculate AGL during evasion
                self.get_logger().info(                      # Log evasion status and altitude
                    f'EVADING... AGL={agl:.1f}m', throttle_duration_sec=1.0)

        # ── Resume AUTO after obstacle cleared ────────────────
        elif self.phase == 'RESUME_AUTO':                    # Phase: Switch back to AUTO mode after evasion
            if not self._pending:                            # Proceed if service client is free
                self._set_mode('AUTO',                       # Request 'AUTO' mode to re-engage mission sequencer
                    on_done=lambda ok: setattr(              # Inline callback: switch to FLYING_AUTO on success
                        self, 'phase', 'FLYING_AUTO') if ok else None)


def main(args=None):                       # Main execution entry point when script launched via terminal
    rclpy.init(args=args)                  # Initialize ROS 2 communications context
    node = PlaneAutopilot()                # Instantiate our PlaneAutopilot node class
    executor = MultiThreadedExecutor()     # Create MultiThreadedExecutor for concurrent timers and callbacks
    executor.add_node(node)                # Add our autopilot node to the executor
    try:                                   # Begin exception handling block for main loop
        executor.spin()                    # Keep node spinning (processing callbacks/timers) indefinitely
    except KeyboardInterrupt:              # Catch Ctrl+C terminal interrupts cleanly
        pass                               # Do nothing on interrupt to allow clean finally block execution
    finally:                               # Clean-up block executed upon exit
        node.destroy_node()                # Destroy node object to clean up memory
        try:                               # Exception block for ROS 2 shutdown
            rclpy.shutdown()               # Shut down the ROS 2 runtime environment
        except Exception:                  # Catch exceptions if ROS 2 already terminated
            pass                           # Ignore secondary exceptions during exit


if __name__ == '__main__':                 # Execute main function only when script run directly
    main()                                 # Call main execution function
