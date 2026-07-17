#!/usr/bin/env python3
import math
import rclpy
import subprocess
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from rcl_interfaces.msg import ParameterValue, ParameterType
from mavros_msgs.msg import State, AttitudeTarget, Waypoint
from mavros_msgs.srv import CommandBool, SetMode, CommandInt, ParamSetV2, WaypointPush, WaypointClear
from sensor_msgs.msg import NavSatFix, LaserScan

# ── Gazebo World Origin (From your .world file) ──────────────
ORIGIN_LAT = -35.363262
ORIGIN_LON = 149.165237

# ── Racetrack Local Coordinates ──────────────────────────────
RACETRACK_WAYPOINTS = []
for i in range(48):
    t = i * (2.0 * math.pi / 48.0)
    x = 300.0 * math.sin(t)
    y = 200.0 + 200.0 * math.sin(t) * math.cos(t)
    RACETRACK_WAYPOINTS.append((round(x, 1), round(y, 1)))

# ── Avoidance parameters ─────────────────────────────────────
DANGER_DISTANCE   = 30.0
MAX_BANK_ANGLE    = 35.0
STEER_THRESHOLD   =  2.0

# ── Vehicle type & Script Internal Params ────────────────────
SEAPLANE_MODE              = False
SEAPLANE_TARGET_ALT_CRUISE = 0.5
CESSNA_TARGET_ALT_CRUISE   = 1.0

# ── MAVROS Parameter Dictionaries ────────────────────────────
SEAPLANE_PARAMS = {
    'TKOFF_ALT':       0.5,
    'RUDDER_ONLY':     0,
    'SERVO1_FUNCTION': 4,
    'TKOFF_THR_MAX':   100.0,
    'THR_MAX':         100.0,
    'AIRSPEED_MIN':    30.0,
    'AIRSPEED_MAX':    50.0,
    'AIRSPEED_CRUISE': 4000,
    'WP_RADIUS':       20.0,
    'WP_LOITER_RAD':   15.0,
}

CESSNA_PARAMS = {
    'TKOFF_ALT':       3.0,
    'RUDDER_ONLY':     0,
    'SERVO1_FUNCTION': 4,
    'TKOFF_THR_MAX':   57.0,
    'THR_MAX':         40.0,
    'AIRSPEED_MIN':    30.0,
    'AIRSPEED_MAX':    50.0,
    'AIRSPEED_CRUISE': 4000,
    'WP_RADIUS':       25.0,
    'WP_LOITER_RAD':   5.0,
}

WAYPOINT_RADIUS = SEAPLANE_PARAMS['WP_RADIUS'] if SEAPLANE_MODE else CESSNA_PARAMS['WP_RADIUS']
CRUISE_ALT      = SEAPLANE_TARGET_ALT_CRUISE if SEAPLANE_MODE else CESSNA_TARGET_ALT_CRUISE
# ────────────────────────────────────────────────────────────


class PlaneAutopilot(Node):

    def __init__(self):
        super().__init__('plane_autopilot')

        self._timer_cbg   = MutuallyExclusiveCallbackGroup()
        self._service_cbg = MutuallyExclusiveCallbackGroup()

        self.state        = State()
        self.current_lat  = None
        self.current_lon  = None
        self.current_alt  = None
        self.home_alt     = None
        self.phase        = 'WAIT_CONNECTION'
        self._pending     = False
        self.deck_deployed = False

        # Parameter Configuration State
        self.params_to_set     = []
        self.current_param_idx = 0

        # Mission tracking (for logging/awareness — AUTO mode handles actual transitions)
        self.current_wp_idx    = 0
        self.wp_reached        = [False] * len(RACETRACK_WAYPOINTS)
        self.target_global_lat = None
        self.target_global_lon = None

        # Subscriptions
        self.create_subscription(
            State, '/mavros/state',
            self._state_cb, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self._gps_cb, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan',
            self._obstacle_cb, qos_profile_sensor_data)

        # Service clients
        self.arm_client       = self.create_client(
            CommandBool,   '/mavros/cmd/arming',
            callback_group=self._service_cbg)
        self.mode_client      = self.create_client(
            SetMode,       '/mavros/set_mode',
            callback_group=self._service_cbg)
        self.repo_client      = self.create_client(
            CommandInt,    '/mavros/cmd/command_int',
            callback_group=self._service_cbg)
        self.param_set_client = self.create_client(
            ParamSetV2,    '/mavros/param/set',
            callback_group=self._service_cbg)
        self.wp_clear_client  = self.create_client(
            WaypointClear, '/mavros/mission/clear',
            callback_group=self._service_cbg)
        self.wp_push_client   = self.create_client(
            WaypointPush,  '/mavros/mission/push',
            callback_group=self._service_cbg)

        self.attitude_pub = self.create_publisher(
            AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        self.create_timer(0.1, self._loop, callback_group=self._timer_cbg)
        self.get_logger().info('Flight Commander started.')

    # ── Subscribers ──────────────────────────────────────────

    def _state_cb(self, msg: State):
        self.state = msg

    def _gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        if self.home_alt is None and msg.altitude > 0:
            self.home_alt = msg.altitude
            self.get_logger().info(f'Home alt locked: {self.home_alt:.1f}m AMSL')

    # ── Local Coordinate Conversion ──────────────────────────

    def _local_to_global(self, x_meters, y_meters):
        R_earth = 6378137.0
        d_lat = y_meters / R_earth
        d_lon = x_meters / (R_earth * math.cos(math.radians(ORIGIN_LAT)))
        return (ORIGIN_LAT + math.degrees(d_lat),
                ORIGIN_LON + math.degrees(d_lon))

    # ── Mission upload ────────────────────────────────────────

    def _clear_mission(self, on_done):
        if self._pending or not self.wp_clear_client.service_is_ready():
            return
        self._pending = True
        f = self.wp_clear_client.call_async(WaypointClear.Request())

        def _cb(fut):
            self._pending = False
            try:
                self.get_logger().info('Mission cleared ✅')
                on_done(True)
            except Exception:
                on_done(False)

        f.add_done_callback(_cb)

    def _push_mission(self, on_done):
        """
        Upload all racetrack waypoints as a MAVLink mission.
        AUTO mode uses L1 look-ahead — plane curves naturally
        through each waypoint without needing to hit it precisely.
        """
        if self._pending or not self.wp_push_client.service_is_ready():
            return
        self._pending = True

        waypoints = []

        # Item 0: Home (required by ArduPilot as first mission item)
        home = Waypoint()
        home.frame        = Waypoint.FRAME_GLOBAL_REL_ALT
        home.command      = 16
        home.is_current   = False
        home.autocontinue = True
        home.x_lat        = ORIGIN_LAT
        home.y_long       = ORIGIN_LON
        home.z_alt        = 0.0
        waypoints.append(home)

        # Items 1..N: racetrack waypoints
        for i, (x, y) in enumerate(RACETRACK_WAYPOINTS):
            lat, lon = self._local_to_global(x, y)
            wp = Waypoint()
            wp.frame        = Waypoint.FRAME_GLOBAL_REL_ALT
            wp.command      = 16   # MAV_CMD_NAV_WAYPOINT
            wp.is_current   = (i == 0)
            wp.autocontinue = True
            wp.param1       = 0.0              # hold time
            wp.param2       = WAYPOINT_RADIUS  # acceptance radius — enables look-ahead
            wp.param3       = 0.0
            wp.param4       = float('nan')     # yaw: NaN = auto
            wp.x_lat        = lat
            wp.y_long       = lon
            wp.z_alt        = CRUISE_ALT
            waypoints.append(wp)

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints   = waypoints
        f = self.wp_push_client.call_async(req)

        def _cb(fut):
            self._pending = False
            try:
                res = fut.result()
                if res and res.success:
                    self.get_logger().info(
                        f'Mission uploaded ✅ ({len(waypoints)} waypoints)')
                    on_done(True)
                else:
                    self.get_logger().error('Mission push failed')
                    on_done(False)
            except Exception as e:
                self.get_logger().error(f'Mission push error: {e}')
                on_done(False)

        f.add_done_callback(_cb)

    # ── Avoidance: steering calculation ──────────────────────

    def _calculate_steering(self, msg: LaserScan) -> float:
        num_rays = len(msg.ranges)
        if num_rays == 0:
            return 0.0

        center_idx     = num_rays // 2
        count_left     = 0
        count_right    = 0
        min_dist_left  = float('inf')
        min_dist_right = float('inf')
        idx_min_left   = center_idx
        idx_min_right  = center_idx

        for i, dist in enumerate(msg.ranges):
            if math.isinf(dist) or math.isnan(dist) or dist > DANGER_DISTANCE:
                continue
            if i < center_idx:
                count_left += 1
                if dist < min_dist_left:
                    min_dist_left = dist
                    idx_min_left  = i
            else:
                count_right += 1
                if dist < min_dist_right:
                    min_dist_right = dist
                    idx_min_right  = i

        if count_left == 0 and count_right == 0:
            return 0.0

        def steer_mag(target_idx: int) -> float:
            dist_from_center = abs(target_idx - center_idx)
            weight = 1.0 - (dist_from_center / max(center_idx, 1))
            return weight * MAX_BANK_ANGLE

        if count_left > 0 and count_right == 0:
            return steer_mag(idx_min_left)
        elif count_right > 0 and count_left == 0:
            return -steer_mag(idx_min_right)
        elif count_left > count_right:
            return steer_mag(idx_min_left)
        elif count_right > count_left:
            return -steer_mag(idx_min_right)
        else:
            self.get_logger().warn('Symmetric obstacle — forcing LEFT evasion!')
            return -steer_mag(idx_min_right)

    # ── Avoidance: attitude commands ──────────────────────────

    def _send_roll_command(self, roll_deg: float):
        roll_rad  = math.radians(-roll_deg)
        pitch_rad = 0.0
        yaw_rad   = 0.0
        cy = math.cos(yaw_rad   * 0.5)
        sy = math.sin(yaw_rad   * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad  * 0.5)
        sr = math.sin(roll_rad  * 0.5)
        msg = AttitudeTarget()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE  |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE   |
            AttitudeTarget.IGNORE_THRUST)
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        self.attitude_pub.publish(msg)

    def _send_yaw_rate_command(self, yaw_rate_deg_s: float):
        msg = AttitudeTarget()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE  |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_THRUST)
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = math.radians(-yaw_rate_deg_s)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        self.attitude_pub.publish(msg)

    # ── Avoidance: obstacle callback ──────────────────────────

    def _obstacle_cb(self, msg: LaserScan):
        # Active only during AUTO flight or while already evading
        if self.phase not in ['FLYING_AUTO', 'EVADING']:
            return

        steering = self._calculate_steering(msg)

        if abs(steering) > STEER_THRESHOLD:
            # ── OBSTACLE DETECTED ──
            if self.phase != 'EVADING':
                self.get_logger().warn(
                    f'OBSTACLE DETECTED — steering={steering:.1f}° switching to GUIDED')
                self.phase = 'EVADING'
                # Emergency mode switch — bypasses _pending intentionally
                if self.mode_client.service_is_ready():
                    req = SetMode.Request()
                    req.custom_mode = 'GUIDED'
                    self.mode_client.call_async(req)

            if SEAPLANE_MODE:
                yaw_rate = steering * 0.5
                self._send_yaw_rate_command(yaw_rate)
                self.get_logger().info(
                    f'Yaw rate cmd: {yaw_rate:.1f} deg/s',
                    throttle_duration_sec=0.5)
            else:
                self._send_roll_command(steering)
                self.get_logger().info(
                    f'Roll cmd: {steering:.1f}°',
                    throttle_duration_sec=0.5)

        elif self.phase == 'EVADING':
            # ── PATH CLEAR — return to AUTO ──
            self.get_logger().info('Path clear — resuming AUTO mission.')
            if SEAPLANE_MODE:
                self._send_yaw_rate_command(0.0)
            else:
                self._send_roll_command(0.0)
            self.phase = 'RESUME_AUTO'

    # ── Service helpers ───────────────────────────────────────

    def _set_mode(self, mode: str, on_done):
        if self._pending or not self.mode_client.service_is_ready():
            return
        self._pending = True
        req = SetMode.Request()
        req.custom_mode = mode
        f = self.mode_client.call_async(req)

        def _cb(fut):
            self._pending = False
            try:
                ok = fut.result() and fut.result().mode_sent
                label = 'TAKEOFF' if mode == '13' else mode
                self.get_logger().info(
                    f'Mode → {label} ✅' if ok else f'Mode {label} rejected')
                on_done(ok)
            except Exception:
                on_done(False)

        f.add_done_callback(_cb)

    def _arm(self, on_done):
        if self._pending or not self.arm_client.service_is_ready():
            return
        self._pending = True
        req = CommandBool.Request()
        req.value = True
        f = self.arm_client.call_async(req)

        def _cb(fut):
            self._pending = False
            try:
                ok = fut.result() and fut.result().success
                self.get_logger().info('Armed ✅' if ok else 'Arming failed')
                on_done(ok)
            except Exception:
                on_done(False)

        f.add_done_callback(_cb)

    def _set_parameter(self, param_id: str, value, on_done):
        if self._pending or not self.param_set_client.service_is_ready():
            return
        self._pending = True
        req = ParamSetV2.Request()
        req.param_id = param_id
        req.value = ParameterValue()
        if isinstance(value, int):
            req.value.type          = ParameterType.PARAMETER_INTEGER
            req.value.integer_value = value
        else:
            req.value.type         = ParameterType.PARAMETER_DOUBLE
            req.value.double_value = float(value)
        f = self.param_set_client.call_async(req)

        def _cb(fut):
            self._pending = False
            try:
                res = fut.result()
                ok  = res.success if res else False
                self.get_logger().info(
                    f'Param {param_id}={value} ✅' if ok
                    else f'Param {param_id} failed')
                on_done(ok)
            except Exception as e:
                self.get_logger().error(f'Param error: {e}')
                on_done(False)

        f.add_done_callback(_cb)

    def _distance_to(self, lat, lon) -> float:
        if self.current_lat is None or lat is None or lon is None:
            return float('inf')
        dlat = math.radians(lat - self.current_lat)
        dlon = math.radians(lon - self.current_lon)
        a    = (math.sin(dlat / 2) ** 2 +
                math.cos(math.radians(self.current_lat)) *
                math.cos(math.radians(lat)) *
                math.sin(dlon / 2) ** 2)
        return 6371000 * 2 * math.asin(math.sqrt(a))

    # ── State machine (10 Hz) ─────────────────────────────────

    def _loop(self):

        if self.phase == 'WAIT_CONNECTION':
            if self.state.connected:
                self.get_logger().info('MAVROS connected.')
                self.phase = 'WAIT_GPS'

        elif self.phase == 'WAIT_GPS':
            if self.home_alt is not None:
                self.get_logger().info('GPS ready.')
                active_dict = SEAPLANE_PARAMS if SEAPLANE_MODE else CESSNA_PARAMS
                self.params_to_set     = list(active_dict.items())
                self.current_param_idx = 0
                self.phase = 'CONFIG_PARAMS'

        # ── Parameter sequencing ──────────────────────────────
        elif self.phase == 'CONFIG_PARAMS':
            if not self._pending:
                if self.current_param_idx < len(self.params_to_set):
                    param_id, value = self.params_to_set[self.current_param_idx]

                    def param_done_cb(ok):
                        if ok:
                            self.current_param_idx += 1
                        else:
                            self.get_logger().error(
                                f'Failed to set {param_id}. Retrying...')

                    self._set_parameter(param_id, value, on_done=param_done_cb)
                else:
                    self.get_logger().info('✅ All parameters configured — clearing mission.')
                    self.phase = 'CLEAR_MISSION'

        # ── Mission upload ────────────────────────────────────
        elif self.phase == 'CLEAR_MISSION':
            if not self._pending:
                self._clear_mission(
                    on_done=lambda ok: setattr(
                        self, 'phase', 'PUSH_MISSION') if ok else None)

        elif self.phase == 'PUSH_MISSION':
            if not self._pending:
                self._push_mission(
                    on_done=lambda ok: setattr(
                        self, 'phase', 'SET_TAKEOFF') if ok else None)

        # ── Flight Sequence ───────────────────────────────────
        elif self.phase == 'SET_TAKEOFF':
            if not self._pending:
                self._set_mode('13',
                    on_done=lambda ok: setattr(
                        self, 'phase', 'ARM') if ok else None)

        elif self.phase == 'ARM':
            if not self._pending:
                self._arm(
                    on_done=lambda ok: setattr(
                        self, 'phase', 'CLIMBING') if ok else None)

        elif self.phase == 'CLIMBING':
            if self.current_alt is not None:
                agl = self.current_alt - self.home_alt
                self.get_logger().info(
                    f'Climbing... AGL={agl:.1f}m', throttle_duration_sec=2.0)

                # ── POP-UP DECK LOGIC (unchanged from original) ──
                if agl >= 0.6 and not self.deck_deployed:
                    self.get_logger().info(
                        '🛡️ Altitude reached! Deploying invisible hard deck to 0.5m!')
                    subprocess.Popen([
                        'gz', 'service', '-s', '/world/runway/set_pose',
                        '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '2000',
                        '--req',
                        'name: "invisible_hard_deck", '
                        'position: {x: 0.0, y: 0.0, z: 0.5}'
                    ])
                    self.deck_deployed = True
                # ─────────────────────────────────────────────────

                target = SEAPLANE_PARAMS['TKOFF_ALT'] if SEAPLANE_MODE else CESSNA_PARAMS['TKOFF_ALT']
                if agl >= 0.3 * target:
                    self.get_logger().info('Cruise alt reached → switching to AUTO')
                    self.phase = 'SET_AUTO'

        # ── Switch to AUTO — mission begins ───────────────────
        elif self.phase == 'SET_AUTO':
            if not self._pending:
                self._set_mode('AUTO',
                    on_done=lambda ok: setattr(
                        self, 'phase', 'FLYING_AUTO') if ok else None)

        # ── AUTO mission running ──────────────────────────────
        # obstacle_cb handles switching to GUIDED when obstacle detected
        elif self.phase == 'FLYING_AUTO':
            if self.current_alt is not None and self.home_alt is not None:
                agl = self.current_alt - self.home_alt

                # Track waypoint progress for logging
                if self.current_wp_idx < len(RACETRACK_WAYPOINTS):
                    lat, lon = self._local_to_global(
                        *RACETRACK_WAYPOINTS[self.current_wp_idx])
                    dist = self._distance_to(lat, lon)
                    self.get_logger().info(
                        f'En route to WP {self.current_wp_idx + 1}/'
                        f'{len(RACETRACK_WAYPOINTS)} → '
                        f'dist={dist:.0f}m AGL={agl:.1f}m',
                        throttle_duration_sec=2.0)

                    if dist < WAYPOINT_RADIUS:
                        self.wp_reached[self.current_wp_idx] = True
                        self.get_logger().info(
                            f'✅ Waypoint {self.current_wp_idx + 1} passed!')
                        self.current_wp_idx = (
                            self.current_wp_idx + 1) % len(RACETRACK_WAYPOINTS)

        # ── EVADING — obstacle_cb publishes attitude cmds ─────
        elif self.phase == 'EVADING':
            if self.current_alt is not None:
                agl = self.current_alt - self.home_alt
                self.get_logger().info(
                    f'EVADING... AGL={agl:.1f}m', throttle_duration_sec=1.0)

        # ── Resume AUTO after obstacle cleared ────────────────
        elif self.phase == 'RESUME_AUTO':
            if not self._pending:
                self._set_mode('AUTO',
                    on_done=lambda ok: setattr(
                        self, 'phase', 'FLYING_AUTO') if ok else None)


def main(args=None):
    rclpy.init(args=args)
    node = PlaneAutopilot()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
