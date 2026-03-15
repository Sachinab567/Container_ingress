import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class IngressController(Node):
    """
    FSM: enter container → reach back wall → 180° spin (TURN_IN) → exit centered → 180° spin (TURN_OUT).
    
    Robot Setup:
    - Spawn yaw = +π/2 (facing container).
    - Drives visually-forward using linear.x < 0.
    - LiDAR Angles: FWD=-90°, LEFT=180°, RIGHT=0°.
    
    STATES: WAIT → APPROACH → INSIDE → TURN_IN → EXIT → TURN_OUT → DONE
    """

    SPEED      = -0.35      # Drive speed
    TURN_SPD   = 1.5        # rad/s for spin-in-place
    KP         = 1.0        # Centering gain
    WALL_TH    = 2.0        # Wall detection threshold
    FRONT_STOP = 0.5        # Stop distance from back wall
    EXIT_TH    = 3.5        # Distance to detect being "outside"

    # LiDAR mapping (relative to robot)
    FWD   = -90.0
    LEFT  = 180.0
    RIGHT = 0.0

    def __init__(self):
        super().__init__('ingress_controller')
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self._tick)

        self.state = 'WAIT'
        self.scan = None
        self.ox = self.oy = 0.0
        self.yaw = 0.0
        self.n = 0

        # Shared state variables for maneuvers
        self._backup_done = False
        self._backup_x0 = 0.0
        self._turn_started = False
        self._turn_start_yaw = 0.0

        self.get_logger().info('Ingress Controller Started - WAIT')

    def _scan_cb(self, msg):
        self.scan = msg

    def _odom_cb(self, msg):
        self.ox = msg.pose.pose.position.x
        self.oy = msg.pose.pose.position.y
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    # ── Sensing Helpers ──────────────────────────────────────────────
    def _r(self, deg):
        if not self.scan: return float('inf')
        rad = math.radians(deg)
        if rad < self.scan.angle_min or rad > self.scan.angle_max:
            return float('inf')
        idx = int((rad - self.scan.angle_min) / self.scan.angle_increment)
        if 0 <= idx < len(self.scan.ranges):
            val = self.scan.ranges[idx]
            if self.scan.range_min <= val <= self.scan.range_max:
                return val
        return float('inf')

    def _avg(self, deg, hw=15):
        if not self.scan: return float('inf')
        res = self.scan.angle_increment
        deg_step = math.degrees(res)
        vals = [self._r(deg + k * deg_step) for k in range(-hw, hw + 1)]
        good = [v for v in vals if v < float('inf')]
        return (sum(good) / len(good)) if good else float('inf')

    # ── Motion Helpers ───────────────────────────────────────────────
    def _pub(self, lx=0.0, az=0.0):
        t = Twist()
        t.linear.x = lx
        t.angular.z = az
        self.cmd.publish(t)

    def _stop(self):
        self._pub(0.0, 0.0)

    def _angle_diff(self, a, b):
        """Shortest signed angle difference: a - b, wrapped to [-π, π]."""
        d = a - b
        while d > math.pi: d -= 2.0 * math.pi
        while d < -math.pi: d += 2.0 * math.pi
        return d

    # ── FSM logic ────────────────────────────────────────────────────
    def _tick(self):
        self.n += 1
        log = (self.n % 10 == 0)

        if self.state == 'WAIT':
            if self.scan:
                self.get_logger().info('Scan detected → APPROACH')
                self.state = 'APPROACH'
            return

        if not self.scan: return

        # Raw readings
        F = self._avg(self.FWD, hw=10)
        L = self._avg(self.LEFT, hw=15)
        R = self._avg(self.RIGHT, hw=15)

        if log:
            self.get_logger().info(
                f'[{self.state}] F={F:.2f} L={L:.2f} R={R:.2f} yaw={math.degrees(self.yaw):.1f}°')

        # ── STATE: APPROACH ──────────────────────────────────────────
        if self.state == 'APPROACH':
            if L < self.WALL_TH and R < self.WALL_TH:
                self.get_logger().info('Walls detected → INSIDE')
                self.state = 'INSIDE'
                return
            
            # Simple approach centering
            az = 0.0
            if L < self.WALL_TH: az = 0.5 * (1.2 - L)
            elif R < self.WALL_TH: az = -0.5 * (1.2 - R)
            self._pub(self.SPEED, az)

        # ── STATE: INSIDE ────────────────────────────────────────────
        elif self.state == 'INSIDE':
            if F < self.FRONT_STOP:
                self._stop()
                self._backup_done = False
                self._backup_x0 = self.ox
                self._turn_started = False
                self.get_logger().info('Back wall reached → TURN_IN')
                self.state = 'TURN_IN'
                return
            
            # Center between L/R walls
            err = R - L
            az = max(-0.4, min(0.4, self.KP * err))
            self._pub(self.SPEED, az)

        # ── STATE: TURN_IN / TURN_OUT ────────────────────────────────
        elif self.state in ['TURN_IN', 'TURN_OUT']:
            # For TURN_IN, we backup slightly first
            if self.state == 'TURN_IN' and not self._backup_done:
                dist = abs(self.ox - self._backup_x0)
                if dist < 0.5:
                    self._pub(-self.SPEED, 0.0) # Move forward (lx > 0 since SPEED is neg)
                    return
                self._backup_done = True
                self.get_logger().info('Backup done, starting spin...')

            if not self._turn_started:
                self._turn_start_yaw = self.yaw
                self._turn_started = True

            turned = abs(self._angle_diff(self.yaw, self._turn_start_yaw))
            remaining = math.pi - turned
            
            if remaining > 0.05:
                # Proportional turn speed with a minimum of 0.3
                spd = max(0.3, min(self.TURN_SPD, 2.0 * remaining))
                self._pub(0.0, spd)
            else:
                self._stop()
                self._turn_started = False
                if self.state == 'TURN_IN':
                    self.get_logger().info('TURN_IN complete → EXIT')
                    self.state = 'EXIT'
                else:
                    self.get_logger().info('TURN_OUT complete → DONE')
                    self.state = 'DONE'

        # ── STATE: EXIT ──────────────────────────────────────────────
        elif self.state == 'EXIT':
            # After 180 spin, relative mapping:
            # L side sees Right Wall, R side sees Left Wall
            wall_dist_L = R
            wall_dist_R = L

            if wall_dist_L > self.EXIT_TH and wall_dist_R > self.EXIT_TH:
                self._stop()
                self._turn_started = False
                self.get_logger().info('Outside container → TURN_OUT')
                self.state = 'TURN_OUT'
                return

            # Keep straight using both wall centering and yaw lock (90 deg from start)
            target_yaw = -math.pi/2 # Facing world -X
            
            # center_err: negative if close to Left Wall (wall_dist_L small)
            center_err = wall_dist_L - wall_dist_R
            # yaw_err: positive if current yaw is > -90
            yaw_err = self._angle_diff(self.yaw, target_yaw)
            
            # Negative center_err -> steer Right (az < 0)
            # Positive yaw_err -> steer Right (az < 0)
            az = 1.0 * center_err - 4.0 * yaw_err
            az = max(-0.5, min(0.5, az))
            self._pub(self.SPEED, az)

        # ── STATE: DONE ──────────────────────────────────────────────
        elif self.state == 'DONE':
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = IngressController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
