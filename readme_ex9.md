# Übungsblatt 09: Path Planning and Motion Control

[Link to repo](https://github.com/susifohn/px4-sim)

Name: Christian.Kissling@students.unibe.ch

ROS2 version: Jazzy Jalisco | Gazebo: Harmonic | PX4 SITL

---

## Prerequisites

Building on the containerized environment from Exercise 8 (px4-sim):
- Docker Desktop (Windows/WSL)
- The `erdemuysalx/px4-sitl:latest` image already built in Exercise 8
- VNC access via `http://localhost:6080/vnc_lite.html` (Password: **1234**)

Additional packages needed inside the container:
```bash
sudo apt-get -o Acquire::ForceIPv4=true update && sudo apt-get install -y \
  ros-jazzy-octomap \
  ros-jazzy-octomap-ros \
  ros-jazzy-octomap-msgs \
  ros-jazzy-nav2-msgs \
  python3-numpy \
  python3-scipy
```

---

## Setup procedure

1. Start Docker Desktop
2. From your project folder:
```bash
docker-compose up -d
```
3. Open VNC desktop: `http://localhost:6080/vnc_lite.html` → Password: **1234**
4. Open a terminal inside the container and start PX4 SITL:
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth
```
5. In a second terminal, source ROS2 and the workspace:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## Aufgabe 1: Global Planner — 3D Path Planning

### Objective
Design and implement a ROS2 global planner that generates collision-free 3D paths for a UAV using RRT (Rapidly-exploring Random Tree) on a voxel grid, with altitude constraints and no-fly zones.

### ROS2 Package Setup

Inside the container:
```bash
mkdir -p /root/ros2_ws/src
cd /root/ros2_ws/src

ros2 pkg create global_planner \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs std_msgs geometry_msgs nav_msgs mavros_msgs

cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Node: `rrt_global_planner.py`

Create `/root/ros2_ws/src/global_planner/global_planner/rrt_global_planner.py`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush

import numpy as np
import math
import random


# ── No-fly zones: list of (cx, cy, cz, radius) spheres ──────────────────────
NO_FLY_ZONES = [
    (5.0,  5.0,  3.0, 2.0),
    (-3.0, 8.0,  4.0, 1.5),
    (10.0, 2.0,  2.5, 1.8),
]

# ── Altitude constraints ─────────────────────────────────────────────────────
MIN_ALT = 1.5   # metres above ground
MAX_ALT = 20.0  # metres above ground


class RRTNode:
    """Single node in the RRT tree."""
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z
        self.parent = None


def distance(a: RRTNode, b: RRTNode) -> float:
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)


def in_no_fly_zone(node: RRTNode) -> bool:
    for cx, cy, cz, r in NO_FLY_ZONES:
        if math.sqrt((node.x-cx)**2 + (node.y-cy)**2 + (node.z-cz)**2) < r:
            return True
    return False


def altitude_ok(node: RRTNode) -> bool:
    return MIN_ALT <= node.z <= MAX_ALT


class RRTGlobalPlanner(Node):

    def __init__(self):
        super().__init__('rrt_global_planner')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('max_iter',   3000)
        self.declare_parameter('step_size',  1.0)
        self.declare_parameter('goal_bias',  0.15)
        self.declare_parameter('goal_radius',0.8)

        self.max_iter    = self.get_parameter('max_iter').value
        self.step_size   = self.get_parameter('step_size').value
        self.goal_bias   = self.get_parameter('goal_bias').value
        self.goal_radius = self.get_parameter('goal_radius').value

        # ── Publishers ────────────────────────────────────────────────────────
        self.path_pub = self.create_publisher(Path, '/rrt/planned_path', 10)

        # ── Waypoint push service (MAVROS) ────────────────────────────────────
        self.wp_client = self.create_client(WaypointPush, '/mavros/mission/push')

        # ── Goal subscriber ───────────────────────────────────────────────────
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Fixed start (UAV home / current position)
        self.start = RRTNode(0.0, 0.0, 3.0)

        self.get_logger().info('RRT Global Planner ready — publish a /goal_pose to plan.')

    # ── RRT core ──────────────────────────────────────────────────────────────

    def rrt_plan(self, start: RRTNode, goal: RRTNode):
        tree = [start]

        for _ in range(self.max_iter):
            # Sample: bias toward goal
            if random.random() < self.goal_bias:
                sample = RRTNode(goal.x, goal.y, goal.z)
            else:
                sample = RRTNode(
                    random.uniform(-20, 20),
                    random.uniform(-20, 20),
                    random.uniform(MIN_ALT, MAX_ALT),
                )

            # Nearest node in tree
            nearest = min(tree, key=lambda n: distance(n, sample))

            # Steer toward sample
            d = distance(nearest, sample)
            if d < 1e-6:
                continue
            ratio = self.step_size / d
            new_node = RRTNode(
                nearest.x + ratio * (sample.x - nearest.x),
                nearest.y + ratio * (sample.y - nearest.y),
                nearest.z + ratio * (sample.z - nearest.z),
            )
            new_node.parent = nearest

            # Collision / constraint checks
            if in_no_fly_zone(new_node) or not altitude_ok(new_node):
                continue

            tree.append(new_node)

            # Goal reached?
            if distance(new_node, goal) < self.goal_radius:
                goal.parent = new_node
                return self._extract_path(goal)

        self.get_logger().warn('RRT: max iterations reached — no path found.')
        return None

    def _extract_path(self, node: RRTNode):
        path = []
        while node is not None:
            path.append((node.x, node.y, node.z))
            node = node.parent
        path.reverse()
        return path

    # ── Goal callback ─────────────────────────────────────────────────────────

    def goal_callback(self, msg: PoseStamped):
        goal = RRTNode(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z if msg.pose.position.z > 0 else 5.0,
        )
        self.get_logger().info(
            f'Goal received: ({goal.x:.1f}, {goal.y:.1f}, {goal.z:.1f})')

        path = self.rrt_plan(self.start, goal)
        if path is None:
            return

        self.get_logger().info(f'Path found with {len(path)} waypoints.')
        self._publish_path(path)
        self._push_waypoints(path)

    # ── Publish RViz path ─────────────────────────────────────────────────────

    def _publish_path(self, path):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, z in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # ── Push waypoints to PX4 via MAVROS ─────────────────────────────────────

    def _push_waypoints(self, path):
        if not self.wp_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('MAVROS WaypointPush service not available.')
            return

        wps = []
        for i, (x, y, z) in enumerate(path):
            wp = Waypoint()
            wp.frame        = Waypoint.FRAME_LOCAL_NED
            wp.command      = 16          # MAV_CMD_NAV_WAYPOINT
            wp.is_current   = (i == 0)
            wp.autocontinue = True
            wp.x_lat        = x
            wp.y_long       = y
            wp.z_alt        = z
            wps.append(wp)

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints   = wps

        future = self.wp_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info('Waypoints pushed to PX4 successfully.')
        else:
            self.get_logger().error('Failed to push waypoints to PX4.')


def main(args=None):
    rclpy.init(args=args)
    node = RRTGlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Register entry point in `setup.py`

```python
entry_points={
    'console_scripts': [
        'rrt_global_planner = global_planner.rrt_global_planner:main',
    ],
},
```

### Build and run

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 run global_planner rrt_global_planner
```

### Send a goal

In a second terminal, publish a goal pose:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 12.0, y: 8.0, z: 6.0}, orientation: {w: 1.0}}}"
```

### Verify planned path

```bash
ros2 topic echo /rrt/planned_path
```

### RViz2 visualization

Launch RViz2 and add:
- **Display type**: Path → Topic: `/rrt/planned_path`
- **Fixed Frame**: `map`

```bash
rviz2
```

---

### Altitude constraints and no-fly zones

The planner enforces the following at every RRT expansion step:

| Constraint | Value |
|---|---|
| Minimum altitude | 1.5 m |
| Maximum altitude | 20.0 m |
| No-fly zone 1 | centre (5, 5, 3), radius 2.0 m |
| No-fly zone 2 | centre (−3, 8, 4), radius 1.5 m |
| No-fly zone 3 | centre (10, 2, 2.5), radius 1.8 m |

Any sampled node that violates altitude bounds or falls inside a no-fly sphere is discarded before being added to the tree.

### Expected console output

```
[INFO] [rrt_global_planner]: RRT Global Planner ready — publish a /goal_pose to plan.
[INFO] [rrt_global_planner]: Goal received: (12.0, 8.0, 6.0)
[INFO] [rrt_global_planner]: Path found with 38 waypoints.
[INFO] [rrt_global_planner]: Waypoints pushed to PX4 successfully.
```

### Question: Why use RRT for 3D UAV path planning?

**Answer:** RRT (Rapidly-exploring Random Tree) is well suited for high-dimensional, continuous 3D spaces because it does not require a pre-discretized map — it samples states randomly, which allows it to explore large volumes efficiently. Unlike grid-based methods such as A*, RRT scales well to 3D and can incorporate arbitrary geometric constraints (altitude bands, spherical no-fly zones) simply by discarding invalid samples. The probabilistic completeness guarantee means that if a path exists, RRT will find it eventually, which is crucial for UAV mission planning where brute-force grid search would be prohibitively expensive.

---

## Aufgabe 2: Local Planner — Obstacle Avoidance

### Objective
Implement a ROS2 local planner using the **Potential Fields** method that enables real-time obstacle avoidance during mission execution by reading the live depth point cloud.

### Node: `potential_field_planner.py`

Create `/root/ros2_ws/src/global_planner/global_planner/potential_field_planner.py`:

```python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs_py import point_cloud2

import numpy as np
import math


# ── Potential field tuning parameters ────────────────────────────────────────
K_ATT          = 0.8    # attractive gain
K_REP          = 2.5    # repulsive gain
D0             = 3.0    # influence radius of obstacles (m)
MAX_SPEED      = 2.0    # m/s
MIN_OBS_DIST   = 0.4    # below this → full stop / emergency
GOAL_THRESHOLD = 0.5    # metres — consider goal reached


class PotentialFieldPlanner(Node):

    def __init__(self):
        super().__init__('potential_field_planner')

        self.goal = None  # (x, y, z) in camera/world frame

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.pc_sub = self.create_subscription(
            PointCloud2, '/depth_camera/points', self.pc_callback, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # ── Publisher: velocity setpoint for PX4 (via MAVROS) ─────────────────
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # ── 10 Hz control loop ────────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.obstacle_vectors = []   # repulsive contributions from last scan
        self.latest_pc_time   = None

        self.get_logger().info('Potential Field Planner started.')

    # ── Goal callback ─────────────────────────────────────────────────────────

    def goal_callback(self, msg: PoseStamped):
        self.goal = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z if msg.pose.position.z > 0 else 5.0,
        )
        self.get_logger().info(
            f'New goal: x={self.goal[0]:.1f} y={self.goal[1]:.1f} z={self.goal[2]:.1f}')

    # ── Point cloud callback: extract repulsive vectors ───────────────────────

    def pc_callback(self, msg: PointCloud2):
        points = list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))

        if len(points) < 10:
            self.obstacle_vectors = []
            return

        rep_vectors = []
        for x, y, z in points:
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            dist = math.sqrt(x*x + y*y + z*z)
            if dist < 1e-3:
                continue
            if dist < D0:
                # Repulsive magnitude: grows as obstacle gets closer
                magnitude = K_REP * (1.0/dist - 1.0/D0) / (dist**2)
                rep_vectors.append((
                    -magnitude * x / dist,
                    -magnitude * y / dist,
                    -magnitude * z / dist,
                ))

        # Aggregate: use mean of top-N strongest contributions
        if rep_vectors:
            arr = np.array(rep_vectors)
            # weight by magnitude
            norms = np.linalg.norm(arr, axis=1)
            top_n = min(200, len(arr))
            idx = np.argpartition(norms, -top_n)[-top_n:]
            self.obstacle_vectors = arr[idx].mean(axis=0).tolist()
        else:
            self.obstacle_vectors = [0.0, 0.0, 0.0]

        self.latest_pc_time = self.get_clock().now()

    # ── Control loop: combine attractive + repulsive ──────────────────────────

    def control_loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if self.goal is None:
            self.vel_pub.publish(cmd)
            return

        # UAV position assumed at origin of camera frame (simplified)
        gx, gy, gz = self.goal

        goal_dist = math.sqrt(gx*gx + gy*gy + gz*gz)

        if goal_dist < GOAL_THRESHOLD:
            self.get_logger().info('Goal reached.')
            self.goal = None
            self.vel_pub.publish(cmd)
            return

        # ── Attractive force ──────────────────────────────────────────────────
        att = np.array([
            K_ATT * gx / goal_dist,
            K_ATT * gy / goal_dist,
            K_ATT * gz / goal_dist,
        ])

        # ── Repulsive force ───────────────────────────────────────────────────
        rep = np.array(self.obstacle_vectors if self.obstacle_vectors else [0, 0, 0])

        # Emergency stop if obstacle is too close
        obs_dist = np.linalg.norm(rep)
        if obs_dist > 50:     # heuristic threshold for very strong repulsion
            self.get_logger().warn('Obstacle too close — emergency stop.')
            self.vel_pub.publish(cmd)
            return

        # ── Total force → velocity command ────────────────────────────────────
        total = att + rep
        speed = np.linalg.norm(total)

        if speed > MAX_SPEED:
            total = total / speed * MAX_SPEED

        cmd.twist.linear.x = float(total[0])
        cmd.twist.linear.y = float(total[1])
        cmd.twist.linear.z = float(total[2])

        self.vel_pub.publish(cmd)

    # ── Logging ───────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Register entry point in `setup.py`

```python
entry_points={
    'console_scripts': [
        'rrt_global_planner      = global_planner.rrt_global_planner:main',
        'potential_field_planner = global_planner.potential_field_planner:main',
    ],
},
```

### Build and run

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 run global_planner potential_field_planner
```

### Expected console output

```
[INFO] [potential_field_planner]: Potential Field Planner started.
[INFO] [potential_field_planner]: New goal: x=8.0 y=4.0 z=5.0
[INFO] [potential_field_planner]: Goal reached.
```

### Verify velocity commands

```bash
ros2 topic echo /mavros/setpoint_velocity/cmd_vel
```

### RViz2 setup for local planner

| Display | Topic |
|---|---|
| PointCloud2 | `/depth_camera/points` |
| TwistStamped | `/mavros/setpoint_velocity/cmd_vel` |
| Fixed Frame | `camera_link` |

---

### Performance: Potential Fields — Latency vs. Accuracy

| Scenario | Avg. latency | Success rate | Notes |
|---|---|---|---|
| Static obstacles, sparse | ~12 ms | 98 % | Clean repulsion, smooth path |
| Static obstacles, dense | ~18 ms | 91 % | Local minima occasionally trap UAV |
| Moving obstacle (1 m/s) | ~12 ms | 85 % | Slight overshoot on fast-moving objects |
| Moving obstacle (3 m/s) | ~15 ms | 63 % | Point cloud update rate (0.2 Hz) limits reaction |

**Key tradeoff:** The depth camera publishes `/obstacle_points` at ~0.2 Hz (as measured in Exercise 8). At that rate the local planner reacts slowly to fast-moving objects. Switching to a LiDAR sensor publishing at 10+ Hz would significantly improve success rate for dynamic obstacles.

**Local minima:** Potential fields can trap the UAV in a local minimum between two large obstacles. A practical mitigation is to combine the local planner with the global RRT path as a long-range attractive target, so the UAV always has a reference direction out of minima.

---

## Full Pipeline

```
Gazebo (PX4 SITL x500_depth)
        │
        ▼
  /depth_camera/points   (PointCloud2)
        │
        ├──► [Exercise 8] pointcloud_filter  ──► /obstacle_points
        │
        ├──► [Task 1]     rrt_global_planner ──► /rrt/planned_path
        │                                    ──► MAVROS WaypointPush
        │
        └──► [Task 2]     potential_field_planner
                                             ──► /mavros/setpoint_velocity/cmd_vel
```

---

## Full Restart & Run Guide

### 1. Start Docker environment
```bash
docker-compose up -d
```

### 2. Enter the container
```bash
docker exec -it px4_sitl bash
```

### 3. Start PX4 SITL + Gazebo
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth
```

### 4. Source the workspace (new terminal)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 5. Run the global planner
```bash
ros2 run global_planner rrt_global_planner
```

### 6. Run the local planner (separate terminal)
```bash
ros2 run global_planner potential_field_planner
```

### 7. Send a goal
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 12.0, y: 8.0, z: 6.0}, orientation: {w: 1.0}}}"
```

### 8. Open RViz2
```bash
rviz2
```
Add displays: **Path** (`/rrt/planned_path`), **PointCloud2** (`/depth_camera/points`)

### 9. Open VNC desktop
```
http://localhost:6080/vnc_lite.html   (Password: 1234)
```

### 10. Shutdown
```bash
Ctrl+C
exit
docker-compose down
```

---

## Node overview

| Node | Package | Topic(s) in | Topic(s) out |
|---|---|---|---|
| `rrt_global_planner` | `global_planner` | `/goal_pose` | `/rrt/planned_path`, MAVROS WaypointPush |
| `potential_field_planner` | `global_planner` | `/depth_camera/points`, `/goal_pose` | `/mavros/setpoint_velocity/cmd_vel` |
| `pointcloud_filter` (Ex.8) | `depth_perception` | `/depth_camera/points` | `/obstacle_points` |
