# agx_arm_motion

MoveIt2 convenience layer for Piper Studio. Provides motion primitives for
the arm that higher-level packages (`agx_arm_manipulation`, VLAs, etc.) can
consume without touching MoveItPy directly.

## Executables

| Executable | When to use |
|---|---|
| `move_to_pose` | One-shot: supply a Cartesian goal via launch args, arm moves once, process exits |
| `move_to_named_pose` | One-shot: look up a pose by name from `config/example_poses.yaml` |
| `pose_goal_server` | Persistent server: subscribes to a ROS 2 topic, executes every incoming pose |

All three accept a `profile:=sim|real` argument (default `sim`).

---

## Quick-start (simulation)

### Step 1 — Build

```bash
cd ~/projects/ros2_projects/piper_studio
colcon build --packages-select agx_arm_motion agx_arm_gzsim
source install/setup.bash
```

### Step 2 — Start the simulation (Terminal 1)

```bash
ros2 launch agx_arm_gzsim piper_with_gripper_moveit_gzsim.launch.py
```

Wait until you see:

```
[move_group]: You can start planning now!
```

### Step 3 — Move the arm (Terminal 2)

**One-shot Cartesian goal (top-down, 30 cm forward, 25 cm high):**

```bash
ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.30 y:=0.0 z:=0.25 pitch:=1.5708
```

**Named pose:**

```bash
ros2 launch agx_arm_motion move_to_named_pose.launch.py pose_name:=ready
```

---

## move_to_pose — arguments

| Argument | Default | Description |
|---|---|---|
| `profile` | `sim` | `sim` (Gazebo) or `real` (hardware) |
| `x` | `0.25` | Target X in metres (forward) |
| `y` | `0.0` | Target Y in metres (left +) |
| `z` | `0.30` | Target Z in metres (up +) |
| `roll` | `0.0` | Roll about X-axis (rad) |
| `pitch` | `1.5708` | Pitch about Y-axis (rad); π/2 = top-down |
| `yaw` | `0.0` | Yaw about Z-axis (rad) |
| `frame_id` | `base_link` | TF frame for the target pose |

```bash
# Horizontal forward reach:
ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.32 y:=0.0 z:=0.18 pitch:=0.0

# Reach to the left, top-down:
ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.22 y:=0.15 z:=0.15 pitch:=1.5708

# Goal in a custom TF frame (frame must exist in the TF tree):
ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.1 z:=0.05 frame_id:=camera_link

# Real hardware:
ros2 launch agx_arm_motion move_to_pose.launch.py profile:=real x:=0.30 z:=0.25
```

---

## move_to_named_pose — arguments

| Argument | Default | Description |
|---|---|---|
| `profile` | `sim` | `sim` or `real` |
| `pose_name` | `ready` | Key in `config/example_poses.yaml` |

Available named poses: `ready`, `bench_center_above`, `bench_center_pick`,
`front_horizontal`, `left_side`.  Edit `config/example_poses.yaml` to add more.

```bash
ros2 launch agx_arm_motion move_to_named_pose.launch.py pose_name:=bench_center_above
```

---

## pose_goal_server — runtime topic

### Start the server (Terminal 2)

```bash
ros2 launch agx_arm_motion pose_goal_server.launch.py
```

Wait for:

```
[pose_goal_server]: Ready. Publish geometry_msgs/PoseStamped to /piper/target_pose.
```

### Send a goal (Terminal 3)

```bash
ros2 topic pub --once /piper/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link},
    pose: {position: {x: 0.30, y: 0.0, z: 0.25},
           orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}}"
```

### Monitor status

```bash
ros2 topic echo /piper/motion_status
```

Publishes: `IDLE` → `PLANNING` → `EXECUTING` → `SUCCESS` (or `FAILED`) → `IDLE`

Goals received while a motion is executing are **dropped** with a warning.

### From a Python node (e.g. agx_arm_manipulation)

```python
from geometry_msgs.msg import PoseStamped
import numpy as np

pose = PoseStamped()
pose.header.frame_id = "camera_link"   # any TF-known frame works
pose.pose.position.x = obj_x
pose.pose.position.y = obj_y
pose.pose.position.z = obj_z + 0.05   # 5 cm above the object

# Top-down orientation (pitch = pi/2 about Y)
half = np.pi / 4
pose.pose.orientation.y = np.sin(half)
pose.pose.orientation.w = np.cos(half)

publisher.publish(pose)
```

The server transforms the pose to `base_link` via TF2 before planning.

---

## Python API (for agx_arm_manipulation and other packages)

```python
import rclpy
from agx_arm_motion import MotionFacade

rclpy.init()
facade = MotionFacade()                        # reads MoveIt params from ROS
success = facade.move_to_pose(pose_stamped)    # pose must be in base_link
success = facade.move_to_named_state("home")   # SRDF named joint state
facade.shutdown()
rclpy.shutdown()
```

`MotionFacade` is the only MoveIt glue that `agx_arm_manipulation` (or any
other package) should import.  Do not duplicate MoveItPy setup elsewhere.

---

## Coordinate frames

```
base_link  – arm base, origin at mounting plate centre
  X  forward (into the workspace)
  Y  left
  Z  up

tcp_link   – nominal tool center point / planning target frame
  By default this is the midpoint between the gripper jaws.
  The gripper approach direction is along tcp_link's Z-axis.
```

Typical gripper orientations (RPY in radians, intrinsic XYZ):

| Approach style | roll | pitch | yaw |
|---|---|---|---|
| Top-down (bench pick) | 0.0 | π/2 ≈ 1.5708 | 0.0 |
| Horizontal forward | 0.0 | 0.0 | 0.0 |
| 45° angled down | 0.0 | π/4 ≈ 0.785 | 0.0 |

Approximate reachable workspace (arm mounted upright, not validated):

```
X  0.10 – 0.38 m   (forward)
Y –0.25 – 0.25 m   (left / right)
Z –0.05 – 0.38 m   (up / down)
```

---

## Architecture

```
agx_arm_manipulation  (task-level behaviors)
        │
        ▼  from agx_arm_motion import MotionFacade
agx_arm_motion        (this package — MoveItPy facade)
        │
        ▼  MoveItPy / MoveItCpp
agx_arm_moveit        (URDF, SRDF, kinematics, planning pipelines)
agx_arm_gzsim         (sim overlay: Gazebo URDF + controllers)
```

MoveItPy reads all configuration (URDF, SRDF, kinematics, planning pipelines)
from ROS parameters loaded at launch time by `agx_arm_motion.launch_utils.build_moveit_config`.
The node is named `moveit_py` in all launch files to match
`MoveItPy(node_name="moveit_py")`.

See [config/example_poses.yaml](config/example_poses.yaml) for documented
example positions and orientation conventions.
