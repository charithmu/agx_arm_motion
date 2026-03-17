# agx_arm_motion

Move the Piper arm TCP to target poses using MoveIt2 / MoveItPy.

## Two modes

| Node | When to use |
|---|---|
| `move_to_pose` | One-shot: supply a pose via launch arguments, arm moves once, process exits |
| `pose_goal_server` | Persistent server: subscribes to a ROS 2 topic, executes every pose received |

Use `move_to_pose` for manual testing or scripted moves.  
Use `pose_goal_server` when poses arrive at runtime from a perception pipeline or another node.

---

## Quick-start (simulation)

### Step 1 — Build

```bash
cd ~/projects/ros2_projects/piper_studio   # your workspace root
colcon build --packages-select agx_arm_motion agx_arm_gzsim
source install/setup.bash
```

### Step 2 — Start the simulation (Terminal 1)

Starts Gazebo, spawns the arm, starts `move_group`, and brings up all controllers:

```bash
ros2 launch agx_arm_gzsim piper_with_gripper_moveit_gzsim.launch.py
```

Wait until you see:

```
[move_group]: You can start planning now!
```

### Step 3 — Move the arm (Terminal 2)

**Top-down approach, 30 cm forward, 25 cm high:**

```bash
ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.30 y:=0.0 z:=0.25 pitch:=1.5708
```

Expected output:

```
[moveit_py]: Target TCP: pos=(0.300, 0.000, 0.250) m  rpy=(0.000, 1.571, 0.000) rad  frame=base_link
[moveit_py]: Planning...
[moveit_py]: Executing...
[moveit_py]: Done.
```

---

## move_to_pose — all arguments

| Argument | Default | Description |
|---|---|---|
| `x` | `0.25` | Target X in metres (forward) |
| `y` | `0.0` | Target Y in metres (left +) |
| `z` | `0.30` | Target Z in metres (up +) |
| `roll` | `0.0` | Roll about X-axis (rad) |
| `pitch` | `1.5708` | Pitch about Y-axis (rad); π/2 = top-down |
| `yaw` | `0.0` | Yaw about Z-axis (rad) |
| `frame_id` | `base_link` | TF frame for the target pose |

```bash
# !!values are just examples, not validated!!

# Horizontal forward reach:
ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.32 y:=0.0 z:=0.18 pitch:=0.0

# Reach to the left, top-down:
ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.22 y:=0.15 z:=0.15 pitch:=1.5708

# No arguments – moves to safe mid-air pose (x=0.25, z=0.30):
ros2 launch agx_arm_motion move_to_pose.launch.py

# Goal in a custom TF frame (frame must exist in TF tree):
ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.1 z:=0.05 frame_id:=camera_link
```

---

## pose_goal_server — runtime topic

### Start the server (Terminal 2)

```bash
ros2 launch agx_arm_motion pose_goal_server.launch.py
```

Wait for:

```
[moveit_py]: Ready. Publish geometry_msgs/PoseStamped to /piper/target_pose.
```

### Send a goal (Terminal 3) 

### !!values are just examples, not validated!!

```bash
ros2 topic pub --once /piper/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link},
    pose: {position: {x: 0.30, y: 0.0, z: 0.25},
           orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}}"
```

### Monitor status (any terminal)

```bash
ros2 topic echo /piper/motion_status
```

Publishes one of: `IDLE` → `PLANNING` → `EXECUTING` → `SUCCESS` (or `FAILED`).

### From a Python node

```python
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
import numpy as np

pose = PoseStamped()
pose.header.frame_id = "camera_link"   # any TF-known frame works
pose.pose.position.x = obj_x
pose.pose.position.y = obj_y
pose.pose.position.z = obj_z + 0.05   # 5 cm above the object

# Top-down orientation (pitch = π/2)
q = Rotation.from_euler("xyz", [0, np.pi / 2, 0]).as_quat()
pose.pose.orientation.x = q[0]
pose.pose.orientation.y = q[1]
pose.pose.orientation.z = q[2]
pose.pose.orientation.w = q[3]

publisher.publish(pose)
```

The server transforms the pose to `base_link` automatically via TF2, then plans and executes. Goals received while a motion is executing are dropped with a warning.

---

## Coordinate frames

```
base_link  – arm base, origin at mounting plate centre
  X  forward (into the workspace)
  Y  left
  Z  up

link6      – TCP / end-effector (planning target frame)
  The gripper approach direction is along link6's Z-axis.
```

Typical gripper orientations (RPY in radians, intrinsic XYZ rotation):

| Approach style | roll | pitch | yaw |
|---|---|---|---|
| Top-down (bench pick) | 0.0 | π/2 ≈ 1.5708 | 0.0 |
| Horizontal forward | 0.0 | 0.0 | 0.0 |
| 45° angled down | 0.0 | π/4 ≈ 0.785 | 0.0 |

Approximate reachable workspace (arm mounted upright) (!!not validated!!):

```
X  0.10 – 0.38 m   (forward)
Y –0.25 – 0.25 m   (left / right)
Z –0.05 – 0.38 m   (up / down)
```

---

## Architecture

MoveItPy reads all of its configuration (URDF, SRDF, kinematics, planning
pipelines) from ROS 2 parameters passed via `--params-file` by the launch
system.  The launch files use `MoveItConfigsBuilder` with an additional
`moveit_cpp.yaml` that sets `planning_pipelines.pipeline_names: [ompl]` —
the nested sub-key format that MoveItCpp actually reads.  The node is named
`moveit_py` in both launch files to match `MoveItPy(node_name="moveit_py")`.

## Reference poses

See [config/example_poses.yaml](config/example_poses.yaml) for documented
example positions with comments on workspace and orientation conventions.
