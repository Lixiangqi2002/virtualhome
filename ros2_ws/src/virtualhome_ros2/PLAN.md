# VirtualHome Robot Abstraction Plan (Phase 4)

We want to wrap the VirtualHome agent as a standard ROS 2 mobile robot equipped with odometry, TF trees, and frame conventions.

## 📐 1. Coordinate System Mapping

### Core Mapping
| Axis | Unity Frame | ROS 2 Frame | Description |
| :--- | :--- | :--- | :--- |
| **X** | `pos[0]` | `position.x` | Maps absolute lateral coordinate. Unity Left/Right. |
| **Y/Z** | `pos[2]` | `position.y` | Depth coordinate in Unity mapped to ROS orthogonal plane. |
| **Height** | `pos[1]` | `0.0` (fixed) | Set to 0.0 for 2D plane navigation references. |

### Angle mapping (Yaw)
-   **Unity Orientation**: Clockwise degrees. `0` = looking Forward North (+Z axis). `90` = looking Right East (+X axis).
-   **ROS 2 Orientation**: Counter-Clockwise radians. `0` = looking Forward East (+X axis). `pi/2` = looking Left North (+Y axis).

**Conversion Formula**:
```python
# Unity Yaw degree to ROS Yaw radian
ros_yaw_rad = math.radians(90.0 - unity_yaw_deg)
```

---

## 🌳 2. Frame Tree Setup (`tf`)

We will publish the following transforms:

### 🌟 Dynamic Transforms (`odom -> base_link`)
-   **Source Node**: `vh_bridge_node` via `tf2_ros.TransformBroadcaster`.
-   Continuous translation mapping from coordinates, and rotation from mapped Quaternion.

### 📍 Static Transforms (`base_link -> camera_link` -> color optical)
-   **Offsets**:
    -   `base_link` point rests at floor level center of agent footprint.
    -   `camera_link` elevated up eye level height, e.g., `Z=1.70m`.
-   **Frames**:
    -   `base_link` -> `camera_link` (Translation: `[0.1, 0.0, 1.7]`, Rotation: `0`)
    -   `camera_link` -> `camera_depth_optical_frame` (Standard ROS camera lens offset rotation offset).

---

## 📩 3. Standard Interface Publications

We will add these topics alongside existing ones:

| Topic | Message Type | Frame ID | Description |
| :--- | :--- | :--- | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | `odom` | Holds position.x, position.y and orientation quat. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Various | Dynamic & Static link hierarchies. |

---

## 🛠️ 4. Action Items

1.  **Modify `vh_bridge_node.py`**:
    -   Incorporate `tf2_ros.TransformBroadcaster` & `tf2_ros.StaticTransformBroadcaster`.
    -   Initialize item publisher for `/odom`.
    -   Translate Unity angles correctly leveraging pure Python-backed yaw trigonometry offsets for safe coordinates.
2.  **Edit `setup.py`**: Add `nav_msgs` and `tf2_ros` into dependency nodes if needed (typically prebuilt Humble libs).

---

## 🔍 5. Verification Plan

1.  **Odometry Streams**: `ros2 topic echo /odom`
2.  **TF Tree Check**: `ros2 run tf2_tools view_frames` -> inspection generated pdf stream.
3.  **Visualization**: Load frames into RViz2 to see axis alignments visually representing forward directions.

---

# 🤖 VirtualHome Robot Standard Control & Scan (Phase 5)

We will extend the abstraction interface to support standard Nav2 components: `/cmd_vel` velocity-based drivers, and a 2D Planar laser stream `/scan` derived from depth images.

## 📈 1. Velocity Driving Kinematics (`/cmd_vel`)
-   **Subscriber**: `/cmd_vel` (`geometry_msgs/msg/Twist`)
-   **Methodology**:
    -   At high-frequency interval loops (e.g. 10-20Hz).
    -   Integrate $v_x$ (linear) and $\omega$ (angular rates) outputs into absolute global coordinate targets $(X, Y, \text{Yaw})$.
    -   Feed calculated offsets into the position-based setup loop on backend discrete handlers.

## 📡 2. Depth To 2D Laserscan Processing (`/scan`)
-   **Topic**: `/scan` (`sensor_msgs/msg/LaserScan`)
-   **Methodology**:
    -   Extract the center row (horizon) slices of the 2D depth image stream matrix.
    -   Calculate range beams correlating pixel index angular indices relying upon Unity Camera Field of Views (FOV).
-   **Static Setup**:
    -   Publish Static Transform: `base_link` $\rightarrow$ `base_scan` elevated eye-level offsets.

## 🛠️ 3. Action Items

1.  **Elevate timer speed** inside `vh_bridge_node.py` up threshold speed e.g., $10Hz$ or $20Hz$.
2.  **Add Subscriber** for `/cmd_vel` updating continuous velocity buffers securely.
3.  **Incorporate math integrations** translates velocity into targets continuously on callbacks.
4.  **Extract depth center slices** iteratively mapping `/scan` structures.

---

# 🗺️ VirtualHome Minimal Nav2 Setup (Phase 6)

We want to deploy a minimal Nav2 navigation stack using a known fake static map (No SLAM) to enable autonomous trajectory deliveries using `/cmd_vel`.

## 📂 1. Proposed Package Structure
Create a dedicated navigation config module, or integrate within `virtualhome_ros2`:
```text
virtualhome_ros2/
├── launch/
│   ├── bridge.launch.py           # Runs bridge_node
│   └── nav2_static.launch.py      # Runs Nav2 + map_server + AMCL
├── config/
│   └── nav2_params.yaml           # Costmaps/Planners configs
└── maps/
    ├── fake_room.pgm              # Pure grid-lines/Empty room
    └── fake_room.yaml             # Resolution/Origin descriptors
```

## 📐 2. Map Constants & YAML
### Advice:
- **Resolution**: $0.05$ (5cm per pixel) is standard and fine-grained enough layout.
- **Image**: $200 \times 200$ pixel square room. White inside, Black borders (1px outer walls boundary).
- **Origin**: `[-10.0, -10.0, 0.0]`. 
  - Since origin is the bottom-left corner of the $.pgm$, placing it at $[-10, -10]$ places absolute coordinate $(0,0)$ securely in the center of the free bounds, giving comfortable buffer nodes sizes representing large VirtualHome absolute agent positions.

### `maps/fake_room.yaml` Template:
```yaml
image: fake_room.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## 📍 3. AMCL and Map-to-Odom
**Should we use AMCL?**
- **Yes**: If you want canonical standard Nav2 stack operations. AMCL consumes `/scan` + `/odom` to publish the crucial `map -> odom` transform that corrects coordinate drift setup.
- **Alternatively**: For pure static anchors verification (No Drift), a `tf2_ros.StaticTransformBroadcaster` can publish $map \rightarrow odom$ fixed at zero offset $[0,0,0]$. This eliminates AMCL calculation freezes and is highly recommended as Step 1.

## 🛠️ 4. RViz Test Procedure
1.  **Fixed Frame**: Set to `map` in RViz setup panels.
2.  **Add Panels**: Add `Map` (from topic), `Odometry`, `LaserScan`, and `RobotModel`.
3.  **Align**: Agent footprint mesh node should sit resting inside white open grids.
4.  **Drive**: Click '2D Navigation Goal' top rib tool, drag forward vectors point target.
5.  **Observe**: Pathline appears. `/cmd_vel` fires updates.

---

## 🌳 5. Achieved Nav2 Stack & Component Overview

We have successfully brought up the core Nav2 architecture using a real static map. Below is the detailed roles of the active components:

### 📍 1. Map Server (`nav2_map_server`)
- **Node**: `map_server`
- **Role**: Loads the static occupancy grid map (`real_room.pgm`) based on metadata descriptors (`real_room.yaml`) and provides the `/map` topic that Nav2 costmaps consume for free/occupied mesh lookups securely.

### 📍 2. Planner Server (`nav2_planner`)
- **Node**: `planner_server`
- **Role**: Computes global collision-free paths from the agent's live pose to target goal offsets.
- **Plugin**: `GridBased` leveraging `NavfnPlanner` plugin structure configured on top operations on underlying `global_costmap` continuous setups.

### 📍 3. Controller Server (`nav2_controller`)
- **Node**: `controller_server`
- **Role**: Continuous kinematic loops following the path tracking vectors into continuous coordinates velocities outputs (`/cmd_vel` topic).
- **Plugin**: `FollowPath` using `RegulatedPurePursuitController` setup enabling smoother curve integrations avoiding abrupt angle locks. Operates using underlying `local_costmap` updates.

### 📍 4. BT Navigator (`nav2_bt_navigator`)
- **Node**: `bt_navigator`
- **Role**: Standard interface providing standard `/navigate_to_pose` Action servers setups. Coordinates planner calculation & controller continuous flows running behavior-tree nodes managing states safely without manual handlers intervention. Spreads triggers sequentially on RViz Goal tool clicks.

### 📍 5. Lifecycle Manager
- **Node**: `lifecycle_manager_navigation`
- **Role**: Transitions Nav2 stack setup steps linearly Configuring states through fully Active loops securely preserving integrity startup dependencies transparently.
