[Ravikumar Shivlal Savaliya](https://git.hs-coburg.de/ravisavaliya) : Path Planning Node


## 📌 Table of Contents
- [Overview](#-overview)
- [Component Architecture](#-component-architecture)
- [ROS 2 Topics](#-ros-2-topics)
- [Component Functionalities](#️-component-functionalities)
- [Installation & Setup](#-installation--setup)
- [Interface Test Procedure](#-interface-test-procedure)

---

## 📖 Overview

The **Path Planning Node** plays a central role in ensuring that the autonomous shuttle reaches its destination safely and efficiently. It calculates the route from the shuttle's current position to destinations like pickup points, drop-off points, or parking spots.

To achieve this, it uses:
- **Position and orientation** data from `/odom`.
- **Environmental map** from `/static_map`.
- **Pick-up and drop-off destination** via `/goal`.
- **Vehicle state signal** from `/vehicle_state`.
- **Parking location** from `/parking_coordinates`.

Based on this, it generates a sequence of waypoints and publishes them on `/path_data`, which can then be followed by a controller node.

---

## 🏗️ Component Architecture

![Path Planning](resource/path%20planning.png)


## 🔌 ROS 2 Topics

| IN/Out | Topic Name             | Message Type                | Description                                                     |
|--------|------------------------|-----------------------------|-----------------------------------------------------------------|
| Input  | `/odom`                | `nav_msgs/Odometry`         | Current pose and orientation of the shuttle.                    |
| Input  | `/goal`                | `goalPoints`                | Custom message with pickup/drop-off coordinates.                |
| Input  | `/vehicle_state`       | `std_msgs/UInt8`            | Trigger signal from decision unit to start path planning.       |
| Input  | `/parking_coordinates` | `geometry_msgs/PoseStamped` | Nearest available parking spot to plan toward.                  |
| Output | `/path_data`           | `nav_msgs/Path`             | Computed path as a list of waypoints for the controller to use. |

### 📦 Custom Message Definition: `goalPoints`

The `/goal` topic uses a custom message type named `goalPoints`, which includes both pickup and drop-off locations for the autonomous shuttle.

```text
goalPoints.msg
------------------------
geometry_msgs/PoseStamped pickup
geometry_msgs/PoseStamped droppoff
```

### Field Description

| Field Name | Type                        | Description                                         |
| ---------- | --------------------------- | --------------------------------------------------- |
| `pickup`   | `geometry_msgs/PoseStamped` | Location where the user wants to be picked up.      |
| `droppoff` | `geometry_msgs/PoseStamped` | Destination location where the user is dropped off. |



## ⚙️ Component Functionalities

The **Path Planning Node** only initiates path computation when it receives a specific trigger signal on the `/vehicle_state` topic, which typically comes from the decision unit. By doing so, the node avoids unnecessary computation and ensures efficient use of resources.

Once triggered, the node begins by reading the shuttle's real-time pose and orientation from the `/odom` topic. At the same time, it listens to the `/goal` topic, which contains the destination coordinates (a pickup or drop-off point) specified by the user through the shuttle interface(HMI).

After shuttle reachs to the supermarket, it searches for the empty parking space in its surrounding and when it finds one, the sensor data fusion and filtering module publishes the `PoseStamped` messaege to the `/parking_coordinates` topic, and path planning node subscribes to it and caluculate efficient path to the parking. 

To ensure safety and feasibility, the node uses the static environment map from `/static_map`, which contains information about obstacles and wall. Using this occupancy grid, the path planner ensures that the generated path is safe for execution.

Once the path is computed, it is published to the `/path_data` topic as a `nav_msgs/Path` message. This message consists of a sequence of waypoints that the path execution controller uses to guide the shuttle through the environment.


## 📥 Installation & Setup

### 🔧 Clone the repository

```bash
git clone https://git.hs-coburg.de/voyagex/vx_path_planning.git
```

##  Build the package
```bash
cd vx_path_planning
colcon build
source install/setup.bash

```

 ## Run the node
```bash
ros2 run path_planner_node path_planner
```

 ## 🧪 Interface Test Procedure

This section outlines how to verify that the **Path Planning Node** correctly subscribes to inputs and publishes the expected path output using ROS 2 topics.


### Step-by-Step Test Procedure

1. **Launch the Node**
   ```bash
   ros2 run path_planner_node path_planner
   ```
Now create dummy publisher by pasting this each in new terminal.

2. **Publish Odometry Input**
   ```bash
   ros2 topic pub /odom nav_msgs/Odometry '{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}'
   ```
3. **Publish Goal Input**
   ```bash
   ros2 topic pub /goal goalPoints '{pickup: {header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}, droppoff: {header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 5.0}, orientation: {w: 1.0}}}}'
   ```
4. **Publish Vehicle State**
   ```bash
   ros2 topic pub /vehicle_state std_msgs/UInt8 'data: 1'
   ```
5. **Publish Parking Coordinates**
   ```bash
   ros2 topic pub /parking_coordinates geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 2.0}, orientation: {w: 1.0}}}'
   ```
6. **Verify Output Path**
   ```bash
   ros2 topic echo /path_data
   ```
