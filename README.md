## Contents
- [Description](#description)
- [Architecture](#architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Contributor](#contributor)
 
## Description

The Path Planning Component is an essential part of the autonomous shuttle system since it will produce secure, optimum, and obstacle-free path between the current location of the shuttle and the destinations specified by its users. These destinations can comprised of pick up points and drop off location.

To provide correct information on the location of the shuttle on the map, the component gets the real position and orientation using the topic `/odom` (`nav_msgs/Odometry`). It also subscribes to the `/static_map` topic (`nav_msgs/OccupancyGrid`), which is the static map of the environment including obstacles and places where it can drive. Its destination goal is received under `/goal` topic (`goalPoints`) that contains pick up or drop off coordinates as formulated by the user.

Further the component subscribes to `/vehicle_state` (`std_msgs/UInt8`) message, which serves as a starting point to the path planning process. Once a user has reached his destination, the shuttle must park as a user may have to shop. At that, to obtain the most appropriate near parking location, it subscribes to the `/parking_coordinates` topic (`geometry_msgs/PoseStamped`).

Playing all the required inputs, the component will publish the ready scheduled path to the `/path_data` topic (`nav_msgs/Path`). The path of this route is composed of the series of the waypoints, which can be tracked by the path execution controller.

## Architecture

```mermaid
graph LR
    Odom["/odom"] -->|"(nav_msgs/Odometry)"| PathPlanning["Path Planning Component"]
    Goal["/goal"] -->|"(goalPoints)"| PathPlanning
    Map["/static_map"] -->|"(nav_msgs/OccupancyGrid)"| PathPlanning
    Decision["/vehicle_state"] -->|"(std_msgs/UInt8)"| PathPlanning
    Parking["/parking_coordinates"] -->|"(geometry_msgs/PoseStamped)"| PathPlanning

    PathPlanning -->|"(nav_msgs/Path)"| PathData["/path_data"]

    %% Styling
    class PathPlanning yellowRounded;
    class Odom,Goal,Map,Decision,Parking,PathData graySquare;

    %% Class definitions
    classDef yellowRounded fill:#ffcc00,stroke:#333333,stroke-width:2px,rx:20,ry:20,color:#000000,font-size:12px,text-align:center;
    classDef graySquare fill:#cccccc,stroke:#333333,stroke-width:1px,rx:0,ry:0,color:#000000,font-size:12px,text-align:center;
```
##  Topics
##### Inputs

| Name                          | Type                      | Description                                                              |
|-------------------------------|---------------------------|--------------------------------------------------------------------------|
| `/odom`                       | `nav_msgs/Odometry`         | Provides the shuttle’s real-time position and orientation within the map. |
| `/goal`                       | `geometry_msgs/PoseStamped` | Receives the destination goal (pickup/drop-off or parking location). |
| `/static_map`                 | `nav_msgs/OccupancyGrid`    | Provides the static map of the environment, including obstacles. |
| `/vehicle_state`              | `std_msgs/UInt8`             | Receives a signal from the decision unit (boolean) to start path planning. |
| `/parking_coordinates`        | `geometry_msgs/PoseStamped` | Receives the parking spot coordinates to plan the path towards the parking area. |

##### Output

| Name                         | Type                      | Description                                                              |
|------------------------------|---------------------------|--------------------------------------------------------------------------|
| `/path_data`                 | `nav_msgs/Path`           | Publishes the computed path as a sequence of waypoints to be followed by the shuttle. |


<!-- ### ## User Stories & Acceptance Criteria
### User Story 1: Path Calculation from Start to Goal
**User Story 1.1**  
As a **Path Planning Component**, I want to dynamically calculate and adjust the shuttle's route based on its current position, destination, so that the shuttle can follow an optimal, and safe path to reach its destination, including parking and re-routing.
**Acceptance Criteria**  
- **1.1.1**  The system should calculate an initial valid path from the shuttle's current position (based on odometry) to the destination (either a pickup, drop-off, or parking spot), ensuring that it follows safe and efficient routes.
- **1.1.2** The system should publish a valid **`/path_data`** message representing the planned path to the destination.
- **1.1.3** The system should incorporate dynamic re-routing, allowing the shuttle to adjust its path if an obstacle is detected by the Environmental Model or Decision Unit and send a re-routing request to the Path Planning Component.



 User Story 2: Dynamic Re-routing for Obstacle Avoidance
**User Story 2.1**  
As a **Path Planning Component**, I want to dynamically re-calculate the path when the **Decision Unit** informs me of the need to re-plan the path due to a detected obstacle, so that the shuttle can avoid collisions and continue safely to its destination.

**Acceptance Criteria**  
- **2.1.1** The **Environmental Model** detects obstacles and sends a **true/false signal** to the **Decision Unit** indicating whether an obstacle is detected.
- **2.1.2** The **Decision Unit** sends a command to the **Path Planning Component** to re-plan the path when an obstacle is detected.
- **2.1.3** Upon receiving the signal from the **Decision Unit**, the **Path Planning Component** invalidates the current path and recalculates a new one that avoids the detected obstacle.
- **2.1.4** The updated path should be published to the **`/path_data`** topic, guiding the shuttle around the obstacle.

### User Story 3: Path Planning for Parking
**User Story 3.1**  
As a **Path Planning Component**, I want to plan a route that allows the shuttle to reach the nearest available parking spot, so that the shuttle can park safely while waiting for the user to complete their shopping.

**Acceptance Criteria**  
- **3.1.1** The system should use the **`/parking_coordinates`** to calculate the path to the nearest available parking spot.
- **3.1.2** The system should publish a valid path to the parking spot, guiding the shuttle safely into the parking area without collisions.
- **3.1.3**  The system should dynamically adjust the path if the parking spot becomes unavailable or if there are new obstacles detected while approaching the parking spot. -->

 
## Installation
```bash
git clone https://git.hs-coburg.de/voyagex/vx_path_planning.git
```
 
## Usage
Run the node:
```bash
ros2 run xx xx
```
 
## Contributor
[Ravikumar Shivlal Savaliya](https://git.hs-coburg.de/ravisavaliya)
 
 
