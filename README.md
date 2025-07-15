## Contents
- [Description](#description)
- [Architecture](#architecture)
- [Topics](#topics)
- [User Stories & Acceptance Criteria](#user-stories--acceptance-criteria)
- [Installation](#installation)
- [Usage](#usage)
- [License](#license)
 
## Description
The **Path Planning Component** plays a crucial role in the autonomous shuttle system by generating safe, efficient, and obstacle-free paths from the shuttle’s current location to user-designated destinations. These destinations may include pickup points, drop-off locations, or parking zones, depending on the phase of the ride.

To accurately determine the shuttle’s location on the map, the component receives real-time position and orientation data through the `/odom` topic (`nav_msgs/Odometry`). It also subscribes to the `/map` topic (`nav_msgs/OccupancyGrid`), which provides a static map of the environment, including obstacles and drivable areas. The destination goal is received via the `/goal` topic (`goalPoints`), which contains either the pickup or drop-off coordinates as defined by the user.

In addition, the component listens to the `/decision_unit` topic (`std_msgs/Bool`), which acts as a trigger to initiate the path planning process. After a user has been dropped off at their destination, the shuttle needs to park while the user completes shopping. For this purpose, it also subscribes to the `/parking_coordinates` topic (`geometry_msgs/PoseStamped`) to receive the most suitable nearby parking location.

Once all necessary inputs are processed, the component publishes the final planned route to the `/path_data` topic (`nav_msgs/Path`). This route consists of a sequence of waypoints that can be followed by the shuttle’s path execution controller to ensure smooth and safe navigation.

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
| `/map`                        | `nav_msgs/OccupancyGrid`    | Provides the static map of the environment, including obstacles. |
| `/decision_unit`              | `std_msgs/UInt8`             | Receives a signal from the decision unit (boolean) to start path planning. |
| `/parking_coordinates`        | `geometry_msgs/PoseStamped` | Receives the parking spot coordinates to plan the path towards the parking area. |

##### Output

| Name                         | Type                      | Description                                                              |
|------------------------------|---------------------------|--------------------------------------------------------------------------|
| `/path_data`                 | `nav_msgs/Path`           | Publishes the computed path as a sequence of waypoints to be followed by the shuttle. |


## User Stories & Acceptance Criteria
### User Story 1: Path Calculation from Start to Goal
**User Story 1.1**  
As a **Path Planning Component**, I want to dynamically calculate and adjust the shuttle's route based on its current position, destination, so that the shuttle can follow an optimal, and safe path to reach its destination, including parking and re-routing.
**Acceptance Criteria**  
- **1.1.1**  The system should calculate an initial valid path from the shuttle's current position (based on odometry) to the destination (either a pickup, drop-off, or parking spot), ensuring that it follows safe and efficient routes.
- **1.1.2** The system should publish a valid **`/path_data`** message representing the planned path to the destination.
- **1.1.3** The system should incorporate dynamic re-routing, allowing the shuttle to adjust its path if an obstacle is detected by the Environmental Model or Decision Unit and send a re-routing request to the Path Planning Component.



<!-- ### User Story 2: Dynamic Re-routing for Obstacle Avoidance
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
 
## License
Licensed under the **Apache 2.0 License**. See [LICENSE](LICENSE) for details.
 
 
