#!/usr/bin/env python3

# Import required message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int8
from vx_custom_msgs.msg import GoalPoint

# Graph and A* utilities
import os
import math
import heapq
import yaml
from ament_index_python.packages import get_package_share_directory

# Define states for easier readability
GO_TO_PICKUP = 1
GO_TO_DROPOFF = 3
GO_TO_PARKING = 5

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('vx_path_planning')  # Initialize node name

        # Publisher to output the planned path
        self.path_pub = self.create_publisher(Path, '/path_data', 10)

        # Subscribers to receive required data
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(GoalPoint, '/pickup_drop_goal', self.pickup_drop_callback, 10)
        self.create_subscription(Int8, '/vehicle_state', self.state_callback, 10)
        self.create_subscription(PoseStamped, '/parking_coordinates', self.parking_callback, 10)

        # Internal state storage
        self.latest_odom_pose = None    # Constantly updated latest pose from odometry (for reference)
        self.start_pose = None          # Pose used as the starting point for planning
        self.pickup_pose = None         # Target pickup pose (Pose type)
        self.dropoff_pose = None        # Target dropoff pose (Pose type)
        self.parking_pose = None        # Parking pose (PoseStamped type, plan logic uses Pose from its 'pose' field)
        self.vehicle_state = 0          # Current state
        self.previous_state = 0         # Store previous state to detect a change
        self.path_published = False     # Flag to track if path has been generated for current state

        # Graph structures (loaded from YAML)
        self.NODES = []         # list of (x,y,id)
        self.EDGES = []         # list of (from_id,to_id,cost)

        # Load graph file from package share (wrap in try/except)
        try:
            graph_file = os.path.join(
                get_package_share_directory('vx_path_planning'),  # replace with actual name
                'config',
                'model_city_graph.yaml'
            )
            
            with open(graph_file, 'r') as f:
                graph_data = yaml.safe_load(f)
            self.NODES = graph_data.get('nodes', [])
            self.EDGES = graph_data.get('edges', [])
            self.SPECIAL_NODES = graph_data.get('special_nodes', {})
            self.get_logger().info(f"Loaded graph with {len(self.NODES)} nodes and {len(self.EDGES)} edges.")
        except Exception as e:
            self.get_logger().warn(f"Could not load graph YAML: {e}. A* planning will be unavailable until graph is fixed.")
            # keep NODES/EDGES empty so planner will fallback or log errors

        self.get_logger().info("Path Planner Node initialized.")

    # Utility: A* on loaded graph
    def heuristic(self, node_a, node_b):
        # node_a/node_b are tuples (x,y,id)
        x1, y1, _ = node_a
        x2, y2, _ = node_b
        return math.hypot(x1 - x2, y1 - y2)

    def a_star_search(self, start_id, goal_id):
        if not self.NODES or not self.EDGES:
            return []

        # build adjacency
        graph = {}
        seen_edges = set()
        for frm, to, cost in self.EDGES:
            frm_i, to_i = int(frm), int(to)
            cost_f = float(cost)

            if (frm_i, to_i) not in seen_edges:
                graph.setdefault(frm_i, []).append((to_i, cost_f))
                seen_edges.add((frm_i, to_i))

            if (to_i, frm_i) not in seen_edges:
                graph.setdefault(to_i, []).append((frm_i, cost_f))
                seen_edges.add((to_i, frm_i))

        try:
            start_node = next(n for n in self.NODES if n[2] == start_id)
            goal_node = next(n for n in self.NODES if n[2] == goal_id)
        except StopIteration:
            return []

        open_set = []
        heapq.heappush(open_set, (0, start_id))
        came_from = {}
        g_score = {node[2]: float('inf') for node in self.NODES}
        g_score[start_id] = 0
        f_score = {node[2]: float('inf') for node in self.NODES}
        f_score[start_id] = self.heuristic(start_node, goal_node)

        visited = set()
        while open_set:
            current_f, current = heapq.heappop(open_set)
            if current == goal_id:
                path = []
                node = current
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start_id)
                path.reverse()
                return path

            if current in visited:
                continue
            visited.add(current)

            for neighbor, cost in graph.get(current, []):
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    try:
                        neighbor_node = next(n for n in self.NODES if n[2] == neighbor)
                    except StopIteration:
                        continue
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor_node, goal_node)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def node_to_pose(self, node_id):
        try:
            node = next(n for n in self.NODES if n[2] == node_id)
        except StopIteration:
            return None
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(node[0])
        pose.pose.position.y = float(node[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def find_nearest_node(self, position):
        # position is a (x,y) tuple or Pose/PoseStamped
        if position is None or not self.NODES:
            return None

        if isinstance(position, tuple) or isinstance(position, list):
            px, py = position[0], position[1]
        else:
            # Pose or PoseStamped
            if hasattr(position, 'pose'):
                p = position.pose
            else:
                p = position
            px, py = p.position.x, p.position.y

        min_dist = float('inf')
        nearest_id = None
        for x, y, nid in self.NODES:
            d = math.hypot(px - x, py - y)
            if d < min_dist:
                min_dist = d
                nearest_id = nid
        return nearest_id

    # Callbacks
    # Save latest pose from odometry
    def odom_callback(self, msg):
        self.latest_odom_pose = msg.pose.pose

        # If don't have a start_pose yet, set it now
        if self.start_pose is None:
            self.start_pose = self.latest_odom_pose
            self.get_logger().info("Start pose initialized from first odom message.")
            self.try_plan()

    # Save pickup and dropoff targets and plan the path
    def pickup_drop_callback(self, msg):
        # Expecting GoalPoint with .pickup.pose and .dropoff.pose as in your code
        try:
            self.pickup_pose = msg.pickup.pose
            self.dropoff_pose = msg.dropoff.pose
        except Exception:
            # fallback if different structure
            self.get_logger().warn("Received GoalPoint with unexpected layout.")
            return
        self.try_plan()

    # Save parking target and plan the path
    def parking_callback(self, msg):
        # msg is PoseStamped
        self.parking_pose = msg.pose
        self.try_plan()

    # When vehicle state changes, reset publishing flag and store new start pose
    def state_callback(self, msg):
        new_state = msg.data

        # Reset goal poses when the state changes
        if new_state != self.previous_state:
            self.vehicle_state = new_state
            self.path_published = False  # Reset so we allow new planning

            # Store the current pose as the starting pose for the new state
            if self.latest_odom_pose:
                self.start_pose = self.latest_odom_pose
            else:
                self.get_logger().warn("No Odom received, cannot set start_pose.")

            self.get_logger().info(f"State changed from {self.previous_state} to {self.vehicle_state}.")
            self.previous_state = new_state

            self.try_plan()

    # Decide which path to plan based on current vehicle state
    def try_plan(self):
        # We must have a fixed start pose to plan a path
        if not self.start_pose:
            return

        if self.path_published:
            return  # Skip if path already published for current state

        # Plan path to pickup
        if self.vehicle_state == GO_TO_PICKUP:
            if self.pickup_pose and self.map_received:
                planned = self.plan_to_goal(self.pickup_pose, goal_name="pickup")
                if planned:
                    self.path_published = True

        # Plan path to dropoff
        elif self.vehicle_state == GO_TO_DROPOFF:
            if self.dropoff_pose and self.map_received:
                planned = self.plan_to_goal(self.dropoff_pose, goal_name="dropoff")
                if planned:
                    self.path_published = True

        # Plan path to parking
        elif self.vehicle_state == GO_TO_PARKING:
            if self.parking_pose and self.map_received:
                planned = self.plan_to_goal(self.parking_pose, goal_name="parking")
                if planned:
                    self.path_published = True

    # Generalized planning: map start_pose and goal_pose to nearest graph nodes, run A*, publish Path
    def plan_to_goal(self, goal_pose, goal_name="goal"):
        if not self.NODES or not self.EDGES:
            self.get_logger().warn("Graph not loaded: cannot run A*; falling back to straight-line interpolated waypoints.")
            # fall back to original simple method to keep behavior
            waypoints = self.create_waypoints(self.start_pose, goal_pose)
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = waypoints
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Fallback {goal_name} path published (linear).")
            return True

        start_node = self.find_nearest_node(self.start_pose)
        goal_node = self.find_nearest_node(goal_pose)

        if start_node is None or goal_node is None:
            self.get_logger().warn("Could not find nearest graph node for start or goal; falling back to linear waypoints.")
            waypoints = self.create_waypoints(self.start_pose, goal_pose)
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = waypoints
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Fallback {goal_name} path published (linear).")
            return True

        path_node_ids = self.a_star_search(start_node, goal_node)
        if not path_node_ids:
            self.get_logger().error(f"A* failed to find path from node {start_node} to {goal_node}.")
            return False

        # Convert node ids to PoseStamped
        poses = []
        for nid in path_node_ids:
            ps = self.node_to_pose(nid)
            if ps:
                poses.append(ps)

        if not poses:
            self.get_logger().error("Converted A* node path to empty PoseStamped list; aborting publish.")
            return False

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = poses
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path to {goal_name} published via A* (nodes: {' -> '.join(str(n) for n in path_node_ids)}).")
        return True

    # Original simple linear waypoint generator (kept for fallback)
    def create_waypoints(self, start: Pose, goal: Pose, steps: int = 5):
        waypoints = []
        for i in range(steps + 1):
            ratio = i / steps

            # Linearly interpolate positions
            interp_pose = Pose()
            interp_pose.position.x = start.position.x + ratio * (goal.position.x - start.position.x)
            interp_pose.position.y = start.position.y + ratio * (goal.position.y - start.position.y)
            interp_pose.position.z = 0.0  # Flat 2D plane

            # Orientation is kept fixed (no rotation planning)
            interp_pose.orientation.w = 1.0

            # Create PoseStamped with frame_id
            stamped = PoseStamped()
            stamped.header.frame_id = "map"
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.pose = interp_pose

            waypoints.append(stamped)

        return waypoints

# Main function to initialize and run the node
def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

