"""Path planner ROS2 node."""
#!/usr/bin/env python3

import os
import math
import heapq
import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from vx_custom_msgs.msg import GoalPoint
from ament_index_python.packages import get_package_share_directory


GO_TO_PICKUP = 1
GO_TO_DROPOFF = 3
GO_TO_PARKING = 5


class PathPlannerNode(Node):
    """Path planner node that computes heading-constrained paths on a pre-defined graph."""

    def __init__(self):
        super().__init__('vx_path_planning')

        # publishers / subscribers
        self.path_pub = self.create_publisher(Path, '/path_data', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(GoalPoint, '/pickup_drop_goal', self.pickup_drop_callback, 10)
        self.create_subscription(Int8, '/vehicle_state', self.state_callback, 10)
        self.create_subscription(PoseStamped, '/parking_coordinates', self.parking_callback, 10)

        # state
        self.latest_odom = None              # Odometry
        self.start_pose = None               # PoseStamped (bumper pose)
        self.pickup_pose = None              # PoseStamped
        self.dropoff_pose = None             # PoseStamped
        self.parking_pose = None             # PoseStamped
        self.vehicle_state = 0
        self.previous_state = 0
        self.path_published = False

        # graph
        self.NODES = []   # list of (x, y, id)                # pylint: disable=invalid-name
        self.EDGES = []   # list of (from_id, to_id, cost)    # pylint: disable=invalid-name

        try:
            graph_file = os.path.join(
                get_package_share_directory('vx_path_planning'),
                'config',
                'model_city_graph.yaml'
            )
            with open(graph_file, "r", encoding="utf-8") as f:
                graph_data = yaml.safe_load(f)
            self.NODES = graph_data.get('nodes', [])
            self.EDGES = graph_data.get('edges', [])
            self.get_logger().info(
                f"Loaded graph : {len(self.NODES)} nodes, {len(self.EDGES)} edges"
            )

        except (FileNotFoundError, yaml.YAMLError, OSError) as exc:
            self.get_logger().warning(f"Could not load graph YAML: {exc}")

    # ---------- bumper transform ----------
    def transform_to_bumper_pose(self, odom_msg: Odometry) -> PoseStamped:
        """Return PoseStamped 0.6 m in front of vehicle in heading direction."""
        yaw = self.quaternion_to_yaw(odom_msg.pose.pose.orientation)

        dx = 0.6 * math.cos(yaw)
        dy = 0.6 * math.sin(yaw)

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = odom_msg.pose.pose.position.x + dx
        ps.pose.position.y = odom_msg.pose.pose.position.y + dy
        ps.pose.position.z = odom_msg.pose.pose.position.z
        ps.pose.orientation = odom_msg.pose.pose.orientation
        return ps

    # ---------- A* with heading ----------
    def a_star_with_heading(self, graph, start_node_id, goal_node_id, base_heading):
        """Run A* where state = (node_id, heading_angle) with a heading constraint."""
        half_cone_rad = math.radians(65.0)

        start_state = (start_node_id, base_heading)

        # Priority queue: (cost + heuristic, state)
        frontier = []
        heapq.heappush(frontier, (0.0, start_state))
        came_from = {start_state: None}
        cost_so_far = {start_state: 0.0}
        goal_node = self.get_node_by_id(goal_node_id)

        while frontier:
            _, current_state = heapq.heappop(frontier)
            current_node, current_heading = current_state

            if current_node == goal_node_id:
                # reconstruct node-id path
                path = []
                state_iter = current_state
                while state_iter is not None:
                    path.append(state_iter[0])
                    state_iter = came_from.get(state_iter)
                path.reverse()  # Nodes are appended from goal to start, so reverse them
                return path

            curr_node_pos = self.get_node_by_id(current_node)
            if curr_node_pos is None:
                continue
            curr_xy = (float(curr_node_pos[0]), float(curr_node_pos[1]))

            for neighbor_id, edge_cost in graph.get(current_node, []):
                nei_node_pos = self.get_node_by_id(neighbor_id)
                if nei_node_pos is None:
                    continue

                # Position of neighbor node and the angle from current node
                nei_xy = (float(nei_node_pos[0]), float(nei_node_pos[1]))
                seg_angle = self.angle_between(curr_xy, nei_xy)

                # heading constraint at every step
                if current_heading is not None:
                    if not self.within_cone(current_heading, seg_angle, half_cone_rad):
                        continue

                # After moving, heading becomes the segment direction
                new_heading = seg_angle
                new_state = (neighbor_id, new_heading)
                new_cost = cost_so_far[current_state] + edge_cost

                # Update if this path is better
                if new_state not in cost_so_far or new_cost < cost_so_far[new_state]:
                    cost_so_far[new_state] = new_cost
                    h_val = self.heuristic(nei_node_pos, goal_node)
                    priority = new_cost + h_val
                    heapq.heappush(frontier, (priority, new_state))
                    came_from[new_state] = current_state

        return None  # no feasible path for this start node


    #------------ Helpers -----------
    def select_start_and_goal_nodes(self, goal_pose, vehicle_yaw):
        """Select nearest valid graph nodes for start and goal."""
        # goal node
        goal_candidates = self.get_k_nearest_nodes(goal_pose, k=1, base_heading=None, cone_deg=None)
        goal_node_id = goal_candidates[0]

        # start nodes
        start_candidates = self.get_k_nearest_nodes(
            self.start_pose,
            k=2,
            base_heading=vehicle_yaw,
            cone_deg=65.0,
        )

        return goal_node_id, start_candidates

    def convert_nodes_to_poses(self, node_ids):
        """Convert a list of node IDs into PoseStamped waypoints."""
        poses = []
        for node_id in node_ids:
            ps = self.node_to_pose(node_id)
            if ps:
                poses.append(ps)
        return poses

    def fix_path_start_and_goal(self, poses, goal_pose):
        """Ensure path starts at bumper and ends cleanly at the goal pose."""
        # Add bumper start if missing
        if self.start_pose is not None:
            bx = float(self.start_pose.pose.position.x)
            by = float(self.start_pose.pose.position.y)
            if not self.point_is_on_any_node(bx, by):
                poses.insert(0, self.make_pose_from_xy(bx, by))

        # Final goal fix
        gx = float(goal_pose.pose.position.x)
        gy = float(goal_pose.pose.position.y)
        half_cone_rad = math.radians(65.0)

        if len(poses) >= 2:
            prev_pose = poses[-2]
            last_pose = poses[-1]
            prev_xy = (prev_pose.pose.position.x, prev_pose.pose.position.y)
            last_xy = (last_pose.pose.position.x, last_pose.pose.position.y)
            goal_xy = (gx, gy)

            dir_prev_last = self.angle_between(prev_xy, last_xy)
            dir_last_goal = self.angle_between(last_xy, goal_xy)

            # If angle is bad → remove last point
            if abs(self.angle_diff(dir_prev_last, dir_last_goal)) > half_cone_rad:
                poses.pop(-1)

        # Add final goal if needed
        if not self.point_is_on_any_node(gx, gy, eps=0.05):
            poses.append(self.make_pose_from_xy(gx, gy))

        return poses


    # ---------- Graph helpers ----------
    def get_node_by_id(self, node_id):
        """Return node (x, y, id) for given node_id."""
        try:
            return next(n for n in self.NODES if int(n[2]) == int(node_id))
        except StopIteration:
            return None

    def build_adjacency(self):
        """Build undirected adjacency list from edges."""
        graph = {}
        seen = set()
        for frm, to, cost in self.EDGES:
            fi, ti = int(frm), int(to)
            edge_cost = float(cost)
            if (fi, ti) not in seen:
                graph.setdefault(fi, []).append((ti, edge_cost))
                seen.add((fi, ti))
            if (ti, fi) not in seen:
                graph.setdefault(ti, []).append((fi, edge_cost))
                seen.add((ti, fi))
        return graph

    def heuristic(self, node_a, node_b):
        """Heuristic distance between two graph nodes."""
        x1, y1, _ = float(node_a[0]), float(node_a[1]), node_a[2]
        x2, y2, _ = float(node_b[0]), float(node_b[1]), node_b[2]
        return math.hypot(x1 - x2, y1 - y2)

    def get_k_nearest_nodes(self, location: PoseStamped, k=2, base_heading=None, cone_deg=65.0):
        """Return up to k nearest node IDs, optionally constrained by heading cone."""
        if location is None or not self.NODES:
            return []

        px = float(location.pose.position.x)
        py = float(location.pose.position.y)

        # collect all nodes with distances and angles
        candidates = []
        for x_val, y_val, node_id in self.NODES:
            nx = float(x_val)
            ny = float(y_val)
            dist = math.hypot(px - nx, py - ny)
            ang = math.atan2(ny - py, nx - px)  # angle from pose -> node
            candidates.append((dist, int(node_id), ang))

        # sort by distance
        candidates.sort(key=lambda cand: cand[0])

        # if no heading constraint requested, return nearest k
        if base_heading is None or cone_deg is None:
            return [node_id for _, node_id, _ in candidates[:k]]

        # limit by heading cone first
        half_cone_rad = math.radians(cone_deg)
        filtered = []
        for dist, node_id, ang in candidates:
            if abs(self.angle_diff(ang, base_heading)) <= half_cone_rad and dist <=1.6:
                filtered.append((dist, node_id))
                if len(filtered) >= k:
                    break

        if filtered:
            # return the k nearest that satisfied heading (already in distance order)
            return [node_id for _, node_id in filtered[:k]]

        # fallback: no node satisfied heading -> return nearest k nodes
        return [node_id for _, node_id, _ in candidates[:k]]

    def node_to_pose(self, node_id):
        """Convert a node id to a PoseStamped in map frame."""
        node = self.get_node_by_id(node_id)
        if node is None:
            return None
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(node[0])
        ps.pose.position.y = float(node[1])
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        return ps

    # ---------- Geometry helpers ----------
    def quaternion_from_yaw(self, yaw):
        """Return quaternion (x, y, z, w) for a planar yaw angle."""
        half = yaw * 0.5
        qx = 0.0
        qy = 0.0
        qz = math.sin(half)
        qw = math.cos(half)
        return (qx, qy, qz, qw)

    def quaternion_to_yaw(self, q):
        """Convert quaternion to planar yaw angle."""
        x_val, y_val, z_val, w_val = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w_val * z_val + x_val * y_val)
        cosy_cosp = 1.0 - 2.0 * (y_val * y_val + z_val * z_val)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_between(self, point_a, point_b):
        """Return angle from point_a to point_b in radians."""
        dx = point_b[0] - point_a[0]
        dy = point_b[1] - point_a[1]
        return math.atan2(dy, dx)

    def angle_diff(self, angle_a, angle_b):
        """Return normalized difference angle_a - angle_b within [-pi, pi]."""
        diff = angle_a - angle_b
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff

    def within_cone(self, base_angle, target_angle, half_cone_rad):
        """Return True if target_angle is within cone around base_angle."""
        return abs(self.angle_diff(target_angle, base_angle)) <= half_cone_rad

    def make_pose_from_xy(self, x_val, y_val):
        """Create a PoseStamped at (x, y) in map frame with identity orientation."""
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x_val)
        ps.pose.position.y = float(y_val)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        return ps

    def point_is_on_any_node(self, x_val, y_val, eps=0.05):
        """Return True if (x, y) is within eps of any node."""
        for node_x, node_y, _ in self.NODES:
            if math.hypot(float(node_x) - x_val, float(node_y) - y_val) <= eps:
                return True
        return False

    # --------- Callbacks ----------
    def odom_callback(self, msg: Odometry):
        """Odometry callback storing latest odom and triggering planning."""
        self.latest_odom = msg
        if self.start_pose is None:
            self.start_pose = self.transform_to_bumper_pose(msg)
            self.try_plan()

    def pickup_drop_callback(self, msg: GoalPoint):
        """Callback to receive pickup and dropoff goal poses."""
        # assume msg.pickup and msg.dropoff are PoseStamped
        self.pickup_pose = msg.pickup
        self.dropoff_pose = msg.dropoff
        self.try_plan()

    def parking_callback(self, msg: PoseStamped):
        """Callback to receive parking goal pose."""
        self.parking_pose = msg
        self.try_plan()

    def state_callback(self, msg: Int8):
        """Vehicle state callback, triggers replanning on state change."""
        new_state = msg.data
        if new_state != self.previous_state:     # if diff from previous
            self.vehicle_state = new_state
            self.path_published = False          # False the path_published
            if self.latest_odom:
                # make bumper a start pose
                self.start_pose = self.transform_to_bumper_pose(self.latest_odom)
            else:
                self.get_logger().warn("No Odom received, cannot set start_pose.")
            self.previous_state = new_state
            self.try_plan()

    def try_plan(self):
        """Try to run planning once start pose and goals are available."""
        if not self.start_pose or self.path_published:
            return

        if self.vehicle_state == GO_TO_PICKUP and self.pickup_pose:
            if self.plan_with_heading(self.pickup_pose, "pickup"):
                self.path_published = True

        elif self.vehicle_state == GO_TO_DROPOFF and self.dropoff_pose:
            if self.plan_with_heading(self.dropoff_pose, "dropoff"):
                self.path_published = True

        elif self.vehicle_state == GO_TO_PARKING and self.parking_pose:
            if self.plan_with_heading(self.parking_pose, "parking"):
                self.path_published = True

    def plan_with_heading(self, goal_pose: PoseStamped, goal_name="goal"):
        """
        Planner using multi-start + A* with heading constraint
        """
        if self.latest_odom is None:
            self.get_logger().warn("No odom available for heading computation.")
            return False

        # --- helper: pick start + goal nodes ---
        vehicle_yaw = self.quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
        goal_node_id, start_candidates = self.select_start_and_goal_nodes(goal_pose, vehicle_yaw)

        # build graph
        graph = self.build_adjacency()

        # --- run A* from acceptable start node ---
        best_path_node_ids = None

        for start_node_id in start_candidates:
            start_node = self.get_node_by_id(start_node_id)
            if start_node is None:
                continue

            cx = float(self.start_pose.pose.position.x)
            cy = float(self.start_pose.pose.position.y)
            sx = float(start_node[0])
            sy = float(start_node[1])

            needed_angle = math.atan2(sy - cy, sx - cx)

            path_node_ids = self.a_star_with_heading(
                graph,
                start_node_id,
                goal_node_id,
                base_heading=needed_angle,
            )

            if path_node_ids is not None:
                best_path_node_ids = path_node_ids
                break

        if best_path_node_ids is None:
            self.get_logger().error(
                "No feasible path: nearest start nodes require too sharp turns."
            )
            return False

        self.get_logger().info(
            f"Heading-constrained A* (nodes: {'->'.join(str(n) for n in best_path_node_ids)})"
                            )

        # --- helper: convert node-path to poses ---
        poses = self.convert_nodes_to_poses(best_path_node_ids)

        # --- helper: add bumper + clean goal ---
        poses = self.fix_path_start_and_goal(poses, goal_pose)

        # --- Compute orientations ---
        for idx, way_point in enumerate(poses):
            if idx < len(poses) - 1:        # For n - 1 waypoints
                dx = poses[idx + 1].pose.position.x - way_point.pose.position.x
                dy = poses[idx + 1].pose.position.y - way_point.pose.position.y
                yaw = math.atan2(dy, dx)
            else:
                if idx > 0:                 # For last waypoint
                    dx = way_point.pose.position.x - poses[idx - 1].pose.position.x
                    dy = way_point.pose.position.y - poses[idx - 1].pose.position.y
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = vehicle_yaw       # If only 1 waypoint

            qx, qy, qz, qw = self.quaternion_from_yaw(yaw)
            way_point.pose.orientation.x = qx
            way_point.pose.orientation.y = qy
            way_point.pose.orientation.z = qz
            way_point.pose.orientation.w = qw

        # publish
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = poses
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path to {goal_name} ({len(path_msg.poses)} poses).")

        return True

def main(args=None):
    """Entry point for the path planner node."""
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
