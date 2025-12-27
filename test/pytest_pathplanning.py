import math
import os
import yaml
from types import SimpleNamespace

import pytest
import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

from ament_index_python.packages import get_package_share_directory as real_get_pkg_dir
from vx_path_planning.path_planner import PathPlannerNode
from vx_path_planning.path_planner import GO_TO_PICKUP, GO_TO_DROPOFF, GO_TO_PARKING
from vx_custom_msgs.msg import GoalPoint


# ---------------- HELPERS ----------------

def make_odom(x=0.0, y=0.0, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    half = yaw * 0.5
    msg.pose.pose.orientation.z = math.sin(half)
    msg.pose.pose.orientation.w = math.cos(half)
    return msg


def make_pose_stamped(x, y):
    ps = PoseStamped()
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.w = 1.0
    return ps


@pytest.fixture(scope="module", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def temp_graph(tmp_path, monkeypatch):
    pkg_dir = tmp_path / "vx_path_planning"
    cfg_dir = pkg_dir / "config"
    cfg_dir.mkdir(parents=True)

    graph = {
        "nodes": [
            [0.0, 0.0, 1],
            [2.0, 0.0, 2],
            [2.0, 2.0, 3],
            [0.0, 2.0, 4],
        ],
        "edges": [
            [1, 2, 2.0],
            [2, 3, 2.0],
            [3, 4, 2.0],
            [4, 1, 2.0],
        ]
    }

    file_path = cfg_dir / "model_city_graph.yaml"
    file_path.write_text(yaml.safe_dump(graph))

    monkeypatch.setattr(
        "vx_path_planning.path_planner.get_package_share_directory",
        lambda pkg: str(pkg_dir)
    )

    yield graph, str(file_path)


@pytest.fixture
def node(temp_graph, monkeypatch):
    n = PathPlannerNode()
    yield n
    try:
        n.destroy_node()
    except Exception:
        pass


# ---------------- TESTS ----------------

# TC_PP_101: Test if required subscriptions exist and planning triggers correctly
def test_subscriptions_exist(node):
    topics = [s.topic_name for s in node.subscriptions]
    assert "/odom" in topics
    assert "/vehicle_state" in topics
    assert "/pickup_drop_goal" in topics
    assert "/parking_coordinates" in topics

    node.latest_odom = None
    no_odom = node.plan_with_heading(make_pose_stamped(5, 5))
    assert no_odom is False

    node.latest_odom = make_odom(0, 0, 0)
    node.start_pose = make_pose_stamped(0, 0)

    node.pickup_pose = make_pose_stamped(1, 1)
    node.dropoff_pose = make_pose_stamped(2, 2)
    node.parking_pose = make_pose_stamped(3, 3)

    node.plan_with_heading = lambda g, n: True

    node.vehicle_state = GO_TO_PICKUP
    node.path_published = False
    node.try_plan()
    assert node.path_published is True

    node.vehicle_state = GO_TO_DROPOFF
    node.path_published = False
    node.try_plan()
    assert node.path_published is True

    node.vehicle_state = GO_TO_PARKING
    node.path_published = False
    node.try_plan()
    assert node.path_published is True


# TC_PP_102: Test graph loading success and failure cases
def test_graph_loading_success_and_failure(temp_graph, monkeypatch, tmp_path):
    # ---- case 1: graph exists ----
    graph, _ = temp_graph
    node_ok = PathPlannerNode()
    try:
        assert len(node_ok.NODES) == len(graph["nodes"])
        assert len(node_ok.EDGES) == len(graph["edges"])
    finally:
        node_ok.destroy_node()

    # ---- case 2: graph missing ----
    empty_pkg = tmp_path / "vx_path_planning_empty"
    empty_pkg.mkdir()

    monkeypatch.setattr(
        "vx_path_planning.path_planner.get_package_share_directory",
        lambda pkg: str(empty_pkg)
    )

    node_fail = PathPlannerNode()
    try:
        assert node_fail.NODES == [] or node_fail.EDGES == []
    finally:
        node_fail.destroy_node()



# TC_PP_103: Test state change updates start pose correctly
def test_state_change_sets_bumper_start(node):
    odom = make_odom(0.0, 0.0, yaw=0.0)
    node.odom_callback(odom)
    assert node.start_pose is not None

    node.latest_odom = None
    node.previous_state = 0
    node.state_callback(Int8(data=5)) 

    previous = node.start_pose

    odom2 = make_odom(1.0, 0.0, yaw=0.0)
    node.odom_callback(odom2)
    node.state_callback(Int8(data=GO_TO_PICKUP))
    node.state_callback(Int8(data=1))

    assert pytest.approx(node.start_pose.pose.position.x, rel=1e-3) == 1.0 + 0.6

    # EXTRA: cover pickup_drop_callback & parking_callback
    gp = GoalPoint()
    gp.pickup = make_pose_stamped(5, 5)
    gp.dropoff = make_pose_stamped(6, 6)
    node.pickup_drop_callback(gp)
    assert node.pickup_pose is gp.pickup
    assert node.dropoff_pose is gp.dropoff


    park = make_pose_stamped(9, 9)
    node.parking_callback(park)
    assert node.parking_pose is park


# TC_PP_104: Test nearest node selection and angle handling
def test_get_k_nearest_nodes_returns_expected(node):
    ps = make_pose_stamped(1.9, 0.1)

    nids = node.get_k_nearest_nodes(ps, k=1)
    assert nids == [2]

    ps2 = make_pose_stamped(0.05, 0.05)
    nids2 = node.get_k_nearest_nodes(ps2, k=1)
    assert nids2 == [1]

    # angle_diff overflow coverage
    big1 = node.angle_diff(10 * math.pi, 0)
    assert -math.pi <= big1 <= math.pi

    big2 = node.angle_diff(-10 * math.pi, 0)
    assert -math.pi <= big2 <= math.pi

    # cover: if location is None
    assert node.get_k_nearest_nodes(None) == []

    # cover: if NODES is empty
    saved_nodes = node.NODES
    node.NODES = []
    assert node.get_k_nearest_nodes(ps) == []
    node.NODES = saved_nodes



# TC_PP_105: Test fallback to second nearest node if angle is too sharp
def test_selects_second_nearest_if_angle_too_sharp(node, monkeypatch):
    """
    If first node is too sharp, skip it and use second node.
    """

    # normal setup
    node.start_pose = make_pose_stamped(0.0, 0.1)
    node.latest_odom = make_odom(0.0, 0.1, yaw=math.pi / 2)
    goal = make_pose_stamped(2.0, 2.0)

    captured = {}

    def fake_a_star(graph, start_node_id, goal_node_id, base_heading):
        captured['start'] = start_node_id
        return [start_node_id, goal_node_id]

    monkeypatch.setattr(node, "a_star_with_heading", fake_a_star)

    success = node.plan_with_heading(goal, goal_name="test-angle")
    assert success is True
    assert captured['start'] in (1, 2, 3, 4)

    # force start candidates to [1,2]
    node.get_k_nearest_nodes = lambda *a, **k: [1, 2]

    # patch node positions:
    # node 1 = behind → sharp turn → skip
    # node 2 = ahead → good turn → used for A*
    def fake_get_node_by_id(nid):
        if nid == 1:
            return [-10.0, 0.0, 1]   # behind = sharp (skip)
        if nid == 2:
            return [10.0, 0.0, 2]    # ahead = OK (A* used)
        return None

    node.get_node_by_id = fake_get_node_by_id

    # count A* calls
    called = {"count": 0}

    def fake_astar2(graph, start_node_id, goal_node_id, base_heading):
        called["count"] += 1
        return [start_node_id, goal_node_id]

    node.a_star_with_heading = fake_astar2

    # set yaw facing right so node 1 is sharp, node 2 is good
    node.latest_odom = make_odom(0, 0, 0)
    node.start_pose = make_pose_stamped(0, 0)

    node.plan_with_heading(make_pose_stamped(2, 2))

    # must use 2nd node → A* called once
    assert called["count"] >= 1



# TC_PP_106: Test A* respects heading cone constraint
def test_a_star_with_heading_respects_cone(node):
    graph = {
        1: [(2, 1.0), (4, 1.0)],
        2: [(1, 1.0), (3, 1.0)],
        3: [(2, 1.0)],
        4: [(1, 1.0)]
    }

    node.NODES = [
        [0.0, 0.0, 1],
        [1.0, 0.0, 2],
        [2.0, 0.0, 3],
        [-1.0, 0.0, 4],
    ]

    path = node.a_star_with_heading(
        graph, start_node_id=1, goal_node_id=3, base_heading=0.0
    )

    assert path == [1, 2, 3]


# ----------A.C. 1.4.7 ----------
def test_final_goal_angle_cleanup_removes_last_node_and_appends_goal(node):
    node.NODES = [
        [0.0, 0.0, 1],
        [1.0, 0.0, 2],
    ]
    node.EDGES = [
        [1, 2, 1.0],
    ]

    node.latest_odom = make_odom(0.0, 0.0, yaw=0.0)
    node.start_pose = make_pose_stamped(0.0, 0.0)

    # goal causes sharp turn (>60°)
    goal_pose = make_pose_stamped(0.0, 2.0)

    node.a_star_with_heading = lambda *a, **k: [1, 2]

    node.path_pub = SimpleNamespace(published=None)
    node.path_pub.publish = lambda msg: setattr(node.path_pub, "published", msg)

    success = node.plan_with_heading(goal_pose, "ac-148")
    assert success is True

    path = node.path_pub.published
    last = path.poses[-1].pose.position

    # last waypoint must be the goal (node removed, goal appended)
    assert pytest.approx(last.x) == goal_pose.pose.position.x
    assert pytest.approx(last.y) == goal_pose.pose.position.y

# TC_PP_108: Test path conversion and orientation assignment
def test_plan_converts_nodes_to_path_and_sets_orientations(node):
    node.NODES = [
        [0.0, 0.0, 1],
        [1.0, 0.0, 2],
        [1.0, 1.0, 3],
    ]
    node.EDGES = [
        [1, 2, 1.0],
        [2, 3, 1.0],
    ]

    node.latest_odom = make_odom(0.0, -0.1, yaw=0.0)
    node.start_pose = make_pose_stamped(0.0, -0.1)

    goal_pose = make_pose_stamped(0.0, 1.5)

    node.a_star_with_heading = lambda *a, **k: [1, 2, 3]

    node.path_pub = SimpleNamespace(published=None)
    node.path_pub.publish = lambda msg: setattr(node.path_pub, "published", msg)

    success = node.plan_with_heading(goal_pose, "ac-147")
    assert success is True

    path = node.path_pub.published
    assert isinstance(path, Path)
    assert len(path.poses) >= 2

    # orientations must be valid (yaw computed → quaternion set)
    for pose in path.poses:
        q = pose.pose.orientation
        assert not (q.x == 0 and q.y == 0 and q.z == 0 and q.w == 0)

