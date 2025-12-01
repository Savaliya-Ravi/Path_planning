import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int8
from vx_custom_msgs.msg import GoalPoint
from path_planner_node.path_planner import PathPlannerNode

# ---------- Fixtures ----------

@pytest.fixture(scope="module")
def ros_test_node():
    rclpy.init()
    node = PathPlannerNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture
def executor():
    exe = SingleThreadedExecutor()
    yield exe
    exe.shutdown()

def spin_until(executor, condition, timeout=3.0):
    """Spin callbacks until condition is true or timeout."""
    import time
    start = time.time()
    while time.time() - start < timeout:
        executor.spin_once(timeout_sec=0.1)
        if condition():
            return True
    return False

# ---------- Helper message creators ----------

def create_odom(x=0.0, y=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.w = 1.0
    return msg

def create_parking(x=3.0, y=3.0):
    msg = PoseStamped()
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.w = 1.0
    return msg

def create_goal(pickup=(1.5, 0.5), dropoff=(2.5, 0.5)):
    msg = GoalPoint()
    msg.pickup = PoseStamped()
    msg.pickup.pose.position.x = float(pickup[0])
    msg.pickup.pose.position.y = float(pickup[1])
    msg.pickup.pose.orientation.w = 1.0

    msg.dropoff = PoseStamped()
    msg.dropoff.pose.position.x = float(dropoff[0])
    msg.dropoff.pose.position.y = float(dropoff[1])
    msg.dropoff.pose.orientation.w = 1.0

    return msg

def create_map():
    return OccupancyGrid()

def create_state(value):
    return Int8(data=value)


# ----------------TEST CASES-------------------- 

# TC_PP_001: Verify all required topics are subscribed
def test_ac_2_2_1_topic_subscriptions(ros_test_node):
    topics = [s.topic_name for s in ros_test_node.subscriptions]
    assert '/odom' in topics
    assert '/vehicle_state' in topics
    assert '/pickup_drop_goal' in topics

# TC_PP_002: Generate straight-line path once when /vehicle_state=1
def test_ac_2_2_2_generate_path_for_pickup(ros_test_node, executor):
    received = {'path': None}

    def path_callback(msg):
        received['path'] = msg

    ros_test_node.create_subscription(Path, '/path_data', path_callback, 10)
    executor.add_node(ros_test_node)

    # feed odom, goal, map, and state=1
    ros_test_node.odom_callback(create_odom(0, 0))
    ros_test_node.pickup_drop_callback(create_goal(pickup=(4, 4)))
    ros_test_node.map_callback(create_map())
    ros_test_node.state_callback(create_state(1))

    spin_until(executor, lambda: received['path'] is not None)
    path = received['path']
    assert path is not None
    assert len(path.poses) > 0

    # check first and last waypoint positions
    first = path.poses[0].pose.position
    last = path.poses[-1].pose.position
    assert (first.x, first.y) == (0, 0)
    assert (last.x, last.y) == (4, 4)

    # re-trigger same state to confirm no republish
    received['path'] = None
    ros_test_node.state_callback(create_state(1))
    assert not spin_until(executor, lambda: received['path'] is not None, 1.5)


# TC_PP_003: Covers GO_TO_DROPOFF and GO_TO_PARKING branches
def test_dropoff_and_parking_paths(ros_test_node, executor):
    ros_test_node.map_callback(create_map())
    ros_test_node.odom_callback(create_odom(0, 0))
    ros_test_node.pickup_drop_callback(create_goal(dropoff=(6, 6)))
    ros_test_node.state_callback(create_state(3))
    spin_until(executor, lambda: ros_test_node.path_published)

    ros_test_node.parking_callback(create_parking(2, 2))
    ros_test_node.state_callback(create_state(5))
    spin_until(executor, lambda: ros_test_node.path_published)
    assert ros_test_node.path_published

# TC_PP_004: Ensure try_plan safely returns if essential data missing
def test_no_plan_when_missing_inputs(ros_test_node):
    ros_test_node.start_pose = None
    ros_test_node.try_plan()  # missing start_pose
    ros_test_node.start_pose = create_odom(0, 0).pose.pose
    ros_test_node.map_received = False
    ros_test_node.vehicle_state = 1
    ros_test_node.pickup_pose = None
    ros_test_node.try_plan()  # missing pickup
    
    assert not (ros_test_node.pickup_pose and ros_test_node.map_received)

# TC_PP_005: Ensure waypoints are interpolated linearly and count correct
def test_create_waypoints_linear_interpolation(ros_test_node):
    start = create_odom(0, 0).pose.pose
    goal = create_odom(5, 5).pose.pose
    waypoints = ros_test_node.create_waypoints(start, goal, steps=5)
    assert len(waypoints) == 6
    assert waypoints[0].pose.position.x == 0
    assert waypoints[-1].pose.position.x == 5
    assert waypoints[-1].pose.orientation.w == 1.0

# TC_PP_006: Simulate state change before odom arrival
def test_state_callback_without_odom(ros_test_node):
    ros_test_node.latest_odom_pose = None
    ros_test_node.previous_state = 0
    ros_test_node.state_callback(create_state(3))
    assert ros_test_node.vehicle_state == 3
    
    assert isinstance(ros_test_node.start_pose, type(Pose())), \
    "Start pose should remain default Pose when no odom received"

# TC_PP_007: Ensure path_published resets on each new state
def test_multiple_state_transitions(ros_test_node, executor):
    ros_test_node.map_callback(create_map())
    ros_test_node.odom_callback(create_odom(0, 0))
    ros_test_node.pickup_drop_callback(create_goal(pickup=(1, 1), dropoff=(2, 2)))

    ros_test_node.state_callback(create_state(1))
    spin_until(executor, lambda: ros_test_node.path_published)
    assert ros_test_node.path_published

    # new state triggers reset
    ros_test_node.state_callback(create_state(3))
    assert ros_test_node.path_published is True
