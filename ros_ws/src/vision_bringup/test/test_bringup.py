"""Launch contracts and database preservation; no camera or SLAM mocks required."""

import importlib.util
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess

import pytest
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, PACKAGE / 'launch' / (name + '.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def launches(tmp_path, monkeypatch):
    # Mirror the package's installed layout without requiring camera binaries.
    share = tmp_path / 'share'
    shutil.copytree(REPO / 'config', share / 'config')
    shutil.copytree(PACKAGE / 'rviz', share / 'rviz')
    front, slam = load_launch('zed_front'), load_launch('zed_rtabmap')
    for module in (front, slam):
        monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(share))
    return front, slam, share


def context_with_defaults(module, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def text(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def parameters(context, node):
    # Humble exposes normalized parameters internally; evaluate them with ROS's
    # own substitution/type rules instead of asserting source-code strings.
    result = {}
    for entry in evaluate_parameters(context, node._Node__parameters):
        if isinstance(entry, Path):
            for block in yaml.safe_load(entry.read_text()).values():
                result.update(block['ros__parameters'])
        else:
            result.update(entry)
    return result


@pytest.mark.parametrize('mode', ['mapping', 'localization'])
def test_existing_database_is_preserved(launches, tmp_path, mode):
    _, slam, _ = launches
    database = tmp_path / 'bedroom.db'
    with sqlite3.connect(database) as connection:
        connection.execute('CREATE TABLE evidence (id INTEGER)')
        connection.execute('INSERT INTO evidence VALUES (7)')
    before = database.read_bytes()
    assert slam.prepare_database_path(str(database), mode) == str(database)
    assert database.read_bytes() == before


def test_new_map_creates_directory_but_not_database(launches, tmp_path):
    _, slam, _ = launches
    database = tmp_path / 'new room' / 'room.db'
    assert slam.prepare_database_path(str(database), 'mapping') == str(database)
    assert database.parent.is_dir()
    assert not database.exists()


@pytest.mark.parametrize('case', ['missing', 'empty', 'directory', 'relative', 'bad_mode'])
def test_invalid_database_request_fails(launches, tmp_path, case):
    _, slam, _ = launches
    database = tmp_path / 'missing' / 'room.db'
    mode = 'localization'
    if case == 'empty':
        database = tmp_path / 'empty.db'
        database.touch()
    elif case == 'directory':
        database = tmp_path
    elif case == 'relative':
        database = Path('room.db')
        mode = 'mapping'
    elif case == 'bad_mode':
        mode = 'typo'
    with pytest.raises(ValueError):
        slam.prepare_database_path(str(database), mode)
    assert not (tmp_path / 'missing').exists()


def test_database_is_independent_of_working_directory(launches, tmp_path, monkeypatch):
    _, slam, _ = launches
    monkeypatch.setattr(Path, 'home', lambda: tmp_path)
    context = context_with_defaults(slam)
    database = context.launch_configurations['database_path']
    elsewhere = tmp_path / 'elsewhere'
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    assert slam.prepare_database_path(database, 'mapping') == str(
        tmp_path / '.ros/robosub/maps/bedroom.db')


@pytest.mark.parametrize('replay', [False, True])
@pytest.mark.parametrize('mode', ['mapping', 'localization'])
def test_slam_composition(launches, tmp_path, mode, replay):
    front, slam, share = launches
    database = tmp_path / 'room.db'
    if mode == 'localization':
        database.write_bytes(b'database preservation fixture')
    svo = tmp_path / 'room.svo2'
    svo.touch()  # Path validation only, never passed to the SDK in this test.
    context = context_with_defaults(
        slam, database_path=str(database), mode=mode, svo_path=str(svo) if replay else 'live')
    actions = slam.launch_setup(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(nodes) == 1
    node = nodes[0]
    assert text(context, node.node_package) == 'rtabmap_slam'
    params = parameters(context, node)
    assert params['publish_tf'] is True
    assert params['odom_frame_id'] == ''
    assert params['frame_id'] == 'zed_front_camera_link'
    assert params['use_sim_time'] is replay
    assert params['Mem/IncrementalMemory'] == ('true' if mode == 'mapping' else 'false')
    assert params['Mem/InitWMWithAllNodes'] == 'true'
    assert params['Mem/BinDataKept'] == 'true'
    assert params['Grid/3D'] == 'true'
    assert params['GridGlobal/MaxNodes'] == '0'
    assert params['approx_sync'] is False
    remaps = {text(context, key): text(context, value) for key, value in node._Node__remappings}
    assert remaps['odom'] == '/zed_front/zed_node/odom'
    assert remaps['rgb/camera_info'] == '/zed_front/zed_node/rgb/color/rect/camera_info'

    include = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    overrides = {key: text(context, value) for key, value in include.launch_arguments}
    camera_context = context_with_defaults(front, **dict(context.launch_configurations, **overrides))
    camera_actions = front.launch_setup(camera_context)
    driver = next(action for action in camera_actions if isinstance(action, IncludeLaunchDescription))
    driver_args = {key: text(camera_context, value) for key, value in driver.launch_arguments}
    assert driver_args['publish_tf'] == 'true'
    assert driver_args['publish_map_tf'] == 'false'
    assert driver_args['publish_urdf'] == 'false'
    assert driver_args['publish_svo_clock'] == str(replay).lower()
    assert driver_args['use_sim_time'] == 'false'
    zed = yaml.safe_load(Path(driver_args['ros_params_override_path']).read_text())['/**']['ros__parameters']
    tracking = zed['pos_tracking']
    assert tracking['imu_fusion'] is True
    assert tracking['area_memory'] is False
    assert tracking['reset_odom_with_loop_closure'] is False
    assert tracking['area_file_path'] == ''
    assert zed['mapping']['mapping_enabled'] is False
    assert zed['debug']['use_pub_timestamps'] is False
    active = [action for action in camera_actions if isinstance(action, Node) and (
        action.condition is None or action.condition.evaluate(camera_context))]
    assert [text(camera_context, action.node_package) for action in active] == [
        'robot_state_publisher', 'rviz2']
    for action in active:
        # Do not execute xacro for this contract test; evaluate its clock entry.
        clock = action._Node__parameters[0]
        clock = {key: value for key, value in clock.items() if text(camera_context, key) == 'use_sim_time'}
        assert evaluate_parameters(camera_context, [clock])[0]['use_sim_time'] is replay


def test_missing_replay_fails_before_starting_camera(launches, tmp_path):
    front, _, _ = launches
    context = context_with_defaults(front, svo_path=str(tmp_path / 'missing.svo2'))
    with pytest.raises(ValueError, match='svo_path'):
        front.launch_setup(context)


def test_rviz_shows_assembled_map_without_history(launches):
    _, _, share = launches
    manager = yaml.safe_load((share / 'rviz/rtabmap_slam.rviz').read_text())['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    clouds = [display for display in manager['Displays'] if display['Class'].endswith('/PointCloud2')]
    enabled = [display for display in clouds if display['Enabled']]
    assert len(enabled) == 1
    assert enabled[0]['Topic']['Value'] == '/rtabmap/cloud_map'
    assert enabled[0]['Decay Time'] == 0
    assert enabled[0]['Topic']['Durability Policy'] == 'Transient Local'


def test_installed_assets():
    share = Path(get_package_share_directory('vision_bringup'))
    for relative in (
        'launch/zed_front.launch.py', 'launch/zed_rtabmap.launch.py',
        'config/zed_front_rtabmap.yaml', 'config/rtabmap.yaml', 'rviz/rtabmap_slam.rviz',
    ):
        assert (share / relative).is_file(), relative


def test_rtabmap_starts_with_project_configuration(tmp_path, monkeypatch):
    """Real node smoke test: parameter types and database creation, not SLAM quality."""
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.executors import SingleThreadedExecutor

    monkeypatch.setenv('ROS_DOMAIN_ID', '219')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    executable = Path(get_package_prefix('rtabmap_slam')) / 'lib/rtabmap_slam/rtabmap'
    config = Path(get_package_share_directory('vision_bringup')) / 'config/rtabmap.yaml'
    database = tmp_path / 'smoke.db'
    output = tmp_path / 'rtabmap.log'
    with output.open('w') as log:
        process = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__ns:=/rtabmap', '-r', '__node:=rtabmap',
            '--params-file', str(config), '-p', 'database_path:=' + str(database),
        ], stdout=log, stderr=subprocess.STDOUT)
        ros_context = rclpy.context.Context()
        rclpy.init(context=ros_context)
        probe = rclpy.create_node('bringup_test', context=ros_context)
        executor = SingleThreadedExecutor(context=ros_context)
        try:
            client = probe.create_client(GetParameters, '/rtabmap/rtabmap/get_parameters')
            assert client.wait_for_service(timeout_sec=15), output.read_text()
            names = ['Grid/3D', 'Mem/BinDataKept', 'odom_frame_id', 'publish_tf',
                     'approx_sync']
            future = client.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(probe, future, executor=executor, timeout_sec=5)
            assert future.done(), output.read_text()
            values = future.result().values
            assert len(values) == len(names), output.read_text()
            assert values[0].string_value == 'true'
            assert values[1].string_value == 'true'
            assert values[2].string_value == ''
            assert values[3].bool_value is True
            assert values[4].bool_value is False
        finally:
            executor.shutdown()
            probe.destroy_node()
            rclpy.shutdown(context=ros_context)
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
    assert process.returncode == 0, output.read_text()
    assert database.is_file() and database.stat().st_size > 0
    with sqlite3.connect(database) as connection:
        assert connection.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall()
