"""Parses the committed Nav2 config (config/nav2.yaml) and navigate.launch.py
directly - deterministic, no sim, no live launch graph - same "no live
human/AI reading of raw telemetry" spirit as verify_navigation.py, but for
static config regressions instead of runtime behavior. Same style as
test_probe_behavior.py/test_localization_state.py (small targeted assertions,
no framework), adapted to config/XML parsing since there is no pure-logic
class in Phase 1.

Requires a sourced ROS2 Jazzy environment with Nav2 installed
(ros-jazzy-navigation2, per M7's Notable risks note) for the
ament_index_python/launch/launch_ros imports and the default-BT-XML lookup
below - run via pytest/colcon test on rambla-vm, not on a plain Mac shell.
"""
import glob
import importlib.util
import os
import unittest
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAV2_CONFIG_PATH = os.path.join(PKG_DIR, 'config', 'nav2.yaml')
NAVIGATE_LAUNCH_PATH = os.path.join(PKG_DIR, 'launch', 'navigate.launch.py')

# Common Nav2/turtlebot stock defaults this package's real footprint must
# NOT match - a regression back to "whatever Nav2 defaults to" would pass
# silently otherwise. (0.22 = nav2_costmap_2d's own hardcoded fallback,
# 0.105 = the stock turtlebot3 burger radius often copy-pasted into configs.)
STOCK_DEFAULT_RADII = {0.22, 0.105}

# base_diameter/2 + bumper_thickness/2 = 0.349/2 + 0.010/2 = 0.1795 (see
# rambla_description/urdf/params.xacro and the robot_radius comment in
# config/nav2.yaml).
EXPECTED_ROBOT_RADIUS_M = 0.1795


def _load_nav2_config():
    with open(NAV2_CONFIG_PATH) as f:
        return yaml.safe_load(f)


def _import_navigate_launch():
    spec = importlib.util.spec_from_file_location(
        'navigate_launch', NAVIGATE_LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _find_default_navigate_to_pose_bt():
    """Resolves nav2_bt_navigator's packaged default NavigateToPose BT (no
    default_nav_to_pose_bt_xml override in nav2.yaml, so this is the file
    bt_navigator actually runs). Matched by a 'recovery' filename hint
    rather than one exact hardcoded name, since the precise filename has
    drifted across ROS2 distros; still deterministic given the installed
    nav2_bt_navigator package.
    """
    bt_navigator_share = get_package_share_directory('nav2_bt_navigator')
    behavior_trees_dir = os.path.join(bt_navigator_share, 'behavior_trees')
    candidates = sorted(
        glob.glob(os.path.join(behavior_trees_dir, 'navigate_to_pose*.xml')))
    if not candidates:
        raise AssertionError(
            f'no navigate_to_pose*.xml found under {behavior_trees_dir}')
    recovery_named = [c for c in candidates if 'recovery' in os.path.basename(c).lower()]
    return recovery_named[0] if recovery_named else candidates[0]


class TestNav2CostmapConfig(unittest.TestCase):

    def setUp(self):
        self.config = _load_nav2_config()

    def test_local_costmap_has_live_scan_obstacle_layer(self):
        params = self.config['local_costmap']['local_costmap']['ros__parameters']
        self.assertIn('obstacle_layer', params['plugins'])
        obstacle_layer = params['obstacle_layer']
        self.assertEqual(
            obstacle_layer['plugin'], 'nav2_costmap_2d::ObstacleLayer')
        sources = obstacle_layer['observation_sources']
        # observation_sources is a space-separated string of source names
        # (nav2_costmap_2d convention), each with its own sub-block.
        source_names = sources.split()
        self.assertIn('scan', source_names)
        self.assertEqual(obstacle_layer['scan']['topic'], '/scan')
        self.assertTrue(params['rolling_window'])

    def test_global_costmap_has_static_layer(self):
        params = self.config['global_costmap']['global_costmap']['ros__parameters']
        self.assertIn('static_layer', params['plugins'])
        self.assertEqual(
            params['static_layer']['plugin'], 'nav2_costmap_2d::StaticLayer')

    def test_footprint_is_real_robot_geometry_not_a_default(self):
        global_radius = self.config['global_costmap']['global_costmap'][
            'ros__parameters']['robot_radius']
        local_radius = self.config['local_costmap']['local_costmap'][
            'ros__parameters']['robot_radius']
        for radius in (global_radius, local_radius):
            self.assertNotIn(radius, STOCK_DEFAULT_RADII)
            self.assertAlmostEqual(radius, EXPECTED_ROBOT_RADIUS_M, places=4)

    def test_bt_navigator_uses_sim_time_and_odom_topic(self):
        params = self.config['bt_navigator']['ros__parameters']
        self.assertTrue(params['use_sim_time'])
        self.assertEqual(params['odom_topic'], '/odometry/filtered')

    def test_behavior_server_declares_spin_backup_wait(self):
        params = self.config['behavior_server']['ros__parameters']
        for behavior in ('spin', 'backup', 'wait'):
            self.assertIn(behavior, params['behavior_plugins'])


class TestControllerCmdVelRemap(unittest.TestCase):
    """Guards against a regression that would let Nav2 write /cmd_vel or
    /cmd_vel_raw directly and bypass rambla_safety."""

    def setUp(self):
        self.navigate_launch = _import_navigate_launch()

    def test_controller_remaps_cmd_vel_to_nav_only_topic(self):
        self.assertEqual(
            self.navigate_launch.CONTROLLER_CMD_VEL_TOPIC, '/cmd_vel_nav')
        self.assertIn(
            ('cmd_vel', '/cmd_vel_nav'), self.navigate_launch.CONTROLLER_REMAPPINGS)

    def test_controller_never_targets_cmd_vel_or_cmd_vel_raw(self):
        forbidden = {'/cmd_vel', '/cmd_vel_raw'}
        for _, target in self.navigate_launch.CONTROLLER_REMAPPINGS:
            self.assertNotIn(target, forbidden)

    def test_lifecycle_manager_drives_all_four_nav2_nodes(self):
        self.assertEqual(
            set(self.navigate_launch.NAV2_LIFECYCLE_NODE_NAMES),
            {'controller_server', 'planner_server', 'behavior_server', 'bt_navigator'},
        )


class TestDefaultBehaviorTreeHasRecoverySubtree(unittest.TestCase):
    """Proves the recovery scope decision (Nav2's built-in spin/backup/wait
    via the default BT, no custom BT authored for M7) is actually wired,
    not just assumed - parses the real installed default BT XML."""

    def test_default_bt_wraps_navigation_in_a_recovery_node(self):
        bt_path = _find_default_navigate_to_pose_bt()
        tree = ET.parse(bt_path)
        root = tree.getroot()

        recovery_nodes = root.findall('.//RecoveryNode')
        self.assertTrue(
            recovery_nodes,
            f'no RecoveryNode found in default BT {bt_path}')

        # At least one RecoveryNode's subtree must reference the
        # spin/backup/wait behaviors declared in nav2.yaml's behavior_server.
        recovery_actions = set()
        for recovery_node in recovery_nodes:
            for elem in recovery_node.iter():
                tag_and_id = {elem.tag, elem.attrib.get('ID', '')}
                recovery_actions.update(tag_and_id)

        for expected in ('Spin', 'BackUp', 'Wait'):
            self.assertIn(
                expected, recovery_actions,
                f'{expected} not found under any RecoveryNode in {bt_path}')


if __name__ == '__main__':
    unittest.main()
