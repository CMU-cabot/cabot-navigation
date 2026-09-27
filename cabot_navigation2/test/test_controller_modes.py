# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: MIT
import importlib.util
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('bringup', PACKAGE / 'launch/bringup_launch.py')
bringup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bringup)


class ControllerConfigurationTest(unittest.TestCase):
    def test_all_controller_blocks_are_loaded(self):
        with tempfile.TemporaryDirectory() as temp:
            with patch.object(bringup, 'launch_config', SimpleNamespace(log_dir=temp)):
                config = yaml.safe_load(Path(bringup.online_controller_params(str(PACKAGE))).read_text())
            params = config['controller_server']['ros__parameters']
            for filename, controller, planner in bringup.CONTROLLERS.values():
                self.assertIn(controller, params['controller_plugins'])
                source = yaml.safe_load((PACKAGE / 'params' / filename).read_text())
                self.assertEqual(params[controller], source['controller_server']['ros__parameters'][controller])
                self.assertIn(planner, config['planner_server']['ros__parameters']['planner_plugins'])

    def test_switching_bt_and_restore_static(self):
        source = PACKAGE.parent / 'cabot_bt/behavior_trees/navigation.xml'
        with tempfile.TemporaryDirectory() as temp:
            target = Path(temp) / 'behavior_trees/navigation.xml'
            target.parent.mkdir()
            target.write_text(source.read_text())
            with patch.object(bringup, 'get_package_share_directory', return_value=temp):
                for online, mode in [('1', 'crowdattn'), ('1', 'follow'), ('0', 'mpc'), ('0', 'follow')]:
                    with patch.dict(os.environ, CABOT_CONTROLLER_SWITCHING=online):
                        bringup.sync_navigation_bt_controller(mode)
                    root = ET.parse(target).getroot()
                    self.assertEqual(len(root.findall('.//SelectController')), int(online))
                    for follow in root.findall('.//FollowPath'):
                        expected = '{selected_controller}' if online == '1' else bringup.CONTROLLERS[mode][1]
                        self.assertEqual(follow.attrib['controller_id'], expected)
                    for planner in root.findall('.//ComputePathToPose'):
                        expected = '{selected_planner}' if online == '1' else bringup.CONTROLLERS[mode][2]
                        self.assertEqual(planner.attrib['planner_id'], expected)


if __name__ == '__main__':
    unittest.main()
