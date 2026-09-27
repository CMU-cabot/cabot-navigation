# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: MIT
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from cabot_ui.event import NavigationEvent
from cabot_ui.status import State

spec = importlib.util.spec_from_file_location(
    'summons_ui_manager', Path(__file__).resolve().parents[1] / 'scripts/cabot_ui_manager.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SummonsSpeedTest(unittest.TestCase):
    def setUp(self):
        self.manager = object.__new__(module.CabotUIManager)
        self.manager._logger = Mock()
        self.manager._status_manager = SimpleNamespace(state=State.idle, set_state=Mock())
        self.manager._navigation = Mock()
        self.manager._interface = Mock()
        self.manager._touchModeProxy = Mock()
        self.manager._userSpeedEnabledProxy = Mock()
        for proxy in [self.manager._touchModeProxy, self.manager._userSpeedEnabledProxy]:
            proxy.wait_for_service.return_value = True
            proxy.call.return_value = SimpleNamespace(success=True)

    def summon(self):
        self.manager._process_navigation_event(NavigationEvent('summons', 'room324'))

    def test_summons_stops_on_touch_and_keeps_user_speed_enabled(self):
        self.summon()
        self.assertFalse(self.manager._touchModeProxy.call.call_args.args[0].data)
        self.assertTrue(self.manager._userSpeedEnabledProxy.call.call_args.args[0].data)
        self.manager._navigation.set_destination.assert_called_once_with('room324')
        self.manager._status_manager.set_state.assert_called_once_with(State.in_summons)

    def test_normal_navigation_keeps_both_controls_enabled(self):
        self.manager._process_navigation_event(NavigationEvent('destination', 'room324'))
        self.assertTrue(self.manager._touchModeProxy.call_async.call_args.args[0].data)
        self.assertTrue(self.manager._userSpeedEnabledProxy.call_async.call_args.args[0].data)

    def test_summons_does_not_start_if_user_speed_cannot_be_enabled(self):
        self.manager._userSpeedEnabledProxy.call.return_value.success = False
        self.summon()
        self.manager._navigation.set_destination.assert_not_called()
        self.assertIsNone(self.manager.destination)

    def test_summons_does_not_start_if_a_required_service_is_missing(self):
        for attribute in ['_touchModeProxy', '_userSpeedEnabledProxy']:
            with self.subTest(service=attribute):
                self.setUp()
                getattr(self.manager, attribute).wait_for_service.return_value = False
                self.summon()
                self.manager._navigation.set_destination.assert_not_called()


if __name__ == '__main__':
    unittest.main()
