"""Exercise actual launch scoping without starting ROS nodes or moving a robot."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


class RvizScopeTest(unittest.TestCase):
    def test_demo_does_not_override_parent_rviz(self):
        path = Path(__file__).resolve().parents[1] / 'launch/waypoint_test.launch.py'
        spec = importlib.util.spec_from_file_location('waypoint_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            child = Path(directory) / 'demo.launch.py'
            child.write_text('from launch import LaunchDescription\n'
                             'from launch.actions import DeclareLaunchArgument\n'
                             'def generate_launch_description():\n'
                             '    return LaunchDescription([DeclareLaunchArgument("use_rviz", default_value="true")])\n')
            for demo in ('false', 'true'):
                for rviz in ('false', 'true'):
                    with self.subTest(start_demo=demo, use_rviz=rviz):
                        started = []

                        def fake_node(**kwargs):
                            def record(context):
                                started.append(kwargs['package'])
                                return []
                            return OpaqueFunction(function=record, condition=kwargs.get('condition'))

                        with patch.object(module, 'Node', side_effect=fake_node), patch.object(
                            module, 'PythonLaunchDescriptionSource',
                            side_effect=lambda _: PythonLaunchDescriptionSource(str(child))
                        ):
                            description = module.generate_launch_description()
                        service = LaunchService()
                        service.include_launch_description(LaunchDescription([
                            SetLaunchConfiguration('start_demo', demo),
                            SetLaunchConfiguration('use_rviz', rviz),
                            *description.entities,
                        ]))
                        self.assertEqual(service.run(), 0)
                        self.assertEqual(started.count('rviz2'), int(rviz == 'true'))
                        self.assertEqual(started.count('fairino_waypoint_test'), 1)


if __name__ == '__main__':
    unittest.main()
