# Copyright 2026 Dosymzhan Birzhanuly
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The URDF, the controller configs and the launch defaults must agree on the wheel joints."""

import importlib.util
import os
import xml.etree.ElementTree as ElementTree

import pytest
import xacro
import yaml

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_launch_defaults():
    path = os.path.join(PACKAGE, 'launch', 'bringup.launch.py')
    spec = importlib.util.spec_from_file_location('bringup_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.DEFAULTS


@pytest.mark.parametrize('wheels', ['2', '4'])
def test_joints_match_controllers_and_motor_lists(wheels):
    defaults = load_launch_defaults()[wheels]
    urdf = xacro.process_file(
        os.path.join(PACKAGE, 'urdf', 'robot.urdf.xacro'),
        mappings={'wheels': wheels, **defaults}).toxml()
    robot = ElementTree.fromstring(urdf)

    control = robot.find('ros2_control')
    joints = [joint.get('name') for joint in control.findall('joint')]
    assert len(joints) == int(wheels)
    urdf_joints = {joint.get('name') for joint in robot.findall('joint')}
    assert set(joints) == urdf_joints

    params = {param.get('name'): param.text for param in control.find('hardware')}
    for name in ('motor_ids', 'motor_directions', 'motor_trims'):
        assert len(params[name].split(',')) == len(joints), name

    with open(os.path.join(PACKAGE, 'config', f'controllers_{wheels}wd.yaml')) as file:
        diff_cont = yaml.safe_load(file)['diff_cont']['ros__parameters']
    configured = diff_cont['left_wheel_names'] + diff_cont['right_wheel_names']
    assert sorted(configured) == sorted(joints)
