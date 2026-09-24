"""Distribution checks without ROS nodes, CAN, or motor commands."""
import ast
import runpy
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def test_launches_and_install_scripts_parse():
    for package in ('ieir_bringup', 'ieir_controllers', 'W3_ROBOT/w3_robot_bridge'):
        for path in (ROOT / package / 'launch').glob('*.py'):
            ast.parse(path.read_text(), filename=str(path))
    for path in (ROOT / 'scripts').glob('*.sh'):
        subprocess.run(['bash', '-n', str(path)], check=True)


def test_dependency_roots_and_rosidl_membership():
    for package in ('ieir_bringup', 'ieir_controllers'):
        manifest = ET.parse(ROOT / package / 'package.xml').getroot()
        deps = {d.text for d in manifest if d.tag.endswith('depend')}
        assert not {'ieir_simulation', 'mujoco', 'libglfw3-dev'} & deps
    bridge = ET.parse(ROOT / 'W3_ROBOT/w3_robot_bridge/package.xml').getroot()
    assert bridge.find('member_of_group').text == 'rosidl_interface_packages'
    assert (ROOT / 'USB2CAN/COLCON_IGNORE').exists()
    for path in (ROOT / 'description').glob('*/package.xml'):
        deps = {d.text for d in ET.parse(path).getroot() if d.tag.endswith('depend')}
        assert not {'rviz2', 'joint_state_publisher_gui', 'ieir_simulation', 'mujoco'} & deps


def test_dual_arm_mesh_assets_are_in_repository():
    packages = {ET.parse(p).getroot().findtext('name'): p.parent
                for p in (ROOT / 'description').glob('*/package.xml')}
    robot = ET.parse(packages['dual_arm_support'] / 'urdf/dual_arm_robot_plug.urdf')
    for mesh in robot.findall('.//mesh'):
        name = mesh.get('filename')
        assert name.startswith('package://'), name
        package, relative = name[len('package://'):].split('/', 1)
        assert (packages[package] / relative).is_file(), name


def test_calibration_defaults_to_web():
    for filename in ('joint_zero_calibration.py', 'joint_auto_calibration.py'):
        tree = ast.parse((ROOT / 'ieir_controllers/scripts' / filename).read_text())
        options = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                   and any(isinstance(a, ast.Constant) and a.value == '--preview-backend'
                           for a in n.args)]
        assert len(options) == 1
        assert next(k.value.value for k in options[0].keywords if k.arg == 'default') == 'web'


def test_real_models_do_not_load_simulation_hardware():
    for filename in ('dual_arm_robot.urdf', 'dual_arm_robot.urdf.xacro'):
        robot = ET.parse(ROOT / 'description/dual_arm_support/urdf' / filename).getroot()
        assert robot.find('ros2_control') is None
    robot = ET.parse(ROOT / 'ieir_controllers/config/dual_arm_ros2_control.urdf.xacro')
    assert robot.find('.//hardware/plugin').text == 'ieir_controllers/DMHardwareInterface'
    assert not (ROOT / 'ieir_controllers/config/dual_arm_sim_controllers.yaml').exists()


def test_merge_with_synthetic_calibration():
    merge = runpy.run_path(str(ROOT / 'ros2_ws_config/merge_offsets.py'))['merge']
    arms = [dict(channel=channel, offsets=[
        dict(name=f'{side}_joint_{i}', slot=i, zero_offset=0.1 * (i + 1),
             axis_sign=1 if i % 2 else -1, direction_verified=True)
        for i in range(7)]) for side, channel in [('left', 0), ('right', 1)]]
    merged = merge(*arms, {})
    assert len(merged['offsets']) == 14
    by_name = {entry['name']: entry for entry in merged['offsets']}
    assert len(by_name) == 14
    for side, channel in [('left', 0), ('right', 1)]:
        arm = arms[channel]
        assert arm['channel'] == channel and len(arm['offsets']) == 7
        assert {entry['name'] for entry in arm['offsets']} == {f'{side}_joint_{i}' for i in range(7)}
        for entry in arm['offsets']:
            result = by_name[entry['name']]
            assert result['channel'] == channel
            assert result['zero_offset'] == entry['zero_offset']
            assert result['axis_sign'] == entry['axis_sign']


def test_machine_calibration_is_ignored():
    rules = (ROOT / '.gitignore').read_text().splitlines()
    for pattern in ('joint_offsets*.yaml*', 'joint_directions*.yaml*',
                    'joint_calibration_*_reviewed.yaml*'):
        assert '/ros2_ws_config/' + pattern in rules
