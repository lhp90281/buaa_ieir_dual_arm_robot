"""Distribution checks without ROS nodes, CAN, or motor commands."""
import ast
import ctypes
import runpy
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def test_launches_and_install_scripts_parse():
    for package in ('eiriarm_bringup', 'eiriarm_controllers', 'W3_ROBOT/w3_robot_bridge'):
        for path in (ROOT / package / 'launch').glob('*.py'):
            ast.parse(path.read_text(), filename=str(path))
    for path in (ROOT / 'scripts').glob('*.sh'):
        subprocess.run(['bash', '-n', str(path)], check=True)


def test_dependency_roots_and_rosidl_membership():
    mujoco = ET.parse(ROOT / 'eiriarm_mujoco/package.xml').getroot()
    deps = {d.text for d in mujoco.findall('depend')}
    assert 'libglfw3-dev' in deps
    assert not {'glfw3', 'mujoco'} & deps  # SDK is vendored; no conflicting apt ABI.
    bridge = ET.parse(ROOT / 'W3_ROBOT/w3_robot_bridge/package.xml').getroot()
    assert bridge.find('member_of_group').text == 'rosidl_interface_packages'
    assert (ROOT / 'USB2CAN/COLCON_IGNORE').exists()
    for path in (ROOT / 'description').glob('*/package.xml'):
        ET.parse(path)


def test_dual_arm_mesh_assets_are_in_repository():
    packages = {ET.parse(p).getroot().findtext('name'): p.parent
                for p in (ROOT / 'description').glob('*/package.xml')}
    robot = ET.parse(packages['dual_arm_support'] / 'urdf/dual_arm_robot_plug.urdf')
    for mesh in robot.findall('.//mesh'):
        name = mesh.get('filename')
        assert name.startswith('package://'), name
        package, relative = name[len('package://'):].split('/', 1)
        assert (packages[package] / relative).is_file(), name


def test_bundled_mujoco_loads_dual_arm_without_missing_assets():
    lib = ctypes.CDLL(str(ROOT / 'eiriarm_mujoco/third_party/mujoco/lib/libmujoco.so.3.3.0'))
    lib.mj_version.restype = ctypes.c_int
    assert lib.mj_version() == 330
    lib.mj_loadXML.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.mj_loadXML.restype = ctypes.c_void_p
    lib.mj_deleteModel.argtypes = [ctypes.c_void_p]
    error = ctypes.create_string_buffer(2048)
    path = ROOT / 'description/dual_arm_support/mjcf/dual_arm_robot.xml'
    model = lib.mj_loadXML(str(path).encode(), None, error, len(error))
    assert model, error.value.decode()
    lib.mj_deleteModel(model)


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
