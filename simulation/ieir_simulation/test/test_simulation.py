"""Optional simulator checks, no ROS nodes or physical hardware."""
import ast
import ctypes
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]


def test_dependencies_and_launches():
    deps = {d.text for d in ET.parse(PACKAGE / 'package.xml').getroot() if d.tag.endswith('depend')}
    assert 'libglfw3-dev' in deps and 'ieir_controllers' in deps
    assert not {'glfw3', 'mujoco', 'ieir_bringup'} & deps
    for filename in (PACKAGE / 'launch').glob('*.py'):
        text = filename.read_text()
        ast.parse(text)
        assert 'gripper_controller_node' not in text
        assert 'w3_robot_bridge' not in text


def test_mjcf_loads_with_bundled_sdk():
    lib = ctypes.CDLL(str(PACKAGE / 'third_party/mujoco/lib/libmujoco.so.3.3.0'))
    lib.mj_version.restype = ctypes.c_int
    assert lib.mj_version() == 330
    lib.mj_loadXML.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.mj_loadXML.restype = ctypes.c_void_p
    lib.mj_deleteModel.argtypes = [ctypes.c_void_p]
    error = ctypes.create_string_buffer(2048)
    for path in (PACKAGE / 'mjcf').glob('*.xml'):
        model = lib.mj_loadXML(str(path).encode(), None, error, len(error))
        assert model, f'{path.name}: {error.value.decode()}'
        lib.mj_deleteModel(model)


def test_simulation_owns_topic_hardware():
    plugin = ET.parse(PACKAGE / 'ieir_hardware_interface.xml').getroot().find('class')
    assert plugin.get('name') == 'ieir_simulation/TopicBasedHardwareInterface'
    assert plugin.get('type') == 'ieir_simulation::TopicBasedHardwareInterface'
