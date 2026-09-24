"""Synthetic calibration input for offline/fake-feedback tests only."""
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def write_test_inputs(directory):
    refs = yaml.safe_load(
        (ROOT / 'ros2_ws_config/joint_calibration_dual_right.yaml').read_text())
    directions = []
    for joint in refs['joints']:
        joint['reference_review'] = dict(
            confirmed=True, confirmed_angle_rad=joint['urdf_pos_at_limit'])
        directions.append(dict(name=joint['name'], slot=joint['slot'],
                               axis_sign=1 if joint['slot'] % 2 else -1,
                               direction_verified=True))
    reference_path = Path(directory) / 'synthetic_reviewed.yaml'
    direction_path = Path(directory) / 'synthetic_directions.yaml'
    reference_path.write_text(yaml.safe_dump(refs))
    direction_path.write_text(yaml.safe_dump(dict(mode='direction', channel=1, directions=directions)))
    return reference_path, direction_path
