#!/usr/bin/env python3
"""Merge W3 arm calibration without separating a measured offset from its sign."""
import argparse
from pathlib import Path
import math
import yaml


def merge(left, right, previous):
    signs = {e['name']: e.get('axis_sign', 1.0)
             for e in previous.get('offsets', [])}
    entries = []
    for side, channel, data in [('left', 0, left), ('right', 1, right)]:
        arm = data['offsets']
        expected = {f'{side}_joint_{i}' for i in range(7)}
        if len(arm) != 7 or {e['name'] for e in arm} != expected:
            raise ValueError(f'{side}: expected exactly seven named arm joints')
        if int(data.get('channel', channel)) != channel:
            raise ValueError(f'{side}: expected W3 channel {channel}')
        for original in arm:
            entry = dict(original)
            if int(entry['slot']) != int(entry['name'].rsplit('_', 1)[1]):
                raise ValueError(f"Wrong slot for {entry['name']}")
            if int(entry.get('channel', channel)) != channel:
                raise ValueError(f"Wrong channel for {entry['name']}")
            entry['channel'] = channel
            measured_sign = float(entry.get('axis_sign', 1.0))
            entry['axis_sign'] = (measured_sign if entry.get('direction_verified') is True
                                  else float(signs.get(entry['name'], measured_sign)))
            if entry['axis_sign'] != measured_sign:
                if 'raw_at_reference' not in entry or 'urdf_pos_at_reference' not in entry:
                    raise ValueError(f"Cannot change sign without reference capture for {entry['name']}")
                entry['zero_offset'] = (float(entry['raw_at_reference'])
                                        - entry['axis_sign'] * float(entry['urdf_pos_at_reference']))
            if entry['axis_sign'] not in (-1.0, 1.0) or not math.isfinite(float(entry['zero_offset'])):
                raise ValueError(f"Invalid offset/sign for {entry['name']}")
            entries.append(entry)
    return {'transport': 'w3', 'offsets': entries}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    directory = Path(__file__).resolve().parent
    parser.add_argument('--left', type=Path, default=directory / 'joint_offsets_left.yaml')
    parser.add_argument('--right', type=Path, default=directory / 'joint_offsets_right.yaml')
    parser.add_argument('--output', type=Path, default=directory / 'joint_offsets_dual.yaml')
    args = parser.parse_args()
    previous = yaml.safe_load(args.output.read_text()) if args.output.exists() else {}
    result = merge(yaml.safe_load(args.left.read_text()),
                   yaml.safe_load(args.right.read_text()), previous or {})
    args.output.write_text(yaml.safe_dump(result, sort_keys=False))
    print(f'Merged 14 W3 joints -> {args.output}')


if __name__ == '__main__':
    main()
