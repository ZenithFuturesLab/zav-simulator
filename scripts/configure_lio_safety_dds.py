#!/usr/bin/env python3
"""Idempotently export PX4 v1.14 landing acknowledgement telemetry over uXRCE-DDS."""

import argparse
import re
from pathlib import Path

import yaml


PUBLICATIONS = {
    '/fmu/out/estimator_aid_src_baro_hgt': 'px4_msgs::msg::EstimatorAidSource1d',
    '/fmu/out/vehicle_command_ack': 'px4_msgs::msg::VehicleCommandAck',
    '/fmu/out/vehicle_land_detected': 'px4_msgs::msg::VehicleLandDetected',
}


def configure(px4):
    path = Path(px4) / 'src/modules/uxrce_dds_client/dds_topics.yaml'
    original = path.read_text()
    text = original
    # Remove the four exports installed by the former three-check policy.
    if '# ZAV LIO failsafe telemetry' in text:
        text = re.sub(r'  - topic: /fmu/out/estimator_aid_src_ev_(?:pos|vel|hgt|yaw)\n'
                      r'    type: px4_msgs::msg::EstimatorAidSource[123]d\n\n?', '', text)
    document = yaml.safe_load(text)
    existing = {item['topic']: item['type'] for item in (document.get('publications') or [])}
    additions = []
    for topic, typ in PUBLICATIONS.items():
        if topic in existing:
            if existing[topic] != typ:
                raise ValueError(f'Unexpected DDS type for {topic}: {existing[topic]}')
        else:
            additions.append(f'  - topic: {topic}\n    type: {typ}\n')
    if additions:
        marker = '\nsubscriptions:'
        if marker not in text:
            raise ValueError('Unsupported PX4 dds_topics.yaml')
        text = text.replace(marker, '\n  # ZAV LIO failsafe telemetry\n'
                            + '\n'.join(additions) + marker, 1)
    if text != original:
        path.write_text(text)
    return len(additions)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-dir', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'src/PX4-Autopilot')
    args = parser.parse_args()
    print(f'Added {configure(args.px4_dir)} PX4 DDS publications; rebuild PX4 to apply.')
