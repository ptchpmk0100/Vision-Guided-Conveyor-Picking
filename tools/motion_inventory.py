#!/usr/bin/env python3
"""Read-only local dependency/capacity inventory; does not start ROS or a simulator."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import re
import subprocess
import yaml

ROOT = Path(__file__).resolve().parents[1]
ROS = Path('/opt/ros/jazzy')


def inventory():
    config = ROOT / 'src/twin_moveit_config/config/ompl_planning.yaml'
    data = yaml.safe_load(config.read_text())
    library = ROS / 'lib/libmoveit_ompl_interface.so'
    strings = subprocess.check_output(['strings', str(library)], text=True)
    advertised = sorted(set(re.findall(r'^geometric::([A-Za-z0-9_]+)$', strings, re.M)))
    packages = ['ros-jazzy-moveit-py', 'ros-jazzy-moveit-planners-ompl',
                'ros-jazzy-moveit-planners-chomp', 'ros-jazzy-moveit-planners-stomp',
                'ros-jazzy-pilz-industrial-motion-planner']
    versions = subprocess.run(['dpkg-query', '-W', '-f=${Package} ${Version}\n', *packages],
                              text=True, capture_output=True)
    files = [config, ROOT/'src/twin_description/urdf/panda.urdf',
             ROOT/'src/twin_moveit_config/config/panda.srdf',
             ROOT/'src/twin_moveit_config/config/joint_limits.yaml',
             ROOT/'src/twin_description/config/panda_controllers.yaml']
    return dict(host=platform.platform(), cpu=platform.processor(),
        memory_info={line.split(':')[0]:line.split(':')[1].strip()
                     for line in Path('/proc/meminfo').read_text().splitlines()
                     if line.split(':')[0] in ('MemTotal','MemAvailable','SwapTotal','SwapFree')},
        packages=versions.stdout.splitlines(), package_query_errors=versions.stderr,
        ompl_library_advertised=advertised,
        configured_planners={name: cfg['type'] for name,cfg in data['planner_configs'].items()},
        missing_requested_planners=[name for name in ('BITstar','InformedRRTstar') if name not in advertised],
        hashes={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        sequence_capability='NOT_CHECKED: installed plugin is not proof of a running synchronized sequence server',
        execution_authorized=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps(inventory(), indent=2)+'\n')
    print(args.output)
