"""Fill parameter-file templates with values from robot_spec.yaml.

A template is an ordinary ROS 2 parameter yaml in which a value can be written as
    $(spec drive.wheel_separation)          -> value from robot_spec.yaml
    $(spec drive.suggested_limits.linear neg) -> the same value negated
so robot_spec.yaml stays the single source of truth for every physical number.
"""
import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

_PATTERN = re.compile(r'\$\(spec\s+([A-Za-z0-9_.]+)(\s+neg)?\)')


def spec_path() -> str:
    return os.path.join(get_package_share_directory('jgb_rover_description'), 'config', 'robot_spec.yaml')


def load_spec() -> dict:
    with open(spec_path()) as f:
        return yaml.safe_load(f)


def lookup(spec: dict, dotted: str):
    value = spec
    for key in dotted.split('.'):
        if not isinstance(value, dict) or key not in value:
            raise KeyError(f'robot_spec.yaml has no entry "{dotted}"')
        value = value[key]
    return value


def render(template_path: str, out_name: str = None) -> str:
    """Render template_path and return the path of the filled-in yaml (in a temp dir)."""
    spec = load_spec()
    with open(template_path) as f:
        text = f.read()

    def substitute(m):
        value = lookup(spec, m.group(1))
        if m.group(2):
            value = -value
        return repr(float(value)) if isinstance(value, (int, float)) else str(value)

    rendered = _PATTERN.sub(substitute, text)
    yaml.safe_load(rendered)                      # fail early on a broken result
    out_dir = os.path.join(tempfile.gettempdir(), 'jgb_rover_params')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, out_name or os.path.basename(template_path))
    with open(out, 'w') as f:
        f.write(rendered)
    return out


def wall_clock(param_path: str, out_name: str = None) -> str:
    """Copy of a parameter file with every use_sim_time set to false, for the real robot.

    Needed because launch_ros passes a {'use_sim_time': False} dict under the '/**' wildcard, and
    node-specific entries in a file (e.g. ekf_filter_node: use_sim_time: true) win over wildcards.
    """
    with open(param_path) as f:
        data = yaml.safe_load(f)

    def clear(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key == 'use_sim_time':
                    node[key] = False
                else:
                    clear(value)
    clear(data)
    out_dir = os.path.join(tempfile.gettempdir(), 'jgb_rover_params')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, out_name or 'wall_' + os.path.basename(param_path))
    with open(out, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    return out
