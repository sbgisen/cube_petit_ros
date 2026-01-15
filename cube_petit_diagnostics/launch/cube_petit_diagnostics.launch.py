import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(pkg, relpath):
    pkg_share = FindPackageShare(pkg).find(pkg)
    path = os.path.join(pkg_share, relpath)
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def get_pkg_file_path(pkg, relpath):
    pkg_share = FindPackageShare(pkg).find(pkg)
    return os.path.join(pkg_share, relpath)


def generate_launch_description():
    pkg = 'cube_petit_diagnostics'
    nodes = []

    # -----------------------------
    # Topic monitors
    # -----------------------------
    topic_cfg = load_yaml(pkg, 'config/topic_monitor.yaml')
    for key in topic_cfg.get('topic_names', []):
        t = topic_cfg[key]
        nodes.append(
            Node(
                package=pkg,
                executable='topic_watchdog_node',
                name=f'{key}_topic_watchdog',
                parameters=[{
                    'topic': t['name'],
                    'msg_type': t['type'],
                    'timeout': t.get('timeout', 1.0),
                }],
                output='screen',
            )
        )

    # -----------------------------
    # Node monitors
    # -----------------------------
    node_cfg = load_yaml(pkg, 'config/node_monitor.yaml')
    for key in node_cfg.get('node_names', []):
        n = node_cfg[key]
        nodes.append(
            Node(
                package=pkg,
                executable='node_watchdog_node',
                name=f'{key}_node_watchdog',
                parameters=[{
                    'node_name': n['name'],
                    'timeout': n.get('timeout', 2.0),
                }],
                output='screen',
            )
        )

    analyzers_file = get_pkg_file_path(pkg, 'config/analyzers.yaml')
    nodes.append(
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            output='screen',
            parameters=[analyzers_file],
            remappings=[
                ('/diagnostics', '/diagnostics'),
                ('/diagnostics_agg', '/diagnostics_agg'),
            ],
        )
    )

    return LaunchDescription(nodes)
