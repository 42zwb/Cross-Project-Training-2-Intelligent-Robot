"""Launch RViz with per-process rendering and writable user configuration."""

from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .common import normalize_namespace
from .map_tools import validate_map


def rendering_environment(context):
    """Use CPU rendering only for GUI processes, leaving sensors accelerated."""
    if LaunchConfiguration('software_rendering').perform(context) == 'true':
        return {'LIBGL_ALWAYS_SOFTWARE': '1', 'LP_NUM_THREADS': '2'}
    return {'LIBGL_ALWAYS_SOFTWARE': '0'}


def make_rviz(context, mode='mapping'):
    """Create a namespaced RViz process only when requested."""
    if LaunchConfiguration('rviz').perform(context) != 'true':
        return []
    namespace = normalize_namespace(LaunchConfiguration('namespace').perform(context))
    package = Path(get_package_share_directory('turtlebot4_tutorial'))
    folder = Path.home() / '.ros/turtlebot4_tutorial' / (namespace or 'default')
    folder.mkdir(parents=True, exist_ok=True)
    config = folder / f'{mode}-v2.rviz'
    if not config.exists():
        shutil.copy2(package / 'config' / f'{mode}.rviz', config)
    return [Node(
        package='rviz2', executable='rviz2', output='screen', namespace=namespace,
        arguments=['-d', str(config)],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        additional_env=rendering_environment(context),
    )]


def check_navigation_map(context):
    """Fail before localization starts if the saved map is unsafe or missing."""
    validate_map(LaunchConfiguration('map').perform(context))
    return []
