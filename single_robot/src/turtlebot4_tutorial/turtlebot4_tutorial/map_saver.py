"""Save maps with thresholds that preserve canonical unknown pixels."""

import argparse
from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_prefix

from .common import parse_cli_args
from .map_tools import validate_map


def main():
    parser = argparse.ArgumentParser(description='保存可正确重新加载的 PGM/YAML 地图')
    parser.add_argument('output', help='输出文件前缀，例如 ~/maps/my_map')
    parser.add_argument('--topic', default='map')
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parse_cli_args(parser)
    if not 0.0 < args.timeout < float('inf'):
        parser.error('timeout 必须是大于 0 的有限数值')
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    executable = Path(get_package_prefix('nav2_map_server')) / 'lib/nav2_map_server/map_saver_cli'
    ros_args = (sys.argv[sys.argv.index('--ros-args'):]
                if '--ros-args' in sys.argv else ['--ros-args'])
    command = [
        str(executable), '-t', args.topic, '-f', str(output),
        '--free', '0.196', '--occ', '0.65', '--fmt', 'pgm', '--mode', 'trinary',
        *ros_args, '-p', f'save_map_timeout:={args.timeout}',
    ]
    try:
        result = subprocess.run(command, check=False, timeout=args.timeout + 15.0)
        if result.returncode:
            return result.returncode
        validate_map(str(output) + '.yaml')
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        print(f'地图保存失败: {error}', file=sys.stderr)
        return 1
    return 0
