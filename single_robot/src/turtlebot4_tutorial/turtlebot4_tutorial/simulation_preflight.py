"""
Linux simulation preflight, shared verbatim by the two independent workspaces.

Only this user's TurtleBot tutorial/fleet processes are eligible. Never use a
name-wide pkill: ROS executables and Gazebo can also belong to other projects.
New children inherit a session tag so even reparented children remain visible.
"""

from dataclasses import dataclass
import fcntl
import json
import os
from pathlib import Path
import signal
import time
import uuid


SESSION = 'TB4_SIM_SESSION'
PROJECT = 'TB4_SIM_PROJECT'
OWNER = 'TB4_SIM_OWNER'
START = 'TB4_SIM_OWNER_START'
PROJECTS = ('single', 'fleet')
ROOTS = (Path.home() / 'turtlebot4_ws', Path.home() / 'turtlebot_fleet_ws')


@dataclass
class Process:
    pid: int
    ppid: int
    start: str
    argv: tuple
    env: dict
    cwd: str
    state: str


def read_process(pid):
    """Use kernel start ticks as well as PID, to guard against PID reuse."""
    folder = Path('/proc') / str(pid)
    try:
        if folder.stat().st_uid != os.getuid():
            return None
        stat = (folder / 'stat').read_text().rsplit(') ', 1)[1].split()
        argv = tuple(x.decode(errors='replace') for x in
                     (folder / 'cmdline').read_bytes().split(b'\0') if x)
        try:
            env = dict(x.decode(errors='replace').split('=', 1) for x in
                       (folder / 'environ').read_bytes().split(b'\0') if b'=' in x)
        except (OSError, ValueError):
            env = {}
        try:
            cwd = str((folder / 'cwd').resolve(strict=True))
        except OSError:
            cwd = ''
        return Process(pid, int(stat[1]), stat[19], argv, env, cwd, stat[0])
    except (OSError, ValueError, IndexError):
        return None


def snapshot():
    result = {}
    for entry in Path('/proc').iterdir():
        if entry.name.isdecimal():
            proc = read_process(int(entry.name))
            if proc and proc.state not in ('Z', 'X'):
                result[proc.pid] = proc
    return result


def ancestors(processes, pid):
    result = {pid}
    while pid in processes and processes[pid].ppid not in result:
        pid = processes[pid].ppid
        result.add(pid)
    return result


def _under(value, root):
    return value == str(root) or value.startswith(str(root) + '/')


def legacy_reason(proc):
    """Recognize pre-upgrade leftovers without matching shell command text."""
    args = proc.argv
    if not args:
        return None
    # ros2 launch is a Python entry point, so its path may be argv[1].
    for i, arg in enumerate(args[:2]):
        if Path(arg).name == 'ros2' and args[i + 1:i + 2] == ('launch',):
            if args[i + 2:i + 3] == ('fleet_bringup',):
                return '旧多机 launch'
            if (args[i + 2:i + 3] == ('turtlebot4_tutorial',)
                    and len(args) > i + 3 and args[i + 3].startswith('tutorial_')):
                return '旧单机 launch'
    if args[1:3] == ('-m', 'turtlebot4_tutorial.process_group'):
        return '旧单机 Gazebo 进程组监督器'
    for arg in args[:2]:
        # run.sh execs ``python3 scripts/supervise.py`` from the workspace.
        # Resolve that relative argv path using the target's cwd, not ours.
        located = os.path.normpath(os.path.join(proc.cwd, arg)) if proc.cwd else arg
        if any(_under(located, root / 'install') for root in ROOTS):
            return '项目安装目录中的节点'
        if located in (str(ROOTS[1] / 'scripts' / name)
                       for name in ('supervise.py', 'resources.py')):
            return '旧多机监督/采样进程'
        if located == str(ROOTS[1] / 'build/tools/truth_probe'):
            return '旧多机验收采样进程'
    executable = Path(args[0]).name
    gazebo = (args[0].startswith(('ign gazebo', 'gz sim')) or
              any(Path(a).name in ('ign', 'gz') and
                  any(v in args[i + 1:i + 3] for v in ('gazebo', 'sim'))
                  for i, a in enumerate(args[:3])))
    ros_node = (any(a.startswith('/opt/ros/') and '/lib/' in a for a in args[:2])
                or any(Path(a).name == 'ros2' and args[i + 1:i + 2] in
                       (('launch',), ('run',)) for i, a in enumerate(args[:2])))
    if not (gazebo or ros_node or executable in ('gzserver', 'gzclient', 'rviz2')):
        return None
    overlay = ':'.join(proc.env.get(k, '') for k in
                       ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'PYTHONPATH'))
    owned = (any(_under(proc.cwd, r) for r in ROOTS)
             or any(_under(part, r) for part in overlay.split(':') for r in ROOTS)
             or any(_under(a, r) for a in args for r in ROOTS)
             or proc.env.get('IGN_PARTITION') in ('tb4_course_fleet', 'tb4_single_demo'))
    return '带有本项目环境的 Gazebo/ROS 节点' if owned else None


def find_targets(processes, protected, current_session=None, legacy=True):
    targets = {}
    for pid, proc in processes.items():
        if pid in protected or (current_session and proc.env.get(SESSION) == current_session):
            continue
        reason = None
        if proc.env.get(PROJECT) in PROJECTS and proc.env.get(SESSION):
            reason = '旧仿真会话 ' + proc.env[SESSION]
        elif legacy:
            reason = legacy_reason(proc)
        if reason:
            targets[pid] = (proc, reason)
    # Capture children before their parent exits and they are reparented.
    changed = True
    while changed:
        changed = False
        for pid, proc in processes.items():
            if pid not in targets and pid not in protected and proc.ppid in targets:
                if current_session and proc.env.get(SESSION) == current_session:
                    continue
                targets[pid] = (proc, '旧仿真的子进程')
                changed = True
    return targets


def still_alive(proc):
    now = read_process(proc.pid)
    return now is not None and now.start == proc.start and now.state not in ('Z', 'X')


def _send(proc, sig):
    # Pin the kernel process identity before checking start ticks. A recycled
    # numeric PID cannot receive our signal even between this check and send.
    try:
        descriptor = os.pidfd_open(proc.pid)
    except ProcessLookupError:
        return
    try:
        if still_alive(proc):
            try:
                signal.pidfd_send_signal(descriptor, sig)
            except ProcessLookupError:
                pass
    finally:
        os.close(descriptor)


def clean_previous(report_path, *, legacy=True, timeouts=(8.0, 3.0, 2.0),
                   current_session=None):
    """Caller holds the startup lock. Fail closed if a process survives."""
    processes = snapshot()
    protected = ancestors(processes, os.getpid())
    targets = find_targets(processes, protected, current_session, legacy)
    report = {'started_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
              'policy': 'same-user tutorial and fleet simulations, across domains/partitions',
              'processes': [], 'signals': [], 'remaining': [], 'success': False}
    print(f'仿真前检查：发现 {len(targets)} 个旧仿真相关进程。', flush=True)
    error = None
    try:
        for sig, duration in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), timeouts):
            live = {pid: pair for pid, pair in targets.items() if still_alive(pair[0])}
            if not live:
                break
            # Give launch/supervisor a chance to stop its own children first.
            recipients = ({pid: pair for pid, pair in live.items() if pair[0].ppid not in live}
                          if sig == signal.SIGINT else live)
            for pid, (proc, _) in recipients.items():
                _send(proc, signal.SIGCONT)  # fault-injected SIGSTOP must not defeat cleanup
                _send(proc, sig)
                report['signals'].append({'pid': pid, 'signal': sig.name})
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                fresh = snapshot()
                # Also catch a final child forked while its old parent exits.
                known = find_targets(fresh, protected, current_session, legacy)
                for pid, pair in fresh.items():
                    if pid not in protected and pair.ppid in targets:
                        known.setdefault(pid, (pair, '退出期间产生的子进程'))
                targets.update(known)
                if not any(still_alive(p[0]) for p in targets.values()):
                    break
                time.sleep(0.1)
        # A final scan is mandatory even if no old process was found initially.
        targets.update(find_targets(snapshot(), protected, current_session, legacy))
        report['remaining'] = [p.pid for p, _ in targets.values() if still_alive(p)]
        report['success'] = not report['remaining']
    except OSError as exc:
        error = str(exc)
        report['error'] = error
    finally:
        report['processes'] = [dict(pid=p.pid, start=p.start, command=list(p.argv), reason=r)
                               for p, r in targets.values()]
        path = Path(report_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    if not report['success']:
        raise RuntimeError(f'旧仿真清理失败，拒绝启动新仿真：{error or report["remaining"]}；记录：{report_path}')
    print(f'仿真前检查通过：旧进程已清空。记录：{report_path}', flush=True)
    return report


def _inherited_session(env):
    if not env.get(SESSION) or env.get(PROJECT) not in PROJECTS:
        return False
    try:
        owner = int(env.get(OWNER, ''))
    except ValueError:
        return False
    processes = snapshot()
    return (owner in ancestors(processes, os.getpid()) and owner in processes
            and processes[owner].start == env.get(START))


def begin_session(project, report_path=None, env=None):
    """Run once per launch tree; nested launch shares the supervisor's session."""
    env = os.environ if env is None else env
    if _inherited_session(env):
        return {k: env[k] for k in (SESSION, PROJECT, OWNER, START)}
    cache = Path.home() / '.cache/turtlebot_simulation'
    cache.mkdir(parents=True, exist_ok=True)
    session = uuid.uuid4().hex
    report_path = report_path or cache / f'preflight-{session}.json'
    # Both independent projects use the same per-user startup mutex. Existing
    # running sessions do not hold it; a new invocation intentionally replaces them.
    with (cache / 'startup.lock').open('a') as lock:
        deadline = time.monotonic() + 30
        while True:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise RuntimeError('另一启动检查尚未结束，拒绝并发启动仿真')
                time.sleep(0.1)
        clean_previous(report_path)
        proc = read_process(os.getpid())
        markers = {SESSION: session, PROJECT: project, OWNER: str(proc.pid), START: proc.start}
        env.update(markers)
    return markers


def launch_preflight(context, project='single', report_path=None):
    """Check before launch and tag the entire tree, including timed nodes."""
    begin_session(project, report_path, context.environment)
    return []
