"""Bounded acceptance stages, cancellation and evidence for one owned run."""
import json
from pathlib import Path
import signal
import time
from fleet_bringup.simulation_preflight import read_process


class AcceptanceInterrupted(RuntimeError):
    pass


class BackendExited(RuntimeError):
    pass


class AcceptanceProgress:
    def __init__(self, run, name, api, backend_status=None):
        self.run, self.name, self.api = Path(run).resolve(), name, api
        self.run.mkdir(parents=True, exist_ok=True)
        self.report = dict(passed=False, outcome='running', stages=[], started_steady=time.monotonic())
        self.stop_reason, self.handlers, self.last_sample = None, {}, 0.
        self.backend_status = backend_status or self.owned_backend_status()
        self.api.stopping = lambda: self.stop_reason is not None

    def owned_backend_status(self):
        pid = int((self.run/'supervisor.pid').read_text())
        process = read_process(pid)
        if (process is None or str(self.run) not in process.argv
                or process.cwd != str(Path(__file__).resolve().parents[1])):
            raise BackendExited('不能确认本轮 supervisor 的进程身份')

        def status():
            current = read_process(pid)
            if current is None or current.start != process.start or current.state in ('Z', 'X'):
                return '本轮 supervisor 已退出'
            return None
        return status

    def __enter__(self):
        for number in (signal.SIGINT, signal.SIGTERM):
            self.handlers[number] = signal.getsignal(number)
            signal.signal(number, self.interrupt)
        self.save()
        return self

    def interrupt(self, number, frame):
        self.stop_reason = signal.Signals(number).name

    def check(self):
        if self.stop_reason:
            raise AcceptanceInterrupted(self.stop_reason)
        if (self.run/'user_requested_stop.json').exists():
            raise AcceptanceInterrupted('本轮已记录用户停止请求')
        problem = self.backend_status()
        if problem:
            raise BackendExited(problem)

    def save(self):
        target = self.run/(self.name+'.json')
        temporary = target.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.report, ensure_ascii=False, indent=2)+'\n')
        temporary.replace(target)

    def stage(self, name, timeout, **expected):
        self.check()
        now = time.monotonic()
        if self.report['stages']:
            self.report['stages'][-1].update(outcome='passed', ended_steady=now)
        self.report['stage'] = name
        self.report['stages'].append(dict(name=name, started_steady=now, timeout=timeout,
                                         expected=expected, outcome='running'))
        self.save()
        print(name, flush=True)

    def state(self):
        self.check()
        state = self.api.state(.1)
        self.check()
        now = time.monotonic()
        sample = dict(steady_time=now, sim_time=state.get('sim_time'), mode=state.get('mode'),
            team_ready=state.get('team_ready'), coordinate_frame=state.get('coordinate_frame'),
            robots={n: {key: robot.get(key) for key in ('ready', 'aligned', 'boot_id', 'map_epoch',
                'mission_id', 'alignment', 'mapping_diagnostics', 'safety', 'regional_travel',
                'nav_ready', 'state_age', 'frame_epoch', 'connected_members', 'pose', 'local_pose',
                'pose_sim_time', 'pose_steady_time', 'goal')}
                for n, robot in state.get('robots', {}).items()})
        self.report['last_state'] = sample
        if now-self.last_sample >= 1.:
            with (self.run/(self.name+'-events.jsonl')).open('a') as stream:
                stream.write(json.dumps(dict(stage=self.report.get('stage'), **sample), ensure_ascii=False)+'\n')
            self.last_sample = now
            self.save()
        return state

    def wait(self, predicate, timeout, startup=False):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            try:
                state = self.state()
            except (OSError, ValueError) as error:
                if not startup:
                    raise
                self.report['last_connection_error'] = str(error)
                self.check()
                time.sleep(.1)
                continue
            if predicate(state):
                return state
        self.check()
        raise TimeoutError(self.report.get('stage', 'acceptance')+' 条件超时')

    def hold(self, seconds):
        started = time.monotonic()
        while time.monotonic()-started < seconds:
            self.state()
        return time.monotonic()-started

    def request(self, command, timeout=20):
        self.check()
        result = self.api.request('command', dict(command=command), timeout)
        self.check()
        if not result.get('success'):
            raise RuntimeError(str(result))
        return result

    def succeed(self, **values):
        self.check()
        self.report.update(passed=True, outcome='passed', **values)

    def __exit__(self, kind, error, traceback):
        if error:
            outcome = ('interrupted' if isinstance(error, (AcceptanceInterrupted, KeyboardInterrupt)) else
                       'backend_exited' if isinstance(error, BackendExited) else 'failed')
            self.report.update(passed=False, outcome=outcome, error=str(error), error_type=type(error).__name__)
        elif not self.report['passed']:
            self.report.update(outcome='failed', error='验收未执行到完成标记')
        now = time.monotonic()
        self.report['ended_steady'] = now
        if self.report['stages']:
            self.report['stages'][-1].update(outcome=self.report['outcome'], ended_steady=now)
        try:
            self.save()
        finally:
            for number, previous in self.handlers.items():
                signal.signal(number, previous)
        return False
