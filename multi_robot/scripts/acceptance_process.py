"""Run an owned acceptance subprocess with bounded cancellation and logs."""
import os
import json
from pathlib import Path
import signal
import subprocess
import time
import sys


def browser_environment(root):
    """Check the real isolated browser before starting any robot processes."""
    env = dict(os.environ)
    cache = Path(env.get('FLEET_BROWSER_PYTHON', Path.home()/'.cache/tb4-fleet-browser-python'))
    if cache.is_dir():
        env['PYTHONPATH'] = str(cache)+os.pathsep+env.get('PYTHONPATH', '')
    code = '''import json, importlib.metadata
from playwright.sync_api import sync_playwright
with sync_playwright() as p:
    browser = p.chromium.launch(headless=True)
    page = browser.new_page()
    page.set_content('<title>fleet-preflight</title>')
    assert page.title() == 'fleet-preflight'
    print(json.dumps(dict(playwright=importlib.metadata.version('playwright'), chromium=browser.version)))
    browser.close()
'''
    try:
        result = subprocess.run([sys.executable, '-c', code], cwd=root, env=env,
                                capture_output=True, text=True, timeout=30, check=True)
        report = json.loads(result.stdout)
        required = next(line.split('==', 1)[1].strip() for line in
                        (Path(root)/'requirements-browser.txt').read_text().splitlines()
                        if line.startswith('playwright=='))
        if report['playwright'] != required:
            raise ValueError('Playwright 版本不符合 requirements-browser.txt')
    except (subprocess.SubprocessError, ValueError, OSError) as error:
        detail = getattr(error, 'stderr', None) or str(error)
        raise RuntimeError('浏览器预检失败，尚未启动仿真。请按 requirements-browser.txt 准备依赖，'
                           '必要时用 FLEET_BROWSER_PYTHON 指定依赖目录。\n'+detail) from error
    return env, dict(report, passed=True, dependency_directory=str(cache), python=sys.executable)


def stop_child(child, grace=90):
    if child.poll() is None:
        try:
            os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError:
            child.wait(timeout=10)
            return
        try:
            child.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait(timeout=10)


def run_command(command, *, cwd, log, progress, timeout, env=None, shutdown_grace=90):
    with Path(log).open('x') as stream:
        child = subprocess.Popen(command, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT,
                                 start_new_session=True, env=env)
        deadline = time.monotonic()+timeout
        try:
            while child.poll() is None:
                progress.check()
                if time.monotonic() >= deadline:
                    raise TimeoutError('验收子进程超时：'+str(command))
                time.sleep(.2)
            progress.check()
            if child.returncode:
                raise subprocess.CalledProcessError(child.returncode, command)
        except BaseException as error:
            # Persist cancellation/failure before potentially slow DDS teardown.
            # The owning context still records its final outcome on exit.
            from acceptance_progress import AcceptanceInterrupted
            progress.report.update(passed=False,
                outcome='interrupted' if isinstance(error, (KeyboardInterrupt, AcceptanceInterrupted)) else 'failed',
                error=str(error), error_type=type(error).__name__)
            progress.save()
            raise
        finally:
            stop_child(child, grace=shutdown_grace)
