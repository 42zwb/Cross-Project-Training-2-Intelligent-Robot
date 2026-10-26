#!/usr/bin/env python3
"""Click the real dashboard against a running simulation; requires Playwright.

Run only on a dedicated simulation: explores, patrols, pauses and tests estop.
This check never injects maps, poses or velocity commands.
"""

import argparse
import json
import time
import urllib.request
from pathlib import Path

from playwright.sync_api import sync_playwright
import yaml
from runtime_context import RuntimeClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url")
    parser.add_argument("--run-dir")
    parser.add_argument("--cycles", type=int, choices=range(1, 4), default=2)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    run = Path(args.run_dir or (root / "runs/latest.txt").read_text()).resolve()
    cfg = yaml.safe_load((run / "assets/fleet.yaml").read_text())
    api = RuntimeClient(run, base_url=args.url)
    base = api.base_url
    report = {"checks": {}, "cycles": args.cycles}

    def state():
        return api.state(timeout=4)

    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(viewport={"width": 1560, "height": 1050})
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))
        report["browser"] = browser.version

        def wait(check, timeout=180):
            deadline, last_print = time.monotonic() + timeout, 0
            while time.monotonic() < deadline:
                s = state()
                if check(s):
                    return s
                if s["mode"] == "FAULT":
                    raise RuntimeError(s["message"])
                if time.monotonic() - last_print > 15:
                    print(json.dumps({"mode": s["mode"], "patrol": s.get("patrol")}, ensure_ascii=False), flush=True)
                    last_print = time.monotonic()
                page.wait_for_timeout(300)
            raise TimeoutError("等待状态超时：" + state()["message"])

        def click(selector, success=True):
            with page.expect_response(lambda response: response.request.method == "POST" and "/api/" in response.url, timeout=20000) as response:
                page.locator(selector).click(timeout=10000)
            result = response.value.json()
            assert result["success"] is success, result
            return result

        try:
            page.goto(base, wait_until="domcontentloaded")
            wait(lambda s: s["ready"])
            page.wait_for_function("document.querySelector('#health').children.length === 3")
            report["planner"] = state()["planner"]
            if not state().get("regions_locked"):
                page.locator("summary").click()
                click("#apply-regions")
                report["checks"]["regions_button"] = True
            click('[data-command="explore"]')
            wait(lambda s: s["mode"] == "EXPLORING" and all(r["travel"] > 0.15 for r in s["robots"].values()))
            report["checks"]["explore_button_moves_three_robots"] = True
            click('[data-command="pause"]')
            wait(lambda s: s["mode"] == "IDLE")
            page.locator("#cycles").fill(str(args.cycles))
            page.locator("#dwell").fill("0.5")
            click("#start-patrol")
            s = wait(lambda s: s["mode"] == "PATROL_DONE", timeout=240)
            assert all(p["completed"] == p["total"] == 2 * args.cycles for p in s["patrol"]["robots"].values())
            report["checks"]["patrol_completion"] = True
            report["patrol"] = s["patrol"]
            report["distance"] = {n: r["distance"] for n, r in s["robots"].items()}
            page.wait_for_timeout(600)
            page.screenshot(path=str(run / "dashboard-patrol.png"), full_page=True)
            page.locator("#routes").fill('{"robot1":[[0,-4,0]]}')
            click("#start-patrol", success=False)
            assert state()["mode"] == "PATROL_DONE"
            report["checks"]["cross_region_rejected_without_interrupt"] = True
            page.locator("#x").fill("99")
            click("#set-formation", success=False)
            report["checks"]["invalid_formation_button"] = True
            page.locator("#x").fill("0")
            click('[data-command="save"]')
            report["checks"]["save_button"] = True
            example = yaml.safe_load((root / "src/fleet_control/config/patrol_example.yaml").read_text())
            page.locator("#routes").fill(json.dumps(example["routes"]))
            page.locator("#cycles").fill("20")
            click("#start-patrol")
            wait(lambda s: s["mode"] == "PATROLLING" and any(abs(r["safety"].get("v", 0)) > 0.01 for r in s["robots"].values()))
            click('[data-command="pause"]')
            s = wait(lambda s: s["mode"] == "IDLE" and s["patrol"]["status"] == "cancelled")
            page.wait_for_timeout(1000)
            before = {n: r["distance"] for n, r in state()["robots"].items()}
            page.wait_for_timeout(3000)
            after = state()
            assert all(after["robots"][n]["distance"] - v < 0.01 for n, v in before.items())
            assert all(not r["safety"].get("moving_allowed") for r in after["robots"].values())
            report["checks"]["pause_cancels_patrol_and_stays_stopped"] = True
            click('[data-command="estop"]')
            wait(lambda s: s["mode"] == "ESTOP")
            click('[data-command="clear-estop"]')
            wait(lambda s: s["mode"] == "IDLE" and not s["estop"])
            assert all(r["goal"] is None for r in state()["robots"].values())
            report["checks"]["estop_clear_does_not_resume"] = True
            assert not errors, errors
            report["checks"]["no_browser_errors"] = True
            report["passed"] = True
        finally:
            try:
                api.request("command", {"command":"pause"}, timeout=15)
            except OSError:
                pass
            report["browser_errors"] = errors
            (run / "browser_acceptance.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
            browser.close()
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
