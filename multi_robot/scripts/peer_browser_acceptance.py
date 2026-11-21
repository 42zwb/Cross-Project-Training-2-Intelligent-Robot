#!/usr/bin/env python3
"""Exercise the peer dashboard with a real browser on a completed test run."""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright, expect
from runtime_context import RuntimeClient
from peer_acceptance_readiness import stationary_patrol_state


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    args = parser.parse_args()
    run = Path(args.run_dir).resolve()
    api = RuntimeClient(run)
    report = dict(passed=False, checks={}, errors=[], requests=[])
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(viewport=dict(width=1560, height=1100))
        page.on('pageerror', lambda error: report['errors'].append(str(error)))
        def click(selector, success=True):
            with page.expect_response(lambda r: r.request.method == 'POST' and '/api/' in r.url, timeout=20000) as response:
                page.locator(selector).click(timeout=10000)
            value = response.value.json()
            report['requests'].append(dict(url=response.value.url,
                body=json.loads(response.value.request.post_data or '{}'), response=value))
            assert value['success'] is success, value
            return value
        try:
            page.goto(api.base_url, wait_until='domcontentloaded')
            expect(page.locator('#health > tr')).to_have_count(3)
            expect(page.locator('#readiness')).to_contain_text('协同就绪')
            assert '坐标系' in page.locator('#readiness').inner_text()
            assert any(word in page.locator('#scope').inner_text() for word in ('分区', '跨区'))
            assert '未知起点' in page.locator('#scene').inner_text()
            report['checks']['relative_coordinates_and_three_robots'] = True
            click('[data-command="pause"]')
            expect(page.locator('#mode')).to_have_text('待命')
            page.locator('#spacing').fill('0.5')
            page.locator('#set-formation').click()
            expect(page.locator('#response')).to_contain_text('输入无效')
            page.locator('#spacing').fill('1.2')
            report['checks']['invalid_numeric_input_rejected'] = True
            if not page.locator('#apply-regions').is_visible():
                page.locator('summary').click()
            for row in page.locator('#regions > div').all():
                for element, value in zip(row.locator('input').all(), ['-1', '-1', '1', '1']):
                    element.fill(value)
            click('#apply-regions', success=False)
            report['checks']['invalid_partition_rejected_by_peers'] = True
            click('[data-command="explore"]')
            click('[data-command="pause"]')
            expect(page.locator('#mode')).to_have_text('待命')
            report['checks']['explore_and_pause_buttons'] = True
            # A stationary current-pose route tests the UI and explicit relative
            # frame transport; real travelling patrol is tested by peer_demo.
            state = stationary_patrol_state(api)
            report['patrol_input'] = dict(coordinate_frame=state['coordinate_frame'],
                robots={n: {k: r.get(k) for k in ('pose', 'boot_id', 'map_epoch', 'frame_epoch',
                    'ready', 'aligned', 'state_age')} for n, r in state['robots'].items()})
            expect(page.locator('#readiness')).to_contain_text('协同就绪')
            expect(page.locator('#readiness')).to_contain_text(state['coordinate_frame'])
            routes = {n: [r['pose']] for n, r in state['robots'].items()}
            page.locator('#routes').fill(json.dumps(routes))
            page.locator('#cycles').fill('1')
            page.locator('#dwell').fill('0')
            click('#start-patrol')
            submitted = json.loads(report['requests'][-1]['body']['argument'])
            assert submitted['routes'] == routes and submitted['coordinate_frame'] == state['coordinate_frame']
            expect(page.locator('#mode')).to_have_text('巡逻完成', timeout=45000)
            report['checks']['patrol_relative_frame_forwarding'] = True
            click('[data-command="estop"]')
            expect(page.locator('#mode')).to_have_text('急停锁定')
            assert page.locator('[data-command="explore"]').is_disabled()
            click('[data-command="clear-estop"]')
            expect(page.locator('#mode')).to_have_text('待命')
            report['checks']['estop_latch_and_clear_buttons'] = True
            click('[data-command="save"]')
            report['checks']['save_button'] = True
            page.screenshot(path=str(run/'peer-dashboard-controls.png'), full_page=True)
            report.update(browser=browser.version, passed=not report['errors'])
            assert report['passed'], report['errors']
        except Exception as error:
            report['failure'] = type(error).__name__+': '+str(error)
            try:
                page.screenshot(path=str(run/'peer-dashboard-failure.png'), full_page=True)
            except Exception as screenshot_error:
                report['screenshot_error'] = str(screenshot_error)
            raise
        finally:
            try:
                api.request('command', dict(command='pause'), timeout=8)
            finally:
                (run/'peer-browser-acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
                browser.close()
                if api._peer:
                    api._peer.close()
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
