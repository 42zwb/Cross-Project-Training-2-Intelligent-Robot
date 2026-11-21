"""Stopping a mapper must cancel its active registration process promptly."""
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
import time
import numpy as np
import pytest
from fleet_mapping.registration import (Snapshot, register, registration_job,
                                        initialize_registration_worker)


def initialized(stop, ready):
    initialize_registration_worker(stop)
    ready.set()


@pytest.mark.parametrize('worker', [register, registration_job])
def test_spawned_registration_honors_cancellation_during_geometric_search(worker):
    data = np.zeros((600, 600), np.int16)
    data[:4] = data[-4:] = 100
    data[:, :4] = data[:, -4:] = 100
    rng = np.random.default_rng(52)
    for x, y in rng.integers(20, 550, size=(60, 2)):
        data[y:y+17, x:x+23] = 100
    source = Snapshot('robot1', 'a', 1, 1, data, .05, (0., 0., 0.))
    target = Snapshot('robot2', 'b', 1, 1, data, .05, (0., 0., 0.))
    ctx = get_context('spawn')
    stop, ready = ctx.Event(), ctx.Event()
    pool = ProcessPoolExecutor(1, mp_context=ctx, initializer=initialized, initargs=(stop, ready))
    try:
        future = pool.submit(worker, source, target)
        assert ready.wait(8)
        time.sleep(.15)
        started = time.monotonic()
        stop.set()
        result = future.result(timeout=3)
        if worker is registration_job:
            assert result['match'] is None and result['diagnostics']['reason'] == 'cancelled'
        else:
            assert result is None
        pool.shutdown(wait=True, cancel_futures=True)
        assert time.monotonic()-started < 3
    finally:
        stop.set()
        pool.shutdown(wait=True, cancel_futures=True)
