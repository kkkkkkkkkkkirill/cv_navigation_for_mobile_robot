#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from urllib.request import Request, urlopen

urllib.request.install_opener(
    urllib.request.build_opener(urllib.request.ProxyHandler({}))
)

BACKEND = os.environ.get('AMR_BACKEND_URL', 'http://127.0.0.1:8010').rstrip('/')
POLL_INTERVAL_S = float(os.environ.get('AMR_DISPATCH_POLL_S', '1.0'))
AUTO_ARRIVE = os.environ.get('AMR_AUTO_SIMULATE_ARRIVAL', 'true').lower() not in {'0', 'false', 'no'}
TRAVEL_TIME_S = float(os.environ.get('AMR_SIMULATED_TRAVEL_TIME_S', '4.0'))
ROBOT_NAME = os.environ.get('AMR_ROBOT_NAME', 'amr-1')


def get_json(path: str) -> dict:
    try:
        with urlopen(BACKEND + path, timeout=2.0) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except urllib.error.HTTPError as exc:
        if exc.code == 503:
            return {'task_id': None, 'phase': 'idle', 'target_station': None, 'source': None, 'destination': None, 'should_return_to_depot': False, 'robot_name': ROBOT_NAME, 'mode': 'external_bridge'}
        raise


def post_json(path: str, payload: dict) -> dict:
    req = Request(BACKEND + path, data=json.dumps(payload).encode('utf-8'), headers={'Content-Type': 'application/json'})
    with urlopen(req, timeout=5.0) as resp:
        return json.loads(resp.read().decode('utf-8'))


def main() -> None:
    print(f'task_command_bridge started against {BACKEND}')
    print(f'auto_arrive={AUTO_ARRIVE} travel_time_s={TRAVEL_TIME_S}')
    last_key = None
    nav_started_for = None
    arrived_for = None
    dispatch_seen_at = 0.0

    while True:
        dispatch = get_json('/robot/dispatch')
        key = (dispatch.get('task_id'), dispatch.get('phase'), dispatch.get('target_station'))

        if key != last_key:
            print('dispatch:', dispatch)
            last_key = key
            nav_started_for = None
            arrived_for = None
            dispatch_seen_at = time.time()

        phase = dispatch.get('phase')
        task_id = dispatch.get('task_id')

        if phase in {'pickup', 'dropoff', 'return'}:
            if nav_started_for != key:
                post_json('/robot/event', {'robot_name': ROBOT_NAME, 'event': 'nav_started', 'task_id': task_id})
                nav_started_for = key
                dispatch_seen_at = time.time()
                print('event: nav_started', key)

            if AUTO_ARRIVE and arrived_for != key and (time.time() - dispatch_seen_at) >= TRAVEL_TIME_S:
                if phase == 'pickup':
                    evt = 'arrived_pickup'
                elif phase == 'dropoff':
                    evt = 'arrived_dropoff'
                else:
                    evt = 'returned_to_depot'
                post_json('/robot/event', {'robot_name': ROBOT_NAME, 'event': evt, 'task_id': task_id})
                arrived_for = key
                print('event:', evt, key)

        time.sleep(POLL_INTERVAL_S)


if __name__ == '__main__':
    main()
