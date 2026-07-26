import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration.session_store import (  # noqa: E402
    SessionStore,
    new_session,
)


PARAMETERS = {
    'speed_to_erpm_gain': 4614.0,
    'speed_to_erpm_offset': 0.0,
    'steering_angle_to_servo_gain': -1.2135,
    'steering_angle_to_servo_offset': 0.5304,
    'wheelbase': 0.324,
}


def test_session_round_trip(tmp_path):
    store = SessionStore(tmp_path)
    session = new_session('movement', PARAMETERS)
    session['trials'].append({'id': 'one', 'accepted': True})
    store.save(session)
    loaded = store.load()
    assert loaded['session_id'] == session['session_id']
    assert loaded['trials'] == [{'id': 'one', 'accepted': True}]


def test_restart_marks_active_capture_interrupted(tmp_path):
    store = SessionStore(tmp_path)
    session = new_session('movement', PARAMETERS)
    session['active_capture'] = {'kind': 'movement'}
    store.save(session)
    loaded = store.load()
    assert loaded['active_capture'] is None
    assert loaded['event_log'][-1]['event'] == 'capture_interrupted'


def test_deliberate_replacement_archive_is_valid_json(tmp_path):
    store = SessionStore(tmp_path)
    session = new_session('movement_steering', PARAMETERS)
    path = store.archive_session(session)
    assert path.exists()
    with path.open(encoding='utf-8') as handle:
        archived = json.load(handle)
    assert archived['session_id'] == session['session_id']


def test_report_archive_writes_json_and_markdown(tmp_path):
    store = SessionStore(tmp_path)
    session = new_session('movement', PARAMETERS)
    session['report'] = {'overall_status': 'review'}
    json_path, markdown_path = store.archive_report(session, '# Report\n')
    assert json.loads(json_path.read_text())['overall_status'] == 'review'
    assert markdown_path.read_text() == '# Report\n'
