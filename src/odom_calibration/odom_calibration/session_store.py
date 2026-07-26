"""Persistent wizard session model.

The browser is only a view/controller. The server owns the session and writes
it atomically after every accepted action, so closing a tab, changing Wi-Fi,
or reconnecting from another laptop does not lose completed measurements.
"""

from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
import uuid


SCHEMA_VERSION = 1
VALID_MODES = ('movement', 'movement_steering')
VALID_STAGES = (
    'setup',
    'preflight',
    'stationary',
    'movement',
    'steering',
    'report',
)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def new_session(mode, current_parameters, vehicle=None):
    if mode not in VALID_MODES:
        raise ValueError(f'mode must be one of {VALID_MODES}')
    timestamp = utc_now()
    return {
        'schema_version': SCHEMA_VERSION,
        'session_id': str(uuid.uuid4()),
        'created_at': timestamp,
        'updated_at': timestamp,
        'mode': mode,
        'stage': 'preflight',
        'vehicle': copy.deepcopy(vehicle or {
            'model': 'Traxxas Ford Fiesta ST Rally VXL 74276-4',
            'wheelbase_m': 0.324,
        }),
        'current_parameters': copy.deepcopy(current_parameters),
        'trials': [],
        'pending_capture': None,
        'active_capture': None,
        'event_log': [{
            'at': timestamp,
            'event': 'session_created',
            'detail': f'Created {mode} session.',
        }],
        'report': None,
    }


def touch(session, event=None, detail=None):
    session['updated_at'] = utc_now()
    if event:
        session.setdefault('event_log', []).append({
            'at': session['updated_at'],
            'event': event,
            'detail': detail or '',
        })
        # Keep a useful audit trail without growing forever.
        session['event_log'] = session['event_log'][-500:]


class SessionStore:
    def __init__(self, directory):
        self.directory = Path(os.path.expanduser(str(directory))).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.active_path = self.directory / 'active_session.json'

    def load(self):
        if not self.active_path.exists():
            return None
        try:
            with self.active_path.open(encoding='utf-8') as handle:
                session = json.load(handle)
        except (OSError, json.JSONDecodeError):
            return None
        if session.get('schema_version') != SCHEMA_VERSION:
            return None
        if session.get('active_capture'):
            session['active_capture'] = None
            touch(
                session,
                'capture_interrupted',
                'Backend restarted during a capture; no partial trial was accepted.',
            )
            self.save(session)
        return session

    def save(self, session):
        touch(session)
        descriptor, temporary_path = tempfile.mkstemp(
            dir=self.directory,
            prefix='.active_session.',
            suffix='.tmp',
        )
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
                json.dump(session, handle, indent=2, sort_keys=True)
                handle.write('\n')
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_path, self.active_path)
        finally:
            try:
                os.unlink(temporary_path)
            except FileNotFoundError:
                pass

    def archive_report(self, session, markdown):
        session_id = session.get('session_id', 'unknown')
        report = session.get('report', {})
        json_path = self.directory / f'calibration-{session_id}.json'
        markdown_path = self.directory / f'calibration-{session_id}.md'
        for path, content in (
                (json_path, json.dumps(report, indent=2, sort_keys=True) + '\n'),
                (markdown_path, markdown)):
            descriptor, temporary_path = tempfile.mkstemp(
                dir=self.directory,
                prefix=f'.{path.name}.',
                suffix='.tmp',
            )
            try:
                with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
                    handle.write(content)
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(temporary_path, path)
            finally:
                try:
                    os.unlink(temporary_path)
                except FileNotFoundError:
                    pass
        return json_path, markdown_path

    def archive_session(self, session):
        """Preserve an old active session before a deliberate replacement."""
        session_id = session.get('session_id', 'unknown')
        path = self.directory / f'session-{session_id}.json'
        descriptor, temporary_path = tempfile.mkstemp(
            dir=self.directory,
            prefix=f'.{path.name}.',
            suffix='.tmp',
        )
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
                json.dump(session, handle, indent=2, sort_keys=True)
                handle.write('\n')
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_path, path)
        finally:
            try:
                os.unlink(temporary_path)
            except FileNotFoundError:
                pass
        return path
