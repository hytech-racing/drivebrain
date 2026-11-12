"""Durable, resumable UART suite reports."""
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import tempfile


def now():
    return datetime.now(timezone.utc).isoformat()


def atomic_write(path, text):
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent, delete=False) as f:
        name = f.name
        try:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        except BaseException:
            os.unlink(name)
            raise
    try:
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


class Report:
    def __init__(self, output, metadata):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.lock = (self.output / '.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.lock.close()
            raise RuntimeError('This suite is already running')
        try:
            path = self.output / 'report.json'
            if path.exists():
                self.data = json.loads(path.read_text())
                old = self.data['metadata']
                for key in ('profile', 'port', 'baud', 'seed'):
                    if old.get(key) != metadata.get(key):
                        raise ValueError(f'Existing suite {key} differs; use a new suite ID')
                if old.get('setup', {}).get('schema_sha256') != metadata.get('setup', {}).get('schema_sha256'):
                    raise ValueError('Existing suite schema differs; use a new suite ID')
                for phase in self.data['phases']:
                    if phase.get('status') == 'running':
                        phase.update(status='aborted', aborted=True, passed=False)
                        phase.setdefault('errors', []).append('Previous attempt ended before this phase completed')
            else:
                self.data = {'started_at': now(), 'metadata': metadata, 'phases': [], 'errors': []}
            self.data.update(complete=False, passed=False, errors=[])
            self.data.setdefault('attempts', []).append({'started_at': now()})
            self.attempt = len(self.data['attempts'])
            self.current = None
            self.events = (self.output / 'events.jsonl').open('a', encoding='utf-8')
            self.save()
        except BaseException:
            self.lock.close()
            raise

    def phase_begin(self, phase):
        self.current = phase['name']
        # Replaced only as each phase actually starts; untouched later results survive.
        self.put({'name': self.current, 'config': phase, 'passed': False, 'status': 'running',
                  'attempt': self.attempt, 'started_at': now(), 'errors': []})

    def put(self, phase):
        phase.setdefault('attempt', self.attempt)
        phases = self.data['phases']
        for i, old in enumerate(phases):
            if old['name'] == phase['name']:
                phases[i] = phase
                break
        else:
            phases.append(phase)
        self.save()

    def event(self, source, value):
        self.events.write(json.dumps({'time': now(), 'attempt': self.attempt, 'phase': self.current,
                                      'source': source, 'data': value}) + '\n')
        self.events.flush()
        if self.current and source in ('pi', 'teensy') and isinstance(value, dict):
            phase = next((p for p in self.data['phases'] if p['name'] == self.current), None)
            if phase is not None and phase.get('status') == 'running':
                phase[source] = value
                phase['updated_at'] = now()
                self.save()

    def save(self):
        required = [p['name'] for p in self.data.get('metadata', {}).get('profile', {}).get('phases', [])]
        required += ['production-payload-check', 'post-overload-recovery'] if required else []
        phases = {p['name']: p for p in self.data['phases']}
        self.data['passed'] = (self.data.get('complete', False) and not self.data['errors'] and
                               all(name in phases and phases[name].get('passed') for name in required) and
                               all(p.get('passed', False) for p in phases.values()))
        atomic_write(self.output / 'report.json', json.dumps(self.data, indent=2) + '\n')
        lines = ['# UART stress-test report', '', f"Passed: **{self.data['passed']}**", '',
                 '| Phase | Result | Verified | Missing | RTT p99 ms |', '|---|---|---:|---:|---:|']
        for phase in self.data['phases']:
            pi = phase.get('pi', {})
            status = phase.get('status', 'PASS' if phase.get('passed') else 'FAIL')
            lines.append(f"| {phase['name']} | {status} | {pi.get('verified', 0)} | {pi.get('missing', 0)} | {pi.get('rtt_p99_ms', 0):.3f} |")
        lines += ['', '## Errors', ''] + ['- ' + str(e) for e in self.data['errors']]
        for phase in self.data['phases']:
            lines += [f"- {phase['name']}: {e}" for e in phase.get('errors', [])]
        atomic_write(self.output / 'report.md', '\n'.join(lines) + '\n')

    def close(self):
        self.events.close()
        self.lock.close()
