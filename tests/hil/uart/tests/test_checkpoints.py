"""Checkpoint replacement and terminal rendering without connected hardware."""
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'tester'))
spec = importlib.util.spec_from_file_location('uart_bench', root / 'tester/uart_bench.py')
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)
from suite_report import Report


class CheckpointTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.profile = {'baud': 921600, 'phases': [
            {'name': name, 'kind': 'command', 'seconds': 1, 'rate': 2}
            for name in ('50hz', '500hz', '1000hz')]}
        self.metadata = {'profile': self.profile, 'setup': {'schema_sha256': 'abc'},
                         'baud': 921600, 'port': 1, 'seed': 1979}

    def result(self, name, count):
        return {'name': name, 'passed': True, 'errors': [], 'pi': {'verified': count}}

    def test_resume_replaces_only_started_phases_and_saves_live_snapshot(self):
        output = self.root / 'suite'
        report = Report(output, self.metadata)
        for name in ('50hz', '500hz', '1000hz'):
            report.put(self.result(name, 10))
        report.close()
        report = Report(output, self.metadata)
        report.phase_begin(self.profile['phases'][1])
        report.event('pi', {'event': 'stats', 'verified': 3})
        saved = json.loads((output / 'report.json').read_text())
        phases = {p['name']: p for p in saved['phases']}
        self.assertEqual(phases['50hz']['pi']['verified'], 10)
        self.assertEqual(phases['500hz']['pi']['verified'], 3)
        self.assertEqual(phases['1000hz']['pi']['verified'], 10)
        report.put(self.result('500hz', 20))
        report.phase_begin(self.profile['phases'][2])
        report.put(self.result('1000hz', 30))
        report.close()
        saved = json.loads((output / 'report.json').read_text())
        phases = {p['name']: p for p in saved['phases']}
        self.assertEqual([phases[n]['pi']['verified'] for n in ('50hz', '500hz', '1000hz')], [10, 20, 30])
        events = [json.loads(line) for line in (output / 'events.jsonl').read_text().splitlines()]
        self.assertEqual((events[-1]['attempt'], events[-1]['phase']), (2, '500hz'))

    def test_mismatched_profile_and_concurrent_use_are_rejected(self):
        output = self.root / 'suite'
        first = Report(output, self.metadata)
        with self.assertRaisesRegex(RuntimeError, 'already running'):
            Report(output, self.metadata)
        first.close()
        with self.assertRaisesRegex(ValueError, 'profile differs'):
            Report(output, {**self.metadata, 'profile': {'phases': []}})

    def test_live_snapshot_survives_reopening_an_interrupted_phase(self):
        output = self.root / 'suite'
        report = Report(output, self.metadata)
        report.phase_begin(self.profile['phases'][1])
        report.event('pi', {'event': 'stats', 'verified': 23, 'missing': 1})
        report.close()
        report = Report(output, self.metadata)
        phase = report.data['phases'][0]
        self.assertEqual(phase['pi']['verified'], 23)
        self.assertEqual(phase['status'], 'aborted')
        report.phase_begin(self.profile['phases'][1])
        self.assertNotIn('pi', report.data['phases'][0])
        report.close()

    def test_start_at_uses_existing_suite_and_skips_prior_phases(self):
        output = self.root / 'suite'
        report = Report(output, self.metadata)
        report.put(self.result('50hz', 10))
        report.close()
        state = {'schema_sha256': 'abc', 'pi': 'user@host', 'executable': '/peer', 'pi_uart': '/dev/ttyAMA0'}
        (self.root / 'state.json').write_text(json.dumps(state))
        (self.root / 'profile.json').write_text(json.dumps(self.profile))
        args = SimpleNamespace(profile=self.root / 'profile.json', baud=None, output=output, suite_id='1',
                               start_at='500hz', port=1, pi=None, pi_uart=None, teensy_device=None,
                               password_env='NOT_SET')
        called = []
        class SSH:
            def __init__(self, *args): pass
            def connect(self): pass
            def execute(self, *args): return json.dumps({'version': 1, 'schema_sha256': 'abc'})
            def close(self): pass
        class Teensy:
            def __init__(self, *args): pass
            def command(self, *args, **kwargs): return {'version': 1, 'schema_sha256': 'abc'}
            def close(self): pass
        def dispatch(ssh, teensy, state, phase, args, report, baud, seed):
            called.append(phase['name'])
            return self.result(phase['name'], 20)
        with patch.object(bench, 'STATE_FILE', self.root / 'state.json'), patch.object(bench, 'SSH', SSH), \
             patch.object(bench, 'Teensy', Teensy), patch.object(bench, 'dispatch', dispatch):
            self.assertEqual(bench.run(args), 0)
        self.assertEqual(called, ['production-payload-check', '500hz', '1000hz', 'post-overload-recovery'])
        phases = {p['name']: p for p in json.loads((output / 'report.json').read_text())['phases']}
        self.assertEqual(phases['50hz']['pi']['verified'], 10)
        self.assertEqual(phases['500hz']['pi']['verified'], 20)


if __name__ == '__main__':
    unittest.main()
