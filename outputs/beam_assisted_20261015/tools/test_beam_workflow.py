"""Small shared protocol regressions; no simulator or production files."""
from pathlib import Path
import copy
import json
import tempfile
import unittest
from beam_workflow import CHECKS, claim, save, sha, validate_delta


class WorkflowContract(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.p = Path(self.temp.name)
        self.batch = dict(maximum_native_attempts=10, native_attempts_started=0,
                          completed_cycles=0, next_native_run_authorized=True,
                          cycles=[dict(id='D01', planned_run='A17_beam_full', native_started=False)])

    def test_one_claim_and_duplicate_rejected(self):
        p = self.p / 'batch.json'; save(p, self.batch)
        self.assertEqual(claim(p, 'A17_beam_full', 'D01')['native_attempts_started'], 1)
        with self.assertRaises(ValueError): claim(p, 'A17_beam_full', 'D01')

    def test_paused_cap_invalid_order_and_unclosed_rejected(self):
        for update in [dict(next_native_run_authorized=False), dict(maximum_native_attempts=11),
                       dict(maximum_native_attempts=True), dict(native_attempts_started=10),
                       dict(completed_cycles=1)]:
            with self.subTest(update=update):
                save(self.p/'batch.json', {**self.batch, **update})
                with self.assertRaises(ValueError): claim(self.p/'batch.json', 'A17_beam_full', 'D01')
        save(self.p/'batch.json', self.batch)
        with self.assertRaises(ValueError): claim(self.p/'batch.json', 'A18_beam_full', 'D02')

    def test_resumed_eight_round_window(self):
        from beam_workflow import validate_attempt_window
        auth=dict(authorized_by_human=True,baseline_started=11,baseline_closed=11,
                  additional_native_attempts=8,maximum_native_attempts=19,human_instruction='开始继续修复，上限是八轮')
        base=dict(maximum_native_attempts=19,native_attempts_started=11,completed_cycles=11,
                  resumed_eight_round_authorization=auth,human_continuation_remains_active=True)
        for started in range(11,19):
            b={**base,'native_attempts_started':started,'completed_cycles':started}
            validate_attempt_window(b,f'A{started+17}_part1_probe',f'D{started+1:02d}')
        for delta in [dict(maximum_native_attempts=12),dict(maximum_native_attempts=20),
                      dict(native_attempts_started=19,completed_cycles=19),dict(completed_cycles=12),
                      dict(human_continuation_remains_active=False),dict(resumed_eight_round_authorization={}),
                      dict(resumed_eight_round_authorization={**auth,'additional_native_attempts':9})]:
            with self.subTest(delta=delta):
                with self.assertRaises(ValueError):validate_attempt_window({**base,**delta},'A28_part1_probe','D12')
        for run,cycle in [('A29_part1_probe','D12'),('A28_part1_probe','D13')]:
            with self.assertRaises(ValueError):validate_attempt_window(base,run,cycle)

    def test_new_single_round_supersedes_old_paused_window_without_extra_budget(self):
        from beam_workflow import validate_attempt_window
        auth = dict(authorized_by_human=True, baseline_started=26, baseline_closed=26,
                    additional_native_attempts=1, maximum_native_attempts=27,
                    cycle='D27', run='A43_beam_full', human_instruction='new final round')
        base = dict(maximum_native_attempts=27, native_attempts_started=26,
                    completed_cycles=26, latest_human_instruction='new final round',
                    human_continuation_remains_active=True, pause_after_cycle='D27',
                    resumed_single_round_authorization=auth,
                    resumed_five_round_authorization={'historical': True})
        validate_attempt_window(base, 'A43_beam_full', 'D27')
        for update in [dict(completed_cycles=25), dict(maximum_native_attempts=28),
                       dict(native_attempts_started=27), dict(pause_after_cycle='D28'),
                       dict(latest_human_instruction='old stop'),
                       dict(human_continuation_remains_active=False)]:
            with self.subTest(update=update):
                with self.assertRaises(ValueError):
                    validate_attempt_window({**base, **update}, 'A43_beam_full', 'D27')
        for key, value in [('authorized_by_human', False), ('baseline_started', 25),
                           ('baseline_closed', 25), ('additional_native_attempts', 2),
                           ('maximum_native_attempts', 28), ('run', 'A44_beam_full'),
                           ('cycle', 'D28')]:
            bad = copy.deepcopy(base); bad['resumed_single_round_authorization'][key] = value
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    validate_attempt_window(bad, 'A43_beam_full', 'D27')

    def fixture(self):
        source, tools, report, delta = [self.p / n for n in ('source.json','tools.json','report.json','delta.json')]
        save(source, {'source':'test'}); save(tools, {'tool':'test'})
        save(report, dict(passed=True, cases=12))
        evidence = dict(path=str(report), sha256=sha(report), assertions=[
            dict(pointer='/passed', equals=True), dict(pointer='/cases', minimum=12)])
        value = dict(run_name='A17_beam_full', cycle='D01',
                     source_manifest_sha256=sha(source), tools_manifest_sha256=sha(tools),
                     changed_factors=['fixture'], checks={n:dict(scope='fixture',reports=[evidence]) for n in CHECKS})
        save(delta, value)
        gate = {**{k:value[k] for k in ('run_name','cycle')},
                'delta_manifest':dict(path=str(delta), sha256=sha(delta))}
        return gate, source, tools, report, delta, value

    def test_complete_delta_and_counts(self):
        g,s,t,_,_,_ = self.fixture()
        self.assertEqual(validate_delta(g,s,t)['checked_assertions'], 12)

    def test_empty_passed_stale_and_missing_check_rejected(self):
        for kind in ('no_count','bad_count','missing_check','source_change','report_change','nan'):
            with self.subTest(kind=kind):
                g,s,t,r,d,v = self.fixture()
                if kind=='no_count':
                    for c in v['checks'].values():c['reports'][0]['assertions']=[dict(pointer='/passed',equals=True)]
                elif kind=='bad_count':save(r,dict(passed=True,cases=0))
                elif kind=='missing_check':v['checks'].pop(next(iter(CHECKS)))
                elif kind=='source_change':save(s, {'source':'changed'})
                elif kind=='report_change':save(r,dict(passed=False,cases=12))
                elif kind=='nan':r.write_text('{"passed":true,"cases":NaN}')
                if kind in ('bad_count','nan'):
                    for c in v['checks'].values():c['reports'][0]['sha256']=sha(r)
                save(d,v);g['delta_manifest']['sha256']=sha(d)
                with self.assertRaises((ValueError,KeyError)):validate_delta(g,s,t)


if __name__ == '__main__':
    unittest.main()
