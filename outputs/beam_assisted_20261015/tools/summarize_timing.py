"""Collect measured cycle intervals without summing nested or parallel work."""
import argparse
import json
from pathlib import Path
from beam_workflow import save, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cycle')
    args = parser.parse_args()
    p = Path(__file__).resolve().parents[1]
    directory = p / 'batch_D_20261016'
    candidates = [directory / 'timing_events.jsonl', directory / 'timing_events_server.jsonl',
                  directory / (args.cycle + '_timing_events_server.jsonl')]
    events = {}
    inputs = []
    for source in candidates:
        if not source.exists():
            continue
        inputs.append(dict(path=str(source.resolve()), sha256=sha(source)))
        for line in source.read_text().splitlines():
            row = json.loads(line)
            if row.get('phase', '').startswith(args.cycle):
                events[json.dumps(row, sort_keys=True)] = row
    rows = sorted(events.values(), key=lambda r: r['wall_unix_s'])
    measured = [r for r in rows if r['action'] == 'end' and 'elapsed_s' in r]
    starts = [r for r in rows if r['action'] == 'start']
    ends = [r for r in rows if r['action'] == 'end']
    unmatched_starts = starts.copy()
    unmatched_ends = []
    for end in ends:
        candidates = [s for s in unmatched_starts if s['phase'] == end['phase']
                      and abs(s['wall_unix_s']-(end['wall_unix_s']-end.get('elapsed_s', 0.))) < .25]
        if len(candidates) == 1:
            unmatched_starts.remove(candidates[0])
        else:
            unmatched_ends.append(end)
    pipeline = p / (args.cycle + '_postprocess_pipeline.json')
    sequential = None
    if pipeline.exists():
        stages = json.loads(pipeline.read_text())['stages']
        sequential = sum(s['elapsed_s'] for s in stages)
    native = [r for r in measured if r['phase'] == args.cycle + '_native']
    report = dict(cycle=args.cycle, inputs=inputs, measured_intervals=measured,
                  native_process_s=native[-1]['elapsed_s'] if native else None,
                  sequential_postprocess_s=sequential,
                  failed_invocations=[r for r in measured if r.get('returncode', 0) != 0],
                  start_without_end=[dict(phase=v['phase'], pid=v['pid'], utc=v['utc']) for v in unmatched_starts],
                  end_without_start=[dict(phase=v['phase'], pid=v['pid'], utc=v['utc']) for v in unmatched_ends],
                  scope='Inclusive wall intervals; nested and parallel stages must not be added. Native includes initialization, ACK wait, physics, observation tail and video finalization. Human analysis/edit wall intervals may include tool and scheduling waits. Missing starts are not reconstructed.',
                  limits=['This is an invocation log, not exclusive CPU time or a complete allocation of every wall second.'])
    save(p / (args.cycle + '_timing_summary_reviewed.json'), report)
    print(json.dumps(dict(cycle=args.cycle, measured_intervals=len(measured),
                         native_process_s=report['native_process_s'],
                         failed_invocations=len(report['failed_invocations']),
                         start_without_end=len(report['start_without_end']))))


if __name__ == '__main__':
    main()
