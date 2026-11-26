#!/usr/bin/env python3
"""Compare a Beam run's progress rows with the verified v239 run (wall-clock time ignored).

  python3 compare_progress.py RUN_OUTPUT_DIR            # smoke: all rows but the last (smoke stop row)
  python3 compare_progress.py RUN_OUTPUT_DIR --full     # full run: every row, plus exit code

RUN_OUTPUT_DIR is the folder holding progress_history.jsonl (smoke_out/ or the full-run OUT).
"""
import json
import sys
from pathlib import Path

REFERENCE = Path(__file__).resolve().parent / 'reference_output_v239'


def rows(path):
    out = []
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        row.pop('elapsed_wall_seconds', None)
        out.append(row)
    return out


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    run = Path(sys.argv[1])
    full = '--full' in sys.argv[2:]
    mine, ref = rows(run / 'progress_history.jsonl'), rows(REFERENCE / 'progress_history.jsonl')
    body = mine if full else mine[:-1]
    first_diff = next((i for i, (a, b) in enumerate(zip(body, ref)) if a != b), None)
    report = {
        'rows': len(mine), 'compared_rows': len(body), 'reference_rows': len(ref),
        'identical_prefix': first_diff is None and len(body) <= len(ref),
        'first_differing_row': first_diff,
    }
    if first_diff is not None:
        report['run_row'], report['reference_row'] = body[first_diff], ref[first_diff]
    if full:
        report['row_count_matches'] = len(mine) == len(ref)
        code = run / 'exit_code'
        report['exit_code'] = code.read_text().strip() if code.exists() else None
        report['passed'] = report['identical_prefix'] and report['row_count_matches'] and report['exit_code'] == '0'
    else:
        report['passed'] = report['identical_prefix']
    print(json.dumps(report, indent=1, ensure_ascii=False))
    sys.exit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
