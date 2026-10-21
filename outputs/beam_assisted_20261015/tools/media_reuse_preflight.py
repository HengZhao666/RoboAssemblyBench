"""Check real retained PNG pixels, stats, encoder bytes and live finalize path."""
import ast
import copy
import json
import os
from pathlib import Path
import sys
import tarfile
import time
from types import ModuleType
from collections import OrderedDict
import cv2
import numpy as np
from beam_workflow import save, sha

p = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(p / 'assisted_repository'))
from toolkits.factory_dual_franka_assembly import export_lerobot as new


def project(source, filename):
    tree = ast.parse(source)
    if filename.endswith('export_lerobot.py'):
        tree.body = [n for n in tree.body if not isinstance(n, ast.FunctionDef) or n.name != '_encode_mp4_from_pngs']
    else:
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LiveRolloutVideoRecorder')
        cls.body = [n for n in cls.body if not isinstance(n, ast.FunctionDef) or n.name != 'finalize']
    return ast.dump(tree, include_attributes=False)


def main():
    cycle = sys.argv[1]
    suffix = '_' + sys.argv[2] if len(sys.argv) > 2 else ''
    encoder = p.parent / 'runtime_tools/bin/ffmpeg'
    assert encoder.is_file(), 'production ffmpeg missing'
    os.environ['PATH'] = str(encoder.parent) + os.pathsep + os.environ['PATH']
    root = p / (cycle + '_media_equivalence' + suffix)
    root.mkdir(exist_ok=False)
    archive = p / 'versions/assisted_v21_before_A18_beam_full.tar.gz'
    base = 'toolkits/factory_dual_franka_assembly/'
    names = [base + 'export_lerobot.py', base + 'generate_demos.py']
    with tarfile.open(archive) as stream:
        old = {n: stream.extractfile(n).read().decode() for n in names}
    current = {n: (p / 'assisted_repository' / n).read_text() for n in names}
    for name in names:
        assert project(old[name], name) == project(current[name], name), name
    namespace = dict(__file__=str(p / 'assisted_repository' / names[0]),
                     __name__='_original_encoder', __package__='toolkits.factory_dual_franka_assembly')
    exec(compile(old[names[0]], namespace['__file__'], 'exec'), namespace)
    old_encode = namespace['_encode_mp4_from_pngs']
    tree = ast.parse(current[names[1]])
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LiveRolloutVideoRecorder')
    module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0), cls], type_ignores=[])
    global_vars = dict(np=np, cv2=cv2, Path=Path, time=time, os=os)
    exec(compile(ast.fix_missing_locations(module), names[1], 'exec'), global_vars)
    recorder = global_vars['LiveRolloutVideoRecorder'].__new__(global_vars['LiveRolloutVideoRecorder'])
    recorder.output_dir = root / 'new'
    recorder.episode_idx, recorder.fps, recorder.requested_fps, recorder.frame_stride = 0, 30., 30., 8
    recorder.keep_frames, recorder.frames_root_dir = True, root / 'frames'
    recorder._frame_dirs, recorder._image_stats = OrderedDict(), {}
    recorder._frame_shapes, recorder._written_png_counts = {}, {}
    clocks = [json.loads(v) for v in (p / 'runs/A18_beam_full/video_frame_clock.jsonl').read_text().splitlines()]
    reports = []
    for camera, start in [('observation.images.front', 1280), ('observation.images.left_wrist', 1450), ('observation.images.right_wrist', 2000)]:
        selected = [r for r in clocks if r['camera'] == camera][start:start + 96]
        assert len(selected) == 96
        directory = root / 'frames' / camera.replace('.', '_')
        directory.mkdir(parents=True)
        stats, hashes = new.RunningImageStats(), []
        for i, row in enumerate(selected):
            source = p / 'runs/A18_beam_full' / row['image_path']
            target = directory / ('frame_%05d.png' % i)
            os.link(source, target)
            bgr = cv2.imread(str(source), cv2.IMREAD_COLOR)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            stats.update(rgb)
            hashes.append(sha(source))
        recorder._frame_dirs[camera] = directory
        recorder._image_stats[camera] = stats
        recorder._frame_shapes[camera] = (bgr.shape[1], bgr.shape[0])
        recorder._written_png_counts[camera] = 96
        baseline_path = root / 'old' / (camera.replace('.', '_') + '.mp4')
        began = time.monotonic()
        baseline = old_encode(directory, baseline_path, 30.)
        elapsed = time.monotonic() - began
        assert baseline['encoder'] == 'ffmpeg-libx264', 'production H264 encoder was not exercised'
        assert baseline['image_stats'] == stats.to_dict(), 'capture/readback pixel statistics differ'
        reports.append(dict(camera=camera, baseline_path=str(baseline_path), old_encode_s=elapsed,
                            stats=baseline['image_stats'], source_png_sha256=hashes))
    result = recorder.finalize()
    for row in reports:
        camera = row['camera']
        summary = result['summaries'][camera]
        assert summary['image_stats'] == row['stats'] and summary['frame_count'] == 96
        assert sha(row['baseline_path']) == sha(result['videos'][camera]), 'encoded original pixels/order changed'
        row.update(new_encode_s=summary['encode_elapsed_s'], video_sha256=sha(result['videos'][camera]),
                   byte_identical=True, capture_stats_equal=True)
    directory = next(iter(recorder._frame_dirs.values()))
    valid = next(iter(recorder._image_stats.values())).to_dict()
    for kind in ('count', 'shape', 'nonfinite'):
        bad = copy.deepcopy(valid)
        if kind == 'count': bad['count'] = [95]
        elif kind == 'shape': bad['mean'] = [0, 0, 0]
        else: bad['std'][0][0][0] = float('nan')
        try: new._encode_mp4_from_pngs(directory, root / ('invalid_' + kind + '.mp4'), 30., precomputed_image_stats=bad)
        except ValueError: pass
        else: raise AssertionError('invalid statistics accepted: ' + kind)
    report = dict(passed=True, cycle=cycle, cameras=3, frames_per_camera=96,
                  stats_and_encoded_bytes_identical=True, invalid_stats_rejected=3,
                  other_source_AST_unchanged=True, source_sha256={n: sha(p / 'assisted_repository' / n) for n in names},
                  encoder_binary_sha256=sha(encoder), tool_sha256=sha(__file__),
                  original_archive_sha256=sha(archive), results=reports, native_physics_executed=0,
                  scope='Unchanged capture class; exact retained PNG pixels and statistics. Current production finalize writes byte-identical MP4s with identical encoder parameters. Measured96-frame subsets only; no full-run speedup claim.')
    save(p / (cycle + '_media_reuse' + suffix + '.json'), report)
    print(json.dumps({k: report[k] for k in ('passed', 'cameras', 'frames_per_camera', 'stats_and_encoded_bytes_identical')}))


if __name__ == '__main__':
    main()
