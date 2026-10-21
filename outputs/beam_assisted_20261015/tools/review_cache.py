"""Small source-bound cache for derived evidence, never for native runs."""
import hashlib
import json
from pathlib import Path


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def file_bindings(paths):
    return {str(Path(p).resolve()): dict(bytes=Path(p).stat().st_size, sha256=digest(p))
            for p in paths}


def reusable(record, binding):
    if not record or record.get('returncode') != 0 or record.get('binding') != binding:
        return False
    outputs = record.get('outputs', {})
    if not outputs:
        return False
    for name, expected in outputs.items():
        p = Path(name)
        if not p.is_file() or p.stat().st_size != expected['bytes'] or digest(p) != expected['sha256']:
            return False
    return True


def phase_entries(history):
    """Timeout/retry records do not overwrite an actual phase entry."""
    result = {}
    for row in history:
        if row.get('event') == 'initialize':
            result.setdefault(row['phase'], row['step_counter'])
        elif (row.get('transition_type') == 'advance'
              and row.get('to_phase') != row.get('from_phase')):
            result.setdefault(row['to_phase'], row['step_counter'])
    return result


def dynamic_mesh_bodies(geometry):
    """Match the sealed monitor's dynamic-pose requirement, not mesh count.

    Noncolliding table legs/decorations have exported fixed mesh body_rest
    transforms, not per-tick native readers. No robot or Beam part may fall
    back to rest. The complete CAD monitor still includes all exported meshes.
    """
    return {m['body'] for m in geometry['visible_meshes']
            if m.get('robot') or str(m.get('object', '')).startswith('fabrica_beam_')}


def require_finished(directory):
    d = Path(directory)
    if not (d / 'finished_at').is_file() or not (d / 'source_after.json').is_file():
        raise ValueError('Only finished native runs may enter postprocessing')
    source = json.loads((d / 'source_after.json').read_text())
    if source.get('differences') or source.get('tool_differences'):
        raise ValueError('Source/tool drift needs investigation before evidence reuse')
    return source


def immutable_inputs(directory):
    """Hash small identity files and bind indexed raw files to immutable stats.

    Raw SHA values come from the original one-time index. Its recorded size and
    mtime are checked on every resume; edited raw files invalidate the cache.
    """
    d = Path(directory)
    names = ['source_after.json', 'collect_results.json', 'task_config_snapshot.json',
             'scene_geometry.json', 'post_failure_recording.json']
    binding = {n: digest(d / n) for n in names if (d / n).is_file()}
    raw_names = ['assisted_trace.jsonl', 'motion_trace.jsonl', 'physics_boundary_trace.jsonl',
                 'entity_geometry_trace.jsonl', 'video_frame_clock.jsonl', 'visual_support_trace.jsonl']
    binding['raw_stats'] = {n: [int((d / n).stat().st_size), int((d / n).stat().st_mtime_ns)]
                            for n in raw_names if (d / n).is_file()}
    binding['video_stats'] = {f.name: [f.stat().st_size, f.stat().st_mtime_ns]
                              for f in sorted((d/'episode_0000_live_videos').glob('*.mp4'))}
    index = d / 'raw_offset_index.json'
    if index.is_file():
        data = json.loads(index.read_text())
        if data['source_after_sha256'] != binding['source_after.json']:
            raise ValueError('Raw index belongs to another source-after identity')
        for n, row in data['files'].items():
            st = (d / n).stat()
            if (st.st_size, st.st_mtime_ns) != (row['bytes'], row['mtime_ns']):
                raise ValueError('Indexed raw file changed: ' + n)
    return binding
