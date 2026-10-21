"""Small counterexamples for evidence reuse and timeout clip boundaries."""
import json
from pathlib import Path
import tempfile
import unittest
from review_cache import digest, file_bindings, fingerprint, immutable_inputs, phase_entries, require_finished, reusable, dynamic_mesh_bodies


class ReviewCacheTests(unittest.TestCase):
    def test_visual_static_meshes_do_not_require_native_reader(self):
        geometry=dict(visible_meshes=[dict(body='arm',robot='left',object='robot'),
                      dict(body='part',robot=None,object='fabrica_beam_1'),
                      dict(body='leg',robot=None,object='factory_table_leg')])
        self.assertEqual(dynamic_mesh_bodies(geometry),{'arm','part'})

    def test_robot_or_payload_cannot_fall_back_to_rest(self):
        geometry=dict(visible_meshes=[dict(body='finger',robot='left'),dict(body='part',object='fabrica_beam_0')])
        self.assertEqual(dynamic_mesh_bodies(geometry)-{'finger'},{'part'})

    def test_timeout_does_not_replace_real_entry(self):
        rows=[dict(event='initialize',phase='a',step_counter=0),
              dict(transition_type='advance',from_phase='a',to_phase='close',step_counter=31972),
              dict(transition_type='timeout',from_phase='close',to_phase='close',step_counter=33972)]
        self.assertEqual(phase_entries(rows),{'a':0,'close':31972})

    def test_only_same_input_and_unchanged_output_reuses(self):
        with tempfile.TemporaryDirectory() as tmp:
            f=Path(tmp)/'clip.mp4';f.write_bytes(b'original')
            r=dict(returncode=0,binding='source-clock-frame-range',outputs=file_bindings([f]))
            self.assertTrue(reusable(r,r['binding']))
            self.assertFalse(reusable(r,'other-source-or-frame-range'))
            f.write_bytes(b'changed!')
            self.assertFalse(reusable(r,r['binding']))

    def test_failed_or_missing_output_cannot_reuse(self):
        self.assertFalse(reusable(dict(returncode=1,binding='x',outputs={}), 'x'))
        self.assertFalse(reusable(dict(returncode=0,binding='x',outputs={}), 'x'))
        self.assertFalse(reusable(dict(returncode=0,binding='x',outputs={'/missing/clip':{}}), 'x'))

    def test_only_changed_clip_is_invalidated(self):
        with tempfile.TemporaryDirectory() as tmp:
            files=[Path(tmp)/f'{i}.mp4' for i in range(3)]
            for f in files:f.write_bytes(f.name.encode())
            rows=[dict(returncode=0,binding=f.name,outputs=file_bindings([f])) for f in files]
            files[1].write_bytes(b'broken')
            self.assertEqual([reusable(r,f.name) for r,f in zip(rows,files)],[True,False,True])

    def test_native_or_dirty_source_not_treated_as_finished(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp)
            with self.assertRaises(ValueError):require_finished(d)
            (d/'finished_at').touch();(d/'source_after.json').write_text('{"differences":["changed"]}')
            with self.assertRaises(ValueError):require_finished(d)

    def test_index_creation_does_not_invalidate_same_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);(d/'source_after.json').write_text('{}');raw=d/'assisted_trace.jsonl';raw.write_text('{}\n')
            before=immutable_inputs(d);st=raw.stat()
            (d/'raw_offset_index.json').write_text(json.dumps(dict(source_after_sha256=digest(d/'source_after.json'),files={raw.name:dict(bytes=st.st_size,mtime_ns=st.st_mtime_ns)})))
            self.assertEqual(before,immutable_inputs(d))
            raw.write_text('changed\n')
            with self.assertRaises(ValueError):immutable_inputs(d)

    def test_tool_or_input_change_invalidates_binding(self):
        self.assertNotEqual(fingerprint(dict(tool='old',source='x')),fingerprint(dict(tool='new',source='x')))
        self.assertNotEqual(fingerprint(dict(tool='old',source='x')),fingerprint(dict(tool='old',source='y')))


if __name__=='__main__':unittest.main()
