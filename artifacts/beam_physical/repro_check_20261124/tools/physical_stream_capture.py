"""Opt-in physical capture, adapted from verified assisted P02.

No controller, clock, camera, cadence or simulation mutation. Full ordered MP4;
phase boundary raw PNGs and complete existing failure-tail samples retained.
"""
import json,queue,subprocess,threading,time
from pathlib import Path
FFMPEG=Path('/home/user/beam_integration_20261010_upstream6f57dae/runtime_tools/bin/ffmpeg')
original_open=open

def install(demo,out):
    Base=demo.LiveRolloutVideoRecorder
    class StreamingRecorder(Base):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self.work = queue.Queue(maxsize=12)
            self.error = None
            self.encoders = {}
            self.worker_s = 0.
            self.last_frames = {}
            self.last_anchors = {}
            self.last_phase = None
            self.video_root = self.output_dir / f'episode_{self.episode_idx:04d}_live_videos'
            self.video_root.mkdir(exist_ok=True)
            self.worker = threading.Thread(target=self.consume, daemon=True)
            self.worker.start()

        def consume(self):
            try:
                while True:
                    item = self.work.get()
                    if item is None:
                        break
                    key, frame, anchor = item
                    if frame is None:
                        import cv2
                        path, saved_frame = anchor
                        if not cv2.imwrite(str(path), cv2.cvtColor(saved_frame, cv2.COLOR_RGB2BGR)):
                            raise RuntimeError('last phase image write failed: '+str(path))
                        continue
                    start = time.perf_counter()
                    if key not in self.encoders:
                        width, height = frame.shape[1], frame.shape[0]
                        path = self.video_root / (key.replace('.', '_')+'.mp4')
                        cmd = [str(FFMPEG),'-threads','1','-hide_banner','-loglevel','error','-y','-f','rawvideo',
                               '-pix_fmt','rgb24','-s',f'{width}x{height}','-r',str(self.fps),
                               '-i','-','-an','-c:v','libx264','-preset','medium','-crf','18',
                               '-pix_fmt','yuv420p',str(path)]
                        log = original_open(self.video_root/(key.replace('.', '_')+'.encoder.log'),'wb')
                        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=log)
                        self.encoders[key] = (proc, path, log)
                    self._image_stats[key].update(frame)
                    self.encoders[key][0].stdin.write(frame.tobytes())
                    self.last_frames[key] = frame
                    if anchor:
                        import cv2
                        if not cv2.imwrite(str(anchor), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)):
                            raise RuntimeError('key image write failed: '+str(anchor))
                    self.worker_s += time.perf_counter()-start
            except BaseException as exc:
                self.error = repr(exc)
            finally:
                for proc, path, log in self.encoders.values():
                    try:
                        proc.stdin.close()
                        if proc.wait(timeout=60) != 0:
                            self.error = self.error or 'encoder failed: '+str(path)
                    except BaseException as exc:
                        self.error = self.error or repr(exc)
                        proc.kill()
                    log.close()

        def enqueue(self, value):
            while True:
                if self.error or not self.worker.is_alive():
                    raise RuntimeError('capture worker failed: '+str(self.error))
                try:
                    self.work.put(value, timeout=.1)
                    return
                except queue.Full:
                    pass

        def record(self, task, obs, *, step_index, post_failure=False, failure_task_step=None):
            if step_index % self.frame_stride:
                return
            from toolkits.factory_dual_franka_assembly.beam_video_clock import video_frame_clock
            phase = (getattr(task, 'phase', None), post_failure)
            keep_anchor = phase != self.last_phase
            phase_changed = self.last_phase is not None and keep_anchor
            self.last_phase = phase
            for spec in self.camera_specs:
                key = spec['video_key']
                frame = self._sensor_frame_from_obs(obs, camera_video_key=key)
                if frame is None:
                    raise RuntimeError('missing camera frame: '+key)
                if key not in self._written_png_counts:
                    self._written_png_counts[key] = 0
                    self._frame_shapes[key] = (frame.shape[1],frame.shape[0])
                    self._image_stats[key] = self._running_image_stats_cls()
                index = self._written_png_counts[key]
                binding = self._camera_bindings[key]
                sensor = ((obs.get(binding['robot_name']) or {}).get('sensors') or {}).get(binding['sensor_name'], {})
                clock = video_frame_clock(task, step=step_index, camera=key, frame_index=index, sensor_observation=sensor)
                clock.update(capture_segment='post_failure' if post_failure else 'rollout',
                             video_path=str((self.video_root/(key.replace('.','_')+'.mp4')).relative_to(self.output_dir)),
                             raw_png_retained=False)
                if post_failure:
                    clock.update(failure_task_step=int(task.step_counter) if failure_task_step is None else failure_task_step,
                                 capture_entry_task_step=int(task.step_counter))
                anchor = None
                if keep_anchor or post_failure:
                    anchors = self.output_dir/'capture_key_frames'
                    anchors.mkdir(exist_ok=True)
                    anchor = anchors/(f'{step_index:06d}_'+key.replace('.','_')+'.png')
                    clock.update(raw_png_retained=True, image_path=str(anchor.relative_to(self.output_dir)))
                if phase_changed and key in self.last_anchors:
                    old_frame,old_path = self.last_anchors[key]
                    self.enqueue((key, None, (old_path, old_frame)))
                raw_path = self.frames_root_dir/key.replace('.', '_')/f'frame_{index:05d}.png'
                raw_path.parent.mkdir(exist_ok=True)
                clock['image_path'] = str((anchor or raw_path).relative_to(self.output_dir))
                self.last_anchors[key] = (frame.copy(), raw_path)
                self.enqueue((key, frame.copy(), anchor))  # Never queue an aliased simulator buffer.
                with (self.output_dir/'video_frame_clock.jsonl').open('a') as h:
                    h.write(json.dumps(clock, allow_nan=False)+'\n')
                self._written_png_counts[key] += 1

        def finalize(self):
            for key,(frame,path) in self.last_anchors.items():
                self.enqueue((key, None, (path, frame)))
            self.enqueue(None)
            self.worker.join(timeout=120)
            if self.worker.is_alive() or self.error:
                raise RuntimeError('capture finalization failed: '+str(self.error))
            summaries = {}
            videos = {}
            import cv2
            for key, frame in self.last_frames.items():
                anchor = self.output_dir/'capture_key_frames'/('last_'+key.replace('.','_')+'.png')
                if not cv2.imwrite(str(anchor), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError('terminal image write failed: '+str(anchor))
            for key, (proc,path,log) in self.encoders.items():
                n = self._written_png_counts[key]
                stats = self._image_stats[key].to_dict()
                if stats['count'] != [n]:
                    raise RuntimeError('capture statistics/frame count mismatch')
                videos[key] = str(path)
                summaries[key] = dict(video_path=str(path),frame_count=n,fps=self.fps,
                                      frame_stride=self.frame_stride,image_stats=stats,
                                      width=self._frame_shapes[key][0],height=self._frame_shapes[key][1],
                                      renderer='live_rollout_stream_capture',raw_png_retained=False)
            (self.output_dir/'capture_performance.json').write_text(json.dumps(dict(streaming=True,encoder_worker_seconds=self.worker_s,frame_counts=self._written_png_counts),indent=2)+'\n')
            return dict(videos=videos,summaries=summaries,video_mode='live_rollout_mp4',
                        frames={},frame_summaries=summaries,frame_mode='stream_no_png')
    demo.LiveRolloutVideoRecorder=StreamingRecorder
