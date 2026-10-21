from pathlib import Path
import sys,json,hashlib,tarfile,subprocess
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');name=sys.argv[1];D=P/'runs'/name
rows=[]
for f in sorted((D/'episode_0000_live_videos').glob('*.mp4')):
 probe=subprocess.run(['/Embodied_AI/miniconda3/envs/fabrica/bin/ffprobe','-v','error','-count_frames','-select_streams','v:0','-show_entries','stream=nb_read_frames,r_frame_rate,duration','-of','json',str(f)],capture_output=True,text=True,check=True)
 decode=subprocess.run(['/Embodied_AI/miniconda3/envs/fabrica/bin/ffmpeg','-v','error','-i',str(f),'-f','null','-'],capture_output=True,text=True)
 rows.append({'path':str(f),'relative_path':str(f.relative_to(D)),'size':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'probe':json.loads(probe.stdout),'decode_exit':decode.returncode,'decode_errors':decode.stderr})
(D/'video_manifest.json').write_text(json.dumps(rows,indent=2)+'\n')
assert len(rows)==3 and all(r['decode_exit']==0 and not r['decode_errors'].strip() for r in rows)
archive=P/(name+'_evidence.tar.gz')
with tarfile.open(archive,'w:gz',compresslevel=3) as t:
 for f in sorted(D.rglob('*')):
  if not f.is_file() or 'episode_0000_live_frames' in f.parts:continue
  t.add(f,arcname=str(f.relative_to(P)),recursive=False)
 log=P/(name.split('_')[0]+'_launch.log')
 if log.exists():t.add(log,arcname=log.name)
report={'archive':str(archive),'size':archive.stat().st_size,'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'videos':rows,'png_frames_excluded':True,'all_other_original_run_files_included':True}
(P/(name+'_archive_manifest.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['archive','size','sha256']}))
