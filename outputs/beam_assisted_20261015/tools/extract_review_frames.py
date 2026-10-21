from pathlib import Path
import subprocess,tarfile,sys,shutil
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
name=sys.argv[1] if len(sys.argv)>1 else 'A03_base_grasp';label=name.split('_')[0]
D=P/'runs'/name;O=P/(label+'_review_frames');O.mkdir(exist_ok=True)
indices=[int(i) for i in sys.argv[2:]] or [232,236,320,360,370,518]
for video in (D/'episode_0000_live_videos').glob('*.mp4'):
 for index in indices:
  target=O/(video.stem+'_'+str(index)+'.png')
  raw=D/'episode_0000_live_frames'/video.stem/f'frame_{index:05d}.png'
  if raw.exists():shutil.copy2(raw,target)
  else:subprocess.run(['/Embodied_AI/miniconda3/envs/fabrica/bin/ffmpeg','-v','error','-i',str(video),'-vf',f"select='eq(n,{index})'",'-frames:v','1','-y',str(target)],check=True)
with tarfile.open(P/(label+'_review_frames.tar.gz'),'w:gz') as t:
 for f in O.glob('*.png'):t.add(f,arcname=f.name)
print('frames',len(list(O.glob('*.png'))))
