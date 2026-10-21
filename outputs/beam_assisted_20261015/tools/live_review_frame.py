from pathlib import Path
import json,sys,tarfile
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs'/sys.argv[1]
root=D/'episode_0000_live_frames';dirs=[root/('observation_images_'+s) for s in ['front','left_wrist','right_wrist']]
common=set.intersection(*[{p.name for p in d.glob('frame_*.png')} for d in dirs]);frame=max(common);index=int(frame[6:11]);label=sys.argv[1].split('_')[0]+'_'+str(index)
O=P/'live_review';O.mkdir(exist_ok=True);out=O/(label+'.tar.gz')
with tarfile.open(out,'w:gz') as t:
 for d in dirs:t.add(d/frame,arcname=d.name+'_'+str(index)+'.png',recursive=False)
print(json.dumps(dict(archive=str(out),video_frame=index,nominal_task_step=index*8,raw_frame_name=frame)))
