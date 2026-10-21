"""Index immutable finished raw traces once; fetch exact original rows by step.

Original reports remain independent. No phase label is used as a clock key.
"""
import argparse,hashlib,json
from pathlib import Path
from beam_workflow import save,sha

FILES=('assisted_trace.jsonl','motion_trace.jsonl','physics_boundary_trace.jsonl',
       'entity_geometry_trace.jsonl','video_frame_clock.jsonl','visual_support_trace.jsonl')

def build(directory):
    d=Path(directory)
    if not (d/'finished_at').exists() or not (d/'source_after.json').exists():
        raise ValueError('Index only after native completion/source-after; running files are mutable')
    out=dict(schema='beam_raw_offset_index_v1',run=d.name,source_after_sha256=sha(d/'source_after.json'),files={})
    for name in FILES:
        f=d/name
        if not f.exists():continue
        before=f.stat();rows=[];h=hashlib.sha256();offset=0
        with f.open('rb') as stream:
            for line in stream:
                h.update(line);r=json.loads(line)
                stamp=r.get('physics_stamp') or {}
                step=r.get('step',r.get('cycle_step',r.get('task_step_label')))
                rows.append([offset,len(line),step,r.get('boundary'),stamp.get('epoch'),
                             stamp.get('index',r.get('physics_step_index'))])
                offset+=len(line)
        after=f.stat()
        if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):raise ValueError('Raw file changed during indexing')
        out['files'][name]=dict(bytes=after.st_size,mtime_ns=after.st_mtime_ns,sha256=h.hexdigest(),rows=rows)
    save(d/'raw_offset_index.json',out)
    return out

def select(directory,name,minimum,maximum,boundary=None):
    d=Path(directory);idx=json.loads((d/'raw_offset_index.json').read_text())
    if idx['run']!=d.name or sha(d/'source_after.json')!=idx['source_after_sha256']:raise ValueError('Index identity stale')
    meta=idx['files'][name];f=d/name;stat=f.stat()
    if (stat.st_size,stat.st_mtime_ns)!=(meta['bytes'],meta['mtime_ns']):raise ValueError('Raw file changed after indexing')
    result=[]
    with f.open('rb') as stream:
        for offset,size,step,kind,epoch,index in meta['rows']:
            if type(step) is not int or not minimum<=step<=maximum or (boundary and kind!=boundary):continue
            stream.seek(offset);result.append(json.loads(stream.read(size)))
    return result

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('directory',type=Path)
    ap.add_argument('--file',choices=FILES);ap.add_argument('--minimum',type=int,default=0)
    ap.add_argument('--maximum',type=int,default=2147483647);ap.add_argument('--boundary')
    a=ap.parse_args()
    if a.file:
        for row in select(a.directory,a.file,a.minimum,a.maximum,a.boundary):print(json.dumps(row))
    else:
        result=build(a.directory);print(json.dumps({k:dict(rows=len(v['rows']),sha256=v['sha256']) for k,v in result['files'].items()}))
if __name__=='__main__':main()
