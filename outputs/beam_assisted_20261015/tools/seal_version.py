"""Seal the current authoritative assisted source/tools once before a run."""
import argparse,json,tarfile
from pathlib import Path
from beam_workflow import save,sha

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('version');ap.add_argument('run');a=ap.parse_args()
    p=Path(__file__).resolve().parents[1];t=p/'tools';repo=p/'assisted_repository'
    old=json.loads((t/'assisted_manifest.json').read_text());tools=json.loads((t/'assisted_tools_manifest.json').read_text())
    if len(old)!=889 or len(tools)!=20:raise ValueError('Reviewed889/20 manifest coverage changed')
    source={name:sha(repo/name) for name in sorted(old)};current_tools={name:sha(t/name) for name in sorted(tools)}
    reportpath=p/(a.run+'_sealed_version.json')
    archives=[p/'versions'/('assisted_'+a.version+'_before_'+a.run+'.tar.gz'),p/'versions'/('assisted_'+a.version+'_tools_before_'+a.run+'.tar.gz')]
    if reportpath.exists() or any(f.exists() for f in archives):raise ValueError('Do not replace a sealed version')
    save(t/'assisted_manifest.json',source);save(t/'assisted_tools_manifest.json',current_tools)
    archives[0].parent.mkdir(exist_ok=True)
    with tarfile.open(archives[0],'w:gz') as bundle:
        for name in source:bundle.add(repo/name,arcname=name,recursive=False)
    with tarfile.open(archives[1],'w:gz') as bundle:
        for name in list(current_tools)+['assisted_manifest.json','assisted_tools_manifest.json']:bundle.add(t/name,arcname=name,recursive=False)
    report=dict(version=a.version,run=a.run,source_files=889,critical_tools=20,
        source_manifest_sha256=sha(t/'assisted_manifest.json'),tools_manifest_sha256=sha(t/'assisted_tools_manifest.json'),
        changed_source_paths=[n for n in source if source[n]!=old[n]],changed_critical_tools=[n for n in tools if tools[n]!=current_tools[n]],
        archives=[dict(path=str(f),bytes=f.stat().st_size,sha256=sha(f)) for f in archives],tool_sha256=sha(__file__))
    save(reportpath,report);print(json.dumps(report))
if __name__=='__main__':main()
