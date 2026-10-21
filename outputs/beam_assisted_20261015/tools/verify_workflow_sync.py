"""Verify exact current document destinations; archived basenames are distinct."""
import argparse,json
from pathlib import Path
from beam_workflow import sha,save

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('log',type=Path)
    ap.add_argument('--output',type=Path);a=ap.parse_args()
    p=Path(__file__).resolve().parents[1]
    expected=json.loads((p/'documentation_expected_sha256.json').read_text())
    r=Path('/home/user/beam_integration_20261010_upstream6f57dae');remote=r/'beam_assisted_20261015'
    roots=[r,r/'candidate_repository',Path('/home/user/RoboAssemblyBench'),remote/'assisted_repository']
    documents={str(root/n):h for root in roots for n,h in expected.items()}
    report=json.loads(a.log.read_text());found=set();metadata=0;allpaths=set()
    if report['passed'] is not True or report['coordinated_document_copies']!=24:raise ValueError('Server synchronization did not pass')
    for row in report['files']:
        path=row['path']
        if path in allpaths:raise ValueError('Duplicate server destination')
        allpaths.add(path)
        if path in documents:
            if row['sha256']!=documents[path]:raise ValueError('Current document SHA mismatch: '+path)
            found.add(path)
        else:
            local=p/Path(path).relative_to(remote)
            if sha(local)!=row['sha256'] or local.stat().st_size!=row['bytes']:raise ValueError('Metadata SHA mismatch: '+path)
            metadata+=1
    if found!=set(documents):raise ValueError('Incomplete current24 document coverage')
    result=dict(passed=True,server_current_document_copies=24,metadata_files=metadata,report_sha256=sha(a.log),scope='Current destinations checked by complete path; historical same-basename copies verified independently.')
    if a.output:save(a.output,dict(verification=result,server_report=report))
    print(json.dumps(result))
if __name__=='__main__':main()
