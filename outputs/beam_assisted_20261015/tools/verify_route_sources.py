from pathlib import Path
import hashlib,json,os,sys
C=Path(os.environ['BEAM_ROUTE_SOURCE']);manifest=Path(os.environ['BEAM_ROUTE_MANIFEST']);expected=json.loads(manifest.read_text())
rows=[];different=[]
for name,sha in expected.items():
 p=C/name;actual=hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
 rows.append({'path':name,'sha256':actual})
 if sha!=actual:different.append(name)
report={'source':str(C),'route':os.environ['BEAM_EXECUTION_ROUTE'],'files':rows,'differences':different}
tool_manifest=os.environ.get('BEAM_ROUTE_TOOLS_MANIFEST')
if tool_manifest:
 root=Path(tool_manifest).parent;tool_expected=json.loads(Path(tool_manifest).read_text());tool_rows=[];tool_diff=[]
 for name,sha in tool_expected.items():
  p=root/name;actual=hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
  tool_rows.append({'path':name,'sha256':actual})
  if actual!=sha:tool_diff.append(name)
 report.update(tool_files=tool_rows,tool_differences=tool_diff)
 if tool_diff:different.extend('route_tool:'+name for name in tool_diff)
Path(sys.argv[1]).write_text(json.dumps(report,indent=2)+'\n')
if different:raise SystemExit('source identity mismatch: '+str(different))
print(json.dumps({'verified_files':len(rows),'source_differences':different}))
