"""One media/metadata transfer, with each server file's exact SHA and size."""
import io,json,sys,tarfile,argparse
from pathlib import Path, PurePosixPath
from beam_workflow import sha
p=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('run');ap.add_argument('cycle');ap.add_argument('--extra',action='append',default=[]);args=ap.parse_args()
run,cycle=args.run,args.cycle;label=run.split('_')[0]
names=[run+'_review_bundle.tar.gz',label+'_key_nodes.tar.gz',run+'_review_bundle_manifest.json',run+'_archive_manifest.json',cycle+'_postprocess_pipeline.json','repair_batch_D_20261016.json','batch_D_20261016/timing_events.jsonl']
names += [cycle+s for s in ('_launch_receipt.json','_first_action_receipt.json','_raw_index_validation.json','_handoff_analysis.json','_timing_summary.json','_timing_summary_reviewed.json','_alignment_native_analysis.json','_failed_carry_corridor.json','_first_column_handoff_analysis.json') if (p/(cycle+s)).exists()]
names += [cycle+s for s in ('_execution_evidence.json','_original_candidates_review.json',
                            '_alignment_native_analysis_reviewed.json') if (p/(cycle+s)).exists()]
for name in args.extra:
    rel=PurePosixPath(name)
    if rel.is_absolute() or '..' in rel.parts or str(rel)!=name or not (p/name).is_file() or not (p/name).resolve().is_relative_to(p.resolve()):raise ValueError('Extra file must remain inside the explicit project root')
    names.append(name)
names=list(dict.fromkeys(names))
rows=[dict(relative_path=n,bytes=(p/n).stat().st_size,sha256=sha(p/n)) for n in names]
manifest=json.dumps(dict(run=run,cycle=cycle,files=rows,tool_sha256=sha(__file__),explicit_extras=args.extra),indent=2).encode()
with tarfile.open(fileobj=sys.stdout.buffer,mode='w|') as archive:
    info=tarfile.TarInfo('transfer_manifest.json');info.size=len(manifest);archive.addfile(info,io.BytesIO(manifest))
    for n in names:archive.add(p/n,arcname=n,recursive=False)
