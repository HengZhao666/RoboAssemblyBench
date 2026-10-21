"""Bounded transport check for the exact, unchanged primary scene URL only."""
from pathlib import Path
import hashlib,json,time,urllib.request
URL='https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd'

def fetch(attempts=3):
    start=time.monotonic()
    assert type(attempts) is int and 1<=attempts<=3
    failures=[]
    for attempt in range(attempts):
        try:
            with urllib.request.urlopen(URL,timeout=15) as response:
                assert response.status==200
                data=response.read(1024*1024+1)
                assert 0<len(data)<=1024*1024 and data.startswith((b'#usda',b'PXR-USDC'))
                return dict(url=URL,status=response.status,bytes=len(data),sha256=hashlib.sha256(data).hexdigest(),
                            etag=response.headers.get('ETag'),elapsed_s=time.monotonic()-start,
                            request_attempts=attempt+1,prior_transport_failures=failures)
        except (TimeoutError,ConnectionError,urllib.error.URLError,OSError) as exc:
            failures.append(dict(attempt=attempt+1,error=repr(exc)))
            if attempt+1==attempts:raise

def install(fixture,output):
    from internutopia.core.scene.isaacsim.scene import IsaacsimScene
    report=fetch();expected=fixture['bounded_primary_scene_probe']
    assert report['url']==expected['url']==URL and report['sha256']==expected['sha256']
    original=IsaacsimScene._scene_asset_exists
    def exists(cls,path):
        if path==URL:return True
        return original(path)
    IsaacsimScene._scene_asset_exists=classmethod(exists)
    report.update(scope='Only replace blocking SDK existence transport for the exact checked URL. Same URL is loaded by original USD/PhysX scene loader. No local/fallback substitution or physics/state edit.')
    (Path(output)/'bounded_primary_scene_runtime.json').write_text(json.dumps(report,indent=2)+'\n')
