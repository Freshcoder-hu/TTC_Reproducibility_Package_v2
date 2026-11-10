from pathlib import Path
import json,zipfile,hashlib,difflib
ROOT=Path(__file__).resolve().parents[1]
WORK=ROOT.parent
OUT=WORK/'output';OUT.mkdir(exist_ok=True)
p=ROOT/'results/formal.jsonl'
raw=p.read_bytes();raw=raw[:raw.rfind(b'\n')+1]
rows=[json.loads(x) for x in raw.splitlines()]
status=dict(status='complete' if len(rows)==2340 and not any(x.get('error') for x in rows) else 'running',
 planned=2340,completed=len(rows),errors=sum(bool(x.get('error')) for x in rows),
 source_sha256=json.loads((ROOT/'results/formal.manifest.json').read_text())['source_sha256'])
(ROOT/'execution_status.json').write_text(json.dumps(status,indent=2))
base=WORK/'tmp/review/bundle/code'
patch=[]
for q in sorted((ROOT/'code').rglob('*')):
 if q.suffix not in ['.py','.cpp'] or '__pycache__' in q.parts:continue
 rel=q.relative_to(ROOT/'code');old=base/rel
 a=old.read_text().splitlines(True) if old.exists() else []
 b=q.read_text().splitlines(True)
 if a!=b:patch.extend(difflib.unified_diff(a,b,fromfile='original/'+str(rel),tofile='supplement/'+str(rel)))
(ROOT/'source_changes.patch').write_text(''.join(patch))
items=[]
for q in sorted(ROOT.rglob('*')):
 if not q.is_file() or '__pycache__' in q.parts or q.name=='MANIFEST.json':continue
 content=raw if q==p else q.read_bytes()
 items.append((q.relative_to(ROOT),content))
manifest=dict(status=status,files=[dict(path=str(q),size=len(data),sha256=hashlib.sha256(data).hexdigest()) for q,data in items])
(ROOT/'MANIFEST.json').write_text(json.dumps(manifest,indent=2))
items.append((Path('MANIFEST.json'),(ROOT/'MANIFEST.json').read_bytes()))
path=OUT/'TTC_Robustness_Experiment.zip'
with zipfile.ZipFile(path,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for rel,data in items:z.writestr(str(Path('TTC_Robustness_Experiment')/rel),data)
print(json.dumps(dict(path=str(path),size=path.stat().st_size,status=status)))
