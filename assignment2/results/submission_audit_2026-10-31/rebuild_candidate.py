"""Rebuild the reviewed allowlist; never commits, pushes, or marks a report approved."""
from pathlib import Path
import argparse,hashlib,json,zipfile

out=Path(__file__).resolve().parent
repo=out.parents[2]
parser=argparse.ArgumentParser()
parser.add_argument('--accept-reviewed-changes',action='store_true')
parser.add_argument('--final-approved',action='store_true',help='Use only after the owner has reviewed and resolved all blockers')
parser.add_argument('--output',type=Path)
args=parser.parse_args()
manifest=json.loads((out/'submission_manifest.json').read_text())
if args.final_approved:
    readme=(out/'CANDIDATE_README.md').read_text()
    if 'NOT APPROVED' in readme or 'Known unresolved items' in readme:
        raise SystemExit('Review/update the candidate README and resolve the audit blockers before final approval.')
    if not args.accept_reviewed_changes:
        raise SystemExit('Final packaging also requires --accept-reviewed-changes after review.')
    manifest['root']='CIS_Assignment2'
notebook=out/'executed_assignment2_template.ipynb'
current=json.loads((repo/'assignment2/notebooks/assignment2_template.ipynb').read_text())
executed=json.loads(notebook.read_text())
if [(c['cell_type'],c['source']) for c in current['cells']] != [(c['cell_type'],c['source']) for c in executed['cells']]:
    raise SystemExit('The executed notebook is stale. Re-execute the approved current source before packaging.')
for row in manifest['included']:
    path=repo/row['source']
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    if digest!=row['sha256'] and not args.accept_reviewed_changes:
        raise SystemExit(f"Changed source requires review: {row['source']}")
    row.update(sha256=digest,bytes=path.stat().st_size)
target=args.output or out/('CIS_Assignment2_Final.zip' if args.final_approved else 'CIS_Assignment2_CANDIDATE.zip')
with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as z:
    for row in manifest['included']:
        z.write(repo/row['source'],manifest['root']+'/'+row['path'])
with zipfile.ZipFile(target) as z:
    assert z.testzip() is None
    for row in manifest['included']:
        assert hashlib.sha256(z.read(manifest['root']+'/'+row['path'])).hexdigest()==row['sha256']
manifest['ready_for_submission']=bool(args.final_approved)
(out/'submission_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(f'{target}: {len(manifest["included"])} verified files')
