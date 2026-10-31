"""Build only code/support/test fixtures; reports and development history are excluded."""
from pathlib import Path
import hashlib, json, shutil, subprocess, zipfile
REPO = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
TEMP = Path('/tmp/cis_assignment2_code_submission')
STAGE = TEMP / 'stage/CIS_Assignment2_CODE_ONLY_CANDIDATE'
FIXTURES = ['assignment2/results/comprehensive_v1_full_summary.json', 'assignment2/results/comprehensive_v1_s001_freeze.json', 'assignment2/results/comprehensive_v1_s001_model.npz', 'assignment2/results/comprehensive_v1_s001_pilot.json', 'assignment2/results/comprehensive_v1_s007_calibration.npz', 'assignment2/results/comprehensive_v1_s007_freeze.json', 'assignment2/results/comprehensive_v1_s007_model.npz', 'assignment2/results/comprehensive_v1_s007_pilot.json', 'assignment2/results/ik_robustness_diagnostic/summary.json', 'assignment2/results/ik_robustness_diagnostic/traces.json.gz', 'assignment2/results/step10_inverse_errors.npz', 'assignment2/results/step10_inverse_summary.json', 'assignment2/results/step8_bound_diagnostic/summary.json', 'assignment2/results/step8_phase_d/acquisition.json', 'assignment2/results/step8_phase_d/calibration.npz', 'assignment2/results/step8_phase_d/fit.npz', 'assignment2/results/step8_phase_d/holdout.npz', 'assignment2/results/step8_phase_d/summary.json', 'assignment2/results/step8_phase_e/fit.npz', 'assignment2/results/step8_phase_e/parameters.json', 'assignment2/results/step8_phase_e/summary.json', 'assignment2/results/step9_forward_errors.npz', 'assignment2/results/step9_forward_summary.json']
if STAGE.exists(): shutil.rmtree(STAGE)
STAGE.mkdir(parents=True)
files = []
for folder in ('src', 'vendor', 'tests', 'experiments'):
    for p in (REPO / 'assignment2' / folder).rglob('*'):
        if not p.is_file() or '__pycache__' in p.parts or '.pytest_cache' in p.parts: continue
        if p.suffix in ('.pyc', '.pyo'): continue
        if folder == 'experiments' and p.suffix != '.py': continue
        files.append(str(p.relative_to(REPO)))
files += ['assignment2/' + p for p in ('configure_environment.py', 'requirements.txt',
          'requirements-lock.txt', 'pytest.ini', 'notebooks/assignment2_template.ipynb')]
files += FIXTURES
manifest = []
for name in sorted(set(files)):
    source = REPO / name
    target = STAGE / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
for target, source in [('README.md', 'CODE_ONLY_README.md'), ('assignment2/SETUP.md', 'CODE_ONLY_SETUP.md')]:
    dst = STAGE / target
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(OUT / source, dst)
for p in sorted(STAGE.rglob('*')):
    if p.is_file():
        assert p.suffix.lower() not in ('.pdf', '.tex', '.zip')
        manifest.append(dict(path=str(p.relative_to(STAGE)), bytes=p.stat().st_size,
                             sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
meta = dict(package_type='CODE_ONLY_CANDIDATE', complete_final_submission=False,
    final_report='PENDING: later add approved PDF under assignment2/notebooks/',
    source_branch=subprocess.check_output(['git','branch','--show-current'],cwd=REPO,text=True).strip(),
    source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
    root=STAGE.name, included=manifest,
    excluded=['all report PDFs and LaTeX', '.git', 'venv', 'caches', 'other assignments',
      'historical notebooks', 'duplicate figures', 'unneeded intermediate results',
      'large evaluation attempt logs', 'previous submission ZIPs and audit artifacts'])
(OUT / 'submission_manifest.json').write_text(json.dumps(meta, indent=2)+'\n')
(OUT / 'submission_manifest.txt').write_text('\n'.join(r['path'] for r in manifest)+'\n')
archive = OUT / (STAGE.name + '.zip')
with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
    for p in sorted(STAGE.rglob('*')):
        if p.is_file(): z.write(p, p.relative_to(STAGE.parent))
with zipfile.ZipFile(archive) as z: assert z.testzip() is None
print(json.dumps(dict(files=len(manifest), zip=str(archive), bytes=archive.stat().st_size)))
