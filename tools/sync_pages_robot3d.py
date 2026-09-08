"""Copy the shared viewer into GitHub Pages; --check detects stale assets."""
import argparse
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]

def sync(check=False):
    source = ROOT / 'robot_system/web/static/robot3d'
    target = ROOT / 'docs/robot3d'
    differences = []
    for item in sorted(source.rglob('*')):
        if not item.is_file():
            continue
        relative = item.relative_to(source)
        # Never publish machine-specific calibration reviews.
        if relative.name == 'calibration-review.json':
            continue
        dest = target / relative
        if not dest.is_file() or dest.read_bytes() != item.read_bytes():
            differences.append(str(relative))
            if not check:
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(item, dest)
    if check and differences:
        raise SystemExit('Pages assets out of date: ' + ', '.join(differences))
    print(f'Pages viewer assets {"checked" if check else "synced"}; {len(differences)} differences')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    sync(parser.parse_args().check)
