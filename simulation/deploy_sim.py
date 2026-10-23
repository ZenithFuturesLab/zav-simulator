#!/usr/bin/env python3
"""[ZAV 변경] simulation을 단일 배포 원본으로 사용합니다. Isaac/PX4를 실행하지 않습니다."""
import argparse
from pathlib import Path
import shutil
import tempfile

ROOT = Path(__file__).resolve().parent
SIM_ROOT = Path('extensions/pegasus.simulator/pegasus/simulator')


def files(pegasus, home):
    for name in ('zav_sim.py', 'slam_test_scene.py'):
        yield ROOT / 'pegasus' / name, pegasus / 'examples' / name
    yield ROOT / 'pegasus/zav_v1.py', pegasus / SIM_ROOT / 'logic/vehicles/multirotors/zav_v1.py'
    for directory, target in (('pegasus/overlay', pegasus / SIM_ROOT),
                              ('pegasus/assets/ZAV_v1', pegasus / SIM_ROOT / 'assets/Robots/ZAV_v1'),
                              ('pegasus/assets/Sensors', pegasus / SIM_ROOT / 'assets/Sensors'),
                              ('tools', pegasus / 'tools')):
        for src in sorted((ROOT / directory).rglob('*')):
            if src.is_file() and '__pycache__' not in src.parts:
                yield src, target / src.relative_to(ROOT / directory)


def registrations(pegasus):
    """원본 Pegasus의 로봇 목록만 보완하며 다른 환경의 렌더링 기본값은 건드리지 않습니다."""
    sim = pegasus / SIM_ROOT
    params = sim / 'params.py'
    if not params.exists():
        raise RuntimeError(f'Pegasus checkout이 필요합니다: {params}')
    text = params.read_text()
    if '"ZAV_v1"' not in text:
        marker = 'ROBOTS = {'
        if marker not in text:
            raise RuntimeError('지원하는 Pegasus ROBOTS 정의를 찾지 못했습니다.')
        text = text.replace(marker, marker + '\n    # [ZAV 변경] 로컬 ZAV USD 등록.\n    "ZAV_v1": ROBOTS_ASSETS + "/ZAV_v1/ZAV_v1.usd",', 1)
    yield params, text
    for relative, line in (
        ('logic/vehicles/multirotors/__init__.py', 'from .zav_v1 import ZAV_v1'),
        ('logic/vehicles/__init__.py', 'from .multirotors import ZAV_v1'),
    ):
        path = sim / relative
        content = path.read_text() if path.exists() else ''
        if 'ZAV_v1' not in content:
            content += '\n# [ZAV 변경] 기존 기체 import를 유지하고 ZAV만 추가합니다.\n' + line + '\n'
        yield path, content


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pegasus-dir', type=Path, default=ROOT.parent / 'src/PegasusSimulator')
    parser.add_argument('--home-dir', type=Path, default=Path.home())
    parser.add_argument('--check', action='store_true', help='복사 대신 배포본과 원본의 차이만 확인')
    parser.add_argument('--dry-run', action='store_true', help='쓰기 없이 대상 파일 목록 출력')
    args = parser.parse_args()
    mismatches = []
    for src, dst in files(args.pegasus_dir, args.home_dir):
        if args.dry_run:
            print(f'{src.relative_to(ROOT)} -> {dst}')
        elif args.check:
            if not dst.exists() or src.read_bytes() != dst.read_bytes():
                mismatches.append(str(dst))
        else:
            dst.parent.mkdir(parents=True, exist_ok=True)
            if not dst.exists() or src.read_bytes() != dst.read_bytes():
                # 실행 중인 shell/Python이 열린 이전 inode를 계속 읽을 수 있게 원자적으로 교체합니다.
                with tempfile.NamedTemporaryFile(dir=dst.parent, delete=False) as staged:
                    staged_path = Path(staged.name)
                try:
                    shutil.copy2(src, staged_path)
                    staged_path.replace(dst)
                finally:
                    staged_path.unlink(missing_ok=True)
            if dst.suffix == '.sh':
                dst.chmod(dst.stat().st_mode | 0o111)
    if not args.dry_run:
        entrypoints = []
        for name in ('run_zav_sim.sh', 'run_isaac_sim.sh', 'run_mid360_rviz.sh', 'run_fastlivo.sh'):
            text = ('#!/usr/bin/env bash\n'
                    'source "${XDG_CONFIG_HOME:-$HOME/.config}/zav/simulation.env"\n'
                    'exec bash "$ZAV_WS_DIR/simulation/scripts/' + name + '" "$@"\n')
            entrypoints.append((args.home_dir / name, text))
        for path, text in [*registrations(args.pegasus_dir), *entrypoints]:
            if args.check:
                if not path.exists() or path.read_text() != text:
                    mismatches.append(str(path))
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text)
    if mismatches:
        print('미배포/불일치 파일:\n' + '\n'.join(mismatches))
        return 1
    print('배포 목록 확인 완료' if args.dry_run else '배포본 일치' if args.check else 'ZAV 센서/실행기 배포 완료')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
