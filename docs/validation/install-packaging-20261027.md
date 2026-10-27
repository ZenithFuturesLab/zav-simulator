# 설치 패키지 준비 검사 — 2026-10-27

대상 브랜치: `ZAVSIM_Fix`.

- 저장소 작업 폴더에서 `./scripts/verify.sh` 통과.
- Git index의 전체 소스만 별도 디렉토리로 내보낸 사본에서 `./scripts/verify.sh` 통과.
  사본에는 `.zav.env`, build/install, 다운로드한 PX4/Agent, 기존 Pegasus checkout이 없음.
  검사에 필요한 시스템 ROS·NumPy·SciPy는 현재 PC의 설치를 사용함.
- `./setup.sh --dry-run` 통과.
- 실제 Windows Isaac `python.bat`에서 `pip install --dry-run --no-index -r`로
  WSL 저장소의 `scripts/requirements-isaac.txt` 읽기와 기존 설치 버전 충족 확인.
- `scripts/audit_package.py`로 포함 파일 크기, 생성물 제외, 알려진 credential 패턴 검사 통과.
- 이번 검사는 새 OS의 apt 설치·외부 소스 다운로드·전체 재빌드를 실행한 검사는 아님.
  GitHub Actions는 push 후 Ubuntu 소스 검사·ROS 빌드를 수행함.
- 실제 비행과 DDS/종료 오류의 확인 범위는 [검증 기록](../verification.md)을 따름.
