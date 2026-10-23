# Simulation source

이 폴더는 ZAV 센서·기체·PX4 airframe·실행기의 배포 원본입니다. 설치 방법과 검증 범위는 [루트 README](../README.md)를 참고하세요.

`deploy_sim.py`가 고정 버전 Pegasus checkout에 overlay와 USD를 배포합니다. 실제 사용자 경로와 플랫폼은 `../.zav.env`에 저장합니다. 수정 후 `scripts/configure.py`를 재실행해 배포하세요. 이전 `setup_sim.sh`는 새 bootstrap 설치기로 연결되는 호환 명령입니다.
