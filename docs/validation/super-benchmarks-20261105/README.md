# SUPER 공식 벤치마크 재현 검증 — 2026-11-05

고정 커밋 `2ad3419c127a617c6d7df6925e81a14175a9c096`의 원본 지도·설정·목표로 ROS 2 Humble에서 실행했다. **perfect_drone_sim 결과이며 PX4 비행 결과가 아니다.**

## 통과한 검증

| 실행 | GUI | 경로 길이 | 최고 속도 | 원본 점군 최소 거리 | 출발 프로세스 생성 후 복귀 도착 |
|---|---|---:|---:|---:|---:|
| [dense-2](dense-2/report.json) | 없음 | 215.32m | 8.000m/s | 0.380m | 42.91s |
| [dense-4](dense-4/report.json) | 전체 지도 + FPV | 217.50m | 8.001m/s | 0.312m | 45.23s |
| [high-speed-1](high-speed-1/report.json) | 없음 | 204.57m | 25.000m/s | 0.930m | 16.65s |
| [high-speed-3](high-speed-3/report.json) | 전체 지도 | 203.30m | 25.002m/s | 0.954m | 15.76s |

모두 `(0,-50,1.5)`에서 시작해 정확한 두 목표 `(0,50,1.5)`, `(0,-50,1.5)`가 순서대로 발행됐고 각 목표의 원본 1m 반경에 도달했다. 이동 경로의 2cm 이하 간격 선분 표본에서 기체 반경 0.2m 이내의 PCD 점은 발견되지 않았다. 표본 간 오차를 고려해 최소 거리에서 1cm를 뺀 선분 여유 거리 하한도 0.2m를 넘었다. PCD의 NaN은 검사에서 제외했고 실제 원본 파일은 수정하지 않았다.

`dense-4`, `high-speed-3`에서는 RViz 프로세스 비정상 종료도 없었다. dense의 두 GUI 및 high-speed의 GUI를 직접 화면으로 확인했다. 실험 완료 후 검증기가 자신이 시작한 launch/process group을 종료했다. 중복 실행 차단도 확인했다.

최적화 실패/재계획 실패 경고는 원본 플래너에서도 발생한다. 개별 재계획 경고를 무조건 임무 실패로 판정하지 않았다. 최종 도달과 실제 실행 경로를 독립 검사했다. 이번 결과는 이 두 지도와 해당 실행 기록에 대한 검증이며, 모든 임의 목표나 실제 기체에서 무충돌을 보장하지 않는다.

## 수정 중 발견한 문제

- `dense-1`: 비행 자체는 왕복했지만 검증 도구가 goal 토픽을 reliable로 구독해 best-effort 발행을 수신하지 못했다. 이 실행은 검증 통과로 세지 않았다. 구독 QoS를 수정한 `dense-2`부터 목표 순서를 확인했다.
- `high-speed-2`, `dense-3`: 비행 왕복은 마쳤지만 RViz가 Snap의 GTK/GIO 경로를 상속해 GLIBC symbol 오류로 종료했다. 기존 실행 보고서를 GUI 실패로 표시해 남겼다. 실행기에서 해당 환경 변수를 제거했고, `high-speed-3`, `dense-4`에서 수정과 화면을 확인했다. 검증기도 child process 비정상 종료를 실패로 처리하도록 보완했다.

## 기록

- [자동 판정](acceptance.json)
- [원본 파일 해시와 호환성 수정 목록](provenance.json)
- [원본 지도 위 실제 기록 경로와 속도](trajectories.png)
- [dense RViz 화면](dense-rviz.png)
- [high-speed RViz 화면](high-speed-rviz.png)

각 실행 폴더의 `odometry.jsonl`에는 약 100Hz의 위치·속도·stamp가 들어 있다. `launch.log`는 이 머신에 함께 남아 있으나 저장소의 `*.log` 제외 규칙을 따른다.

[사용 및 재실행 방법](../../super-official-benchmarks.md)
