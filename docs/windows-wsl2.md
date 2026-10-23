# Windows 11 + WSL2 Ubuntu 22.04

Isaac Sim은 **Windows에서 실행**하고 PX4·ROS·FAST-LIVO2·RViz는 WSL에서 실행하는 구성입니다. Windows의 Isaac Python 3.11 내부 Humble과 WSL Ubuntu의 ROS Humble Python 3.10을 분리합니다. Windows용 ROS나 Livox SDK를 별도로 설치하지 않습니다.

## 사전 준비

1. Windows 11 22H2 이상, 최신 WSL2, NVIDIA Windows 드라이버, Windows Isaac Sim 5.1.0을 설치합니다. Isaac Sim 기본 예제를 Windows에서 먼저 실행합니다.
2. 관리자 PowerShell에서 Ubuntu 22.04를 설치합니다.

```powershell
wsl --update
wsl --install -d Ubuntu-22.04
```

3. Windows 사용자 폴더의 `%USERPROFILE%\.wslconfig`에 기존 설정과 병합하여 다음을 추가합니다.

```ini
[wsl2]
networkingMode=mirrored
dnsTunneling=true
```

4. 작업을 저장하고 PowerShell에서 `wsl --shutdown` 후 Ubuntu를 다시 엽니다. WSLg GUI가 동작하는지 확인합니다. 자세한 조건은 [Microsoft WSL 네트워크 문서](https://learn.microsoft.com/en-us/windows/wsl/networking)를 참고하세요. 이 구성은 양쪽의 `127.0.0.1` 통신을 사용하므로 기본 NAT 모드로는 같은 실행 설정을 보장하지 않습니다.

## 설치

소스는 성능을 위해 WSL Linux 파일시스템의 `~/zenith_ws`에 clone합니다. Windows 드라이브의 Isaac 설치 폴더는 그대로 사용합니다. 저장소 접근 권한이 필요합니다.

```bash
sudo apt-get update
sudo apt-get install -y git software-properties-common
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get install -y gh
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git
git clone https://github.com/ZenithFuturesLab/zav-simulator.git ~/zenith_ws
cd ~/zenith_ws
./scripts/bootstrap.sh --platform wsl --isaac-windows-python /mnt/c/isaac-sim/python.bat
```

`gh auth login`에 표시된 주소와 일회용 코드를 Windows 브라우저에서 열어 GitHub 인증을 완료하세요.

`python.bat` 경로는 실제 Windows 설치 위치를 `/mnt/c/...` 형식으로 지정합니다. 공백이 있으면 따옴표로 감싸세요. Windows에서 WSL의 파일을 읽을 수 있어야 합니다. 실행기는 `wslpath`와 `WSLENV`로 시뮬레이터·캘리브레이션 경로를 변환합니다.

설치 후 새 WSL 터미널 네 개에서 README의 동일 명령을 실행합니다. `zav_sim`이 Windows Kit와 WSL PX4를 연결합니다.

## 통신과 진단

| 통신 | 용도 |
| --- | --- |
| TCP 4560 | Windows Pegasus ↔ WSL PX4 MAVLink |
| UDP 8888, WSL 내부 | PX4 ↔ Micro XRCE-DDS Agent |
| TCP 17441, 17443, 17444, 17445 | Windows 센서 → WSL relay: pointcloud, camera, IMU, Livox |
| TCP 17442, 17446, 17449, 17450, WSL loopback | Cyclone DDS 센서·mapper·PX4 gateway·flight manager |

WSL ROS 그래프는 Cyclone TCP를 사용하고 PX4 Agent 쪽 Fast DDS와 별도 프로세스 gateway로 연결합니다. gateway는 상태·명령뿐 아니라 `SensorCombined`, `TimesyncStatus`, `vehicle_visual_odometry`도 전달합니다. ROS domain ID는 양쪽 실행기에 동일하게 전달됩니다.

방화벽을 끄지 마세요. 연결이 차단되면 Windows Defender/Hyper-V 방화벽에서 위 프로젝트 포트와 해당 프로그램의 로컬 통신만 허용합니다. 조직 정책이나 VPN에 따라 추가 설정이 필요할 수 있습니다. 네트워크 격리 진단은 Microsoft 문서의 mirrored mode 및 Hyper-V firewall 절을 따르세요.

이 배포본의 실제 Windows GPU 종단 실행은 아직 검증하지 못했습니다. Ubuntu에서 수행한 DDS·센서·경로 테스트는 Windows GUI 검증을 대신하지 않습니다. 설치 후 반드시 [실행 검증](verification.md)을 완료하세요.
