# USB 웹캠 ROS 2 노드

Ubuntu 22.04 또는 WSL2 Ubuntu 22.04(x86_64)에서 실제 웹캠을 ROS 2 Humble에 연결할 수 있습니다. 시뮬레이터 설치 없이 아래 명령만 실행하면 됩니다.

```bash
git clone --depth 1 --single-branch --branch usb-webcam https://github.com/ZenithFuturesLab/zav-simulator.git
cd zav-simulator
./install_usb_webcam.sh
./run_usb_webcam.sh
```

설치 스크립트는 필요한 Ubuntu 패키지가 없을 때 `sudo`를 사용합니다. Ubuntu에서는 Linux UVC/V4L2 드라이버와 ROS `usb_cam`을 사용합니다. WSL에서는 Windows 사용자 폴더에 FFmpeg를 설치하고 Windows DirectShow의 첫 번째 카메라를 사용합니다. WSL에서 Windows 카메라를 사용할 때 USB 패스스루와 Windows 관리자 권한은 필요 없습니다.

다른 터미널에서 같은 저장소 폴더로 이동해 수신을 확인합니다. `--ros`는 WSL에서 구독자에게 웹캠 노드와 같은 로컬 DDS 설정을 적용합니다.

```bash
./run_usb_webcam.sh --ros ros2 topic hz /webcam/image_raw
./run_usb_webcam.sh --ros ros2 topic echo --once /webcam/camera_info
```

### 해상도 선택

| 옵션 | 해상도 | 발행 목표 |
| --- | --- | --- |
| `--480p` (기본값) | 640×480 | 30 FPS |
| `--720p` | 1280×720 | 30 FPS |
| `--1080p` | 1920×1080 | 30 FPS |

```bash
./run_usb_webcam.sh --1080p
./run_usb_webcam.sh /dev/video2 --720p
```

두 번째 명령은 V4L2 장치가 있는 경우에 사용합니다. WSL의 Windows 카메라 경로에서 720p는 MJPEG 60 FPS로 캡처하고 ROS 2에는 30 FPS로 발행합니다. 1080p는 MJPEG 30 FPS로 캡처합니다. 선택한 모드는 카메라에서 지원해야 합니다.

- 노드는 `/webcam/usb_webcam`이고, 보정 파일을 적용하기 전의 `camera_info`에는 카메라 내부 파라미터가 들어 있지 않습니다.
- Ubuntu에서 장치 번호가 다르면 `./run_usb_webcam.sh /dev/video2`처럼 지정합니다. 카메라가 MJPEG만 지원하면 장치 경로 뒤에 `-p pixel_format:=mjpeg2rgb`를 붙입니다.
- Windows에 카메라가 여러 대면 `./run_usb_webcam.sh --camera "HD USB Camera"`처럼 Windows에 표시된 이름을 지정합니다. Windows의 카메라 개인정보 설정에서 데스크톱 앱의 카메라 접근을 허용해야 합니다.
- WSL에서 이미 USB 패스스루를 사용 중이면 `./install_usb_webcam.sh --v4l2` 후 장치 경로를 지정해 실행할 수 있습니다. 이 경우 구독자는 기존 ROS 환경에서 실행합니다.
