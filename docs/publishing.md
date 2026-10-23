# 비공개 GitHub 저장소 업로드

대상은 `ZenithFuturesLab/zav-simulator`, 공개 범위는 **private**입니다. 조직에 비공개 저장소를 만들 권한이 있는 GitHub 계정이 필요합니다. 저장소를 clone하는 사람도 조직 또는 저장소 접근 권한이 있어야 합니다.

[GitHub CLI](https://cli.github.com/) 설치 후 이 저장소 루트에서 실행합니다. 비밀번호나 token을 스크립트·채팅·Git 파일에 입력하지 않습니다.

```bash
gh auth login --hostname github.com --git-protocol https --web --scopes repo,read:org,workflow
./scripts/publish.sh
```

브라우저에서 일회용 코드를 입력하고 GitHub CLI와 조직 접근을 승인합니다. 조직 SSO/앱 제한이 있으면 관리자의 승인이 추가로 필요할 수 있습니다.

업로드 스크립트는 기본 소스 검사와 포함 파일 검사를 수행하고, Git 작성자 정보가 없으면 인증된 GitHub 계정의 noreply 주소를 사용합니다. 비공개 저장소를 생성하거나 동일한 비공개 저장소를 사용하고 `main`을 push합니다. 공개 저장소 또는 다른 origin이 있으면 자동 변경하지 않고 중단합니다. force push는 사용하지 않습니다.

포함: 소스·USD·캘리브레이션 YAML·실행기·고정 커밋 목록·설치/검증 문서·CI.

제외: Isaac 설치/venv, 외부 저장소 checkout 및 Git 이력, build/install/log, PX4 BSON·ULog, 개인 migration 자료·테스트 로그, 인증 정보.

새 컴퓨터에서 README대로 clone·bootstrap한 뒤 실행 검증을 완료하세요. GitHub CI는 GPU를 실행하지 않으며 Windows GPU 종단 동작을 보증하지 않습니다.
