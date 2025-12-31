# GitHub 업로드 가이드

## 워크스페이스 처리 방법

### 현재 설정 (권장)

워크스페이스 소스 크기 확인 결과:
- `workspaces/micro_ros_ws/src/`: 844K (61개 파일)
- `workspaces/px4_ros2_ws/src/`: 2.0M (274개 파일)
- `workspaces/mavlink-router/src/`: 324K (40개 파일)
- **총합: 약 3.2MB** ✅ GitHub 업로드 가능

### 포함되는 것 vs 제외되는 것

#### ✅ GitHub에 포함됨
- 모든 소스 코드 (`workspaces/*/src/`)
- 스크립트, 설정 파일, 문서
- 프로젝트 구조 전체

#### ❌ GitHub에 제외됨 (`.gitignore`)
- `workspaces/*/build/` - 빌드 결과물 (수 GB)
- `workspaces/*/install/` - 설치 결과물 (수 GB)
- `workspaces/*/log/` - 빌드 로그

## GitHub 저장소 초기화

### 1. 저장소 크기 확인

```bash
cd ~/humiro_fire_suppression

# 포함될 파일 크기 (빌드 결과물 제외)
du -sh --exclude=build --exclude=install --exclude=log .

# 워크스페이스 소스만
du -sh workspaces/*/src
```

### 2. Git 초기화

```bash
cd ~/humiro_fire_suppression

# Git 초기화
git init

# .gitignore 확인
git check-ignore -v workspaces/*/build

# 첫 커밋
git add .
git status  # 제외된 파일 확인

git commit -m "Initial commit: Project restructure"
```

### 3. GitHub에 푸시

```bash
# 원격 저장소 추가
git remote add origin <your-github-repo-url>

# 푸시
git branch -M main
git push -u origin main
```

## 클론 후 워크스페이스 빌드

저장소를 클론한 후 워크스페이스를 빌드해야 합니다:

```bash
# 저장소 클론
git clone <repo-url> ~/humiro_fire_suppression
cd ~/humiro_fire_suppression

# 환경 변수 설정
source setup_env.sh

# 자동 설치 (권장)
sudo ./scripts/install/000-install_all.sh

# 또는 수동 빌드
cd workspaces/micro_ros_ws && colcon build
cd ../px4_ros2_ws && colcon build
```

## 저장소 크기 최적화 (선택적)

만약 나중에 저장소가 너무 커지면:

### 옵션 1: Git Submodule 사용

워크스페이스 소스를 별도 저장소로 분리:

```bash
# .gitmodules 생성
cat > .gitmodules << 'EOF'
[submodule "workspaces/micro_ros_ws/src/micro-ROS-Agent"]
    path = workspaces/micro_ros_ws/src/micro-ROS-Agent
    url = https://github.com/micro-ROS/micro-ROS-Agent.git
EOF

# .gitignore에 추가
echo "workspaces/micro_ros_ws/src/" >> .gitignore
```

### 옵션 2: 설치 스크립트로 자동 다운로드

워크스페이스 소스를 Git에서 제외하고 설치 시 자동 다운로드:

```bash
# .gitignore에 추가
echo "workspaces/micro_ros_ws/src/" >> .gitignore
echo "workspaces/px4_ros2_ws/src/" >> .gitignore
```

그리고 설치 스크립트에서 자동 클론하도록 수정.

## 체크리스트

- [x] `.gitignore`에 빌드 결과물 제외 확인
- [x] 워크스페이스 소스 크기 확인 (3.2MB - 허용 가능)
- [x] README에 빌드 방법 명시
- [x] 설치 스크립트가 워크스페이스 자동 빌드 확인
- [ ] Git 저장소 초기화 및 첫 커밋
- [ ] GitHub 저장소 생성 및 푸시
