# GitHub 저장소 설정 가이드

## 워크스페이스 처리 방법

### 현재 설정 (권장)

워크스페이스는 다음과 같이 처리됩니다:

1. **소스 코드 포함**: `workspaces/*/src/` 디렉토리는 Git에 포함
2. **빌드 결과물 제외**: `build/`, `install/`, `log/` 디렉토리는 `.gitignore`로 제외

### 저장소 크기 확인

```bash
# 워크스페이스 소스 크기 확인
cd ~/humiro_fire_suppression
du -sh workspaces/*/src 2>/dev/null

# 전체 프로젝트 크기 (빌드 결과물 제외)
du -sh --exclude=build --exclude=install --exclude=log .
```

### 저장소가 너무 큰 경우

#### 옵션 1: Git Submodule 사용

워크스페이스 소스를 submodule로 관리:

```bash
# .gitmodules 파일 생성
cat > .gitmodules << 'EOF'
[submodule "workspaces/micro_ros_ws/src/micro-ROS-Agent"]
    path = workspaces/micro_ros_ws/src/micro-ROS-Agent
    url = https://github.com/micro-ROS/micro-ROS-Agent.git
    branch = humble

[submodule "workspaces/px4_ros2_ws/src/px4_msgs"]
    path = workspaces/px4_ros2_ws/src/px4_msgs
    url = https://github.com/PX4/px4_msgs.git
    branch = ros2
EOF

# .gitignore에 소스 디렉토리 추가 (submodule 사용 시)
echo "workspaces/micro_ros_ws/src/" >> .gitignore
echo "workspaces/px4_ros2_ws/src/" >> .gitignore
```

#### 옵션 2: 설치 스크립트로 자동 다운로드

워크스페이스 소스를 Git에서 제외하고, 설치 스크립트가 자동으로 다운로드:

```bash
# .gitignore에 추가
echo "workspaces/micro_ros_ws/src/" >> .gitignore
echo "workspaces/px4_ros2_ws/src/" >> .gitignore
echo "workspaces/mavlink-router/" >> .gitignore  # 전체가 크면
```

그리고 설치 스크립트에서 자동으로 클론하도록 수정.

## GitHub 저장소 초기화

### 1. 저장소 생성 전 확인

```bash
cd ~/humiro_fire_suppression

# .gitignore 확인
cat .gitignore

# 제외될 파일 확인
git check-ignore -v workspaces/*/build workspaces/*/install

# 포함될 파일 확인
git ls-files workspaces/ 2>/dev/null || echo "Git 저장소 아직 초기화 안됨"
```

### 2. Git 저장소 초기화

```bash
cd ~/humiro_fire_suppression

# Git 초기화
git init

# .gitignore 확인
git check-ignore -v workspaces/*/build

# 첫 커밋
git add .
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

## 저장소 크기 최적화 팁

### 1. Git LFS 사용 (큰 바이너리 파일)

```bash
# Git LFS 설치
git lfs install

# 큰 파일 타입 추적
git lfs track "*.so"
git lfs track "*.bin"
git lfs track "workspaces/**/*.so"
```

### 2. 저장소 정리

```bash
# .git 폴더 크기 확인
du -sh .git

# 불필요한 파일 제거
git gc --aggressive --prune=now
```

### 3. 부분 클론 (클론 시)

```bash
# 소스만 클론 (빌드 결과물 제외)
git clone --filter=blob:none <repo-url>
```

## 권장 워크플로우

1. **개발 환경**: 워크스페이스 소스 포함 (편의성)
2. **GitHub 저장소**: 
   - 소스 포함 (용량 허용 시)
   - 또는 submodule/자동 다운로드 (용량 문제 시)
3. **CI/CD**: GitHub Actions로 빌드 테스트

## 체크리스트

- [ ] `.gitignore`에 빌드 결과물 제외 확인
- [ ] 워크스페이스 소스 크기 확인
- [ ] 저장소 크기가 100MB 이하인지 확인 (권장)
- [ ] 설치 스크립트가 워크스페이스 자동 빌드하는지 확인
- [ ] README에 빌드 방법 명시
- [ ] `.github/workflows/`에 빌드 테스트 추가 (선택)
