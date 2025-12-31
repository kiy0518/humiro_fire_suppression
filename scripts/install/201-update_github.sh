#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$PROJECT_ROOT/setup_env.sh"
# GitHub 업데이트 스크립트
# 사용법: ./update_github.sh "커밋 메시지"

set -e

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "=========================================="
echo "GitHub 업데이트 스크립트"
echo "=========================================="
echo ""

# 커밋 메시지 확인
if [ -z "$1" ]; then
    echo -e "${YELLOW}⚠ 경고: 커밋 메시지가 제공되지 않았습니다.${NC}"
    echo "사용법: ./update_github.sh \"커밋 메시지\""
    echo ""
    read -p "커밋 메시지를 입력하세요: " COMMIT_MSG
    if [ -z "$COMMIT_MSG" ]; then
        echo -e "${RED}✗ 커밋 메시지가 없어 업데이트를 취소합니다.${NC}"
        exit 1
    fi
else
    COMMIT_MSG="$1"
fi

# 현재 디렉토리 확인
if [ ! -d ".git" ]; then
    echo -e "${RED}✗ Git 저장소가 아닙니다.${NC}"
    exit 1
fi

# 변경사항 확인
echo "1. 변경사항 확인 중..."
CHANGES=$(git status --short)
if [ -z "$CHANGES" ]; then
    echo -e "${YELLOW}⚠ 변경사항이 없습니다.${NC}"
    exit 0
fi

echo "변경된 파일:"
git status --short
echo ""

# 확인 요청
read -p "위 변경사항을 GitHub에 업로드하시겠습니까? (y/n): " CONFIRM
if [ "$CONFIRM" != "y" ] && [ "$CONFIRM" != "Y" ]; then
    echo -e "${YELLOW}업데이트가 취소되었습니다.${NC}"
    exit 0
fi

# 파일 추가
echo ""
echo "2. 파일 추가 중..."
git add .
echo -e "${GREEN}✓ 파일 추가 완료${NC}"

# 커밋
echo ""
echo "3. 커밋 생성 중..."
git commit -m "$COMMIT_MSG"
echo -e "${GREEN}✓ 커밋 완료: $COMMIT_MSG${NC}"

# 푸시
echo ""
echo "4. GitHub에 푸시 중..."
echo -e "${YELLOW}인증 정보가 필요합니다:${NC}"
echo "  Username: kiy0518"
echo "  Password: Personal Access Token 입력"
echo ""

# 푸시 시도
if git push; then
    echo ""
    echo -e "${GREEN}=========================================="
    echo "✓ GitHub 업데이트 완료!"
    echo "==========================================${NC}"
    echo ""
    echo "저장소 확인: https://github.com/kiy0518/humiro_fire_suppression"
else
    echo ""
    echo -e "${RED}✗ 푸시 실패${NC}"
    echo "토큰을 확인하거나 수동으로 푸시하세요:"
    echo "  git push"
    exit 1
fi

