# 백업 정보

## projects 폴더 백업

**백업 일시**: $(date)

**백업 파일 위치**: `~/backups/projects_backup_*.tar.gz`

**백업 내용**:
- `~/projects/Cluster_Drone/` - 기존 클러스터 드론 프로젝트
- `~/projects/Thermal/` - 열화상 카메라 프로젝트

**백업 파일 크기**: 약 24MB (압축 후)

## 복원 방법

필요시 다음 명령어로 복원할 수 있습니다:

```bash
cd ~
tar -xzf backups/projects_backup_YYYYMMDD_HHMMSS.tar.gz
```

## 현재 프로젝트 구조

기존 `projects/` 폴더의 내용은 다음 위치로 이동되었습니다:

- `~/projects/Cluster_Drone/` → `~/humiro_fire_suppression/`
- `~/projects/Thermal/` → `~/humiro_fire_suppression/thermal/`

## 주의사항

- 백업 파일은 안전한 위치에 보관하세요
- 복원이 필요하지 않다면 백업 파일도 삭제할 수 있습니다
- 새 프로젝트 구조에서 정상 동작을 확인한 후 백업 파일을 삭제하는 것을 권장합니다
