#!/usr/bin/env bash
#
# run_direct.sh — colcon 빌드 없이 바로 실행 (터미널 2개 필요)
#
#   터미널 1 : ./run_direct.sh node
#   터미널 2 : ./run_direct.sh rviz
#
# 패키지를 빌드해 런치로 띄우려면 README 의 '방법 A' 를 따르세요.

set -u
cd "$(dirname "$0")" || exit 1

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ROS 2 환경이 source 되지 않았습니다."
    echo "  source /opt/ros/<배포판>/setup.bash  를 먼저 실행하세요."
    exit 1
fi

case "${1:-}" in
  node)
    echo "마커 퍼블리셔를 시작합니다. (Ctrl+C 로 종료 → 0.2초 뒤 마커 소멸)"
    exec python3 rviz_visualization/custom_marker_publisher.py
    ;;
  rviz)
    echo "RViz2 를 설정 파일과 함께 엽니다."
    exec rviz2 -d config/integrated_view.rviz
    ;;
  echo)
    exec ros2 topic echo /visual_markers --once
    ;;
  *)
    echo "사용법: $0 {node|rviz|echo}"
    exit 1
    ;;
esac
