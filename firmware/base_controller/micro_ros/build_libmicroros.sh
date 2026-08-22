#!/usr/bin/env bash
# libmicroros.a를 뽑는다. host에서 부르는 진입점이다.
#
#   ./micro_ros/build_libmicroros.sh          # 없으면 만든다
#   ./micro_ros/build_libmicroros.sh --clean  # 지우고 다시 만든다
#
# 산출물 micro_ros/libmicroros/ 는 .gitignore 대상이다. **레시피는 git에 있고
# 산출물은 없다** — 30 MB 짜리 .a를 저장소에 넣는 대신 이 스크립트를 다시 돌린다.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$HERE")"
LIB_FOLDER="$(basename "$HERE")"
IMAGE="microros/micro_ros_static_library_builder:jazzy"

if [ "${1:-}" = "--clean" ]; then
    rm -rf "$HERE/libmicroros"
fi

# ABI flag 대조. 애플리케이션과 라이브러리의 -mcpu/-mfpu/-mfloat-abi가 갈리면
# 링크는 통과하고 런타임에 깨진다 — 그 실패는 디버깅하기 아주 나쁘다.
app_abi=$(grep -o '\-mcpu=[^ "]*\|\-mfpu=[^ "]*\|\-mfloat-abi=[^ "]*' \
          "$PROJECT_DIR/cmake/gcc-arm-none-eabi.cmake" | sort | tr '\n' ' ')
lib_abi=$(grep -o '\-mcpu=[^ "]*\|\-mfpu=[^ "]*\|\-mfloat-abi=[^ "]*' \
          "$HERE/library_generation/toolchain.cmake" | sort | tr '\n' ' ')
if [ "$app_abi" != "$lib_abi" ]; then
    echo "ABI flag 불일치 — libmicroros.a를 만들지 않는다." >&2
    echo "  애플리케이션: $app_abi" >&2
    echo "  라이브러리  : $lib_abi" >&2
    exit 1
fi
echo "ABI 대조 통과: $app_abi"

# 사용자 정의 메시지를 **한 곳에서만** 관리한다. 원본은 software/ros_pkgs/mr_msgs이고
# (호스트 노드가 그것을 빌드한다) 여기로는 빌드 직전에 복사만 한다. 두 벌을 각각
# 손으로 고치면 언젠가 갈리고, 그때 증상은 "agent가 디코드하지 못한다"로 나타나서
# 원인을 펌웨어에서 찾게 된다.
HOST_MSGS="$(cd "$PROJECT_DIR/../.." && pwd)/software/ros_pkgs/mr_msgs"
EXTRA="$HERE/library_generation/extra_packages"
rm -rf "$EXTRA/mr_msgs"
if [ -d "$HOST_MSGS" ]; then
    mkdir -p "$EXTRA"
    cp -R "$HOST_MSGS" "$EXTRA/mr_msgs"
    echo "extra_packages <- $HOST_MSGS"
else
    echo "mr_msgs 원본이 없다: $HOST_MSGS — 표준 타입만 빌드한다."
fi

docker run --rm \
    -v "$PROJECT_DIR":/project \
    -e MICROROS_LIBRARY_FOLDER="$LIB_FOLDER" \
    -e HOST_UID="$(id -u)" \
    -e HOST_GID="$(id -g)" \
    "$IMAGE"
