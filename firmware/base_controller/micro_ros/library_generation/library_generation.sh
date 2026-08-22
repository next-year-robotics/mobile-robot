#!/bin/bash
# 컨테이너 **안에서** 도는 부분. host에서는 ../build_libmicroros.sh를 부른다.
#
# 상위 흐름(micro_ros_stm32cubemx_utils의 library_generation.sh)과 다른 점 세 가지:
#
#  1. **CFLAGS를 .mk에서 추출하지 않는다.** 그 방식은 STM32CubeIDE의 Makefile
#     프로젝트를 전제한다. 이 프로젝트는 CMake + Ninja라 .mk가 없다. 대신
#     library_generation/toolchain.cmake에 flag를 직접 적고, host 스크립트가
#     애플리케이션 toolchain 파일과 ABI flag를 대조한다.
#  2. **tf2_msgs와 control_msgs를 받지 않는다.** base_contract.md의 세 토픽이
#     쓰는 타입은 sensor_msgs / geometry_msgs / std_msgs / builtin_interfaces뿐이고,
#     이 넷은 generate_lib 기본 mcu_ws에 이미 들어 있다.
#  3. 산출물 소유권을 host 사용자에게 되돌린다(chmod 777이 아니라 chown).
set -e

export BASE_PATH=/project/$MICROROS_LIBRARY_FOLDER

if [ -f "$BASE_PATH/libmicroros/libmicroros.a" ]; then
    echo "libmicroros.a가 이미 있다. 다시 뽑으려면 $MICROROS_LIBRARY_FOLDER/libmicroros/ 를 지워라."
    exit 0
fi

######## cross compiler ########
if ! command -v arm-none-eabi-gcc > /dev/null; then
    apt-get update
    apt-get install -y gcc-arm-none-eabi
fi
arm-none-eabi-gcc --version | head -1

######## mcu_ws ########
cd /uros_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

######## 사용자 정의 메시지 (M5.md 8.3 — mr_msgs) ########
# extra_packages/ 가 비어 있으면 아무 일도 하지 않는다. 1차(표준 타입만) 단계는
# 이 경로를 타지 않는다.
if [ -n "$(ls -A $BASE_PATH/library_generation/extra_packages 2>/dev/null)" ]; then
    mkdir -p firmware/mcu_ws/extra_packages
    cp -R $BASE_PATH/library_generation/extra_packages/* firmware/mcu_ws/extra_packages/
    echo "extra_packages:"
    ls firmware/mcu_ws/extra_packages
fi

######## colcon.meta 반영 확인 (사전) ########
# "names" 아래에 객체가 아닌 값을 두면 colcon이 매핑 전체를 조용히 버린다.
# 그러면 transport가 기본값 udp로 돌아가 rmw_microxrcedds가 깨진다 — 2026-08-22 실측.
python3 - "$BASE_PATH/library_generation/colcon.meta" <<'PYEOF'
import json, sys
meta = json.load(open(sys.argv[1]))
names = meta.get("names", {})
bad = [k for k, v in names.items() if not isinstance(v, dict)]
if bad:
    sys.exit("colcon.meta: \"names\" 아래 값은 객체여야 한다. 위반: %s" % bad)
print("colcon.meta 검사 통과: %s" % ", ".join(sorted(names)))
PYEOF

######## build ########
export TOOLCHAIN_PREFIX=/usr/bin/arm-none-eabi-
ros2 run micro_ros_setup build_firmware.sh \
    $BASE_PATH/library_generation/toolchain.cmake \
    $BASE_PATH/library_generation/colcon.meta

######## 산출물 수집 ########
# 생성 include 트리에 .c가 섞여 있다. 애플리케이션 빌드가 그것을 컴파일하지 않도록 지운다.
find firmware/build/include/ -name "*.c" -delete
rm -rf $BASE_PATH/libmicroros
mkdir -p $BASE_PATH/libmicroros/include
cp -R firmware/build/include/* $BASE_PATH/libmicroros/include/
cp firmware/build/libmicroros.a $BASE_PATH/libmicroros/libmicroros.a

# 생성 트리는 include/<pkg>/<pkg>/... 로 한 겹 더 들어가 있다. #include <pkg/msg/x.h>가
# 그대로 되도록 한 겹 걷어 낸다.
pushd firmware/mcu_ws > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list | awk '{print $1}')
popd > /dev/null

for pkg in ${INCLUDE_ROS2_PACKAGES}; do
    if [ -d "$BASE_PATH/libmicroros/include/${pkg}/${pkg}" ]; then
        rsync -r $BASE_PATH/libmicroros/include/${pkg}/${pkg}/* $BASE_PATH/libmicroros/include/${pkg}
        rm -rf $BASE_PATH/libmicroros/include/${pkg}/${pkg}
    fi
done

######## 재현 기록 ########
# "무엇으로 빌드했는가"를 산출물 옆에 남긴다. 이 파일이 없으면 몇 주 뒤에 같은
# libmicroros.a를 다시 만들 수 없다.
find firmware/mcu_ws \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) \
    | awk -F"/" '{print $(NF-2)"/"$NF}' | sort > $BASE_PATH/libmicroros/available_ros2_types

: > $BASE_PATH/libmicroros/built_packages
pushd firmware > /dev/null
    for f in $(find $(pwd) -name .git -type d); do
        pushd $f > /dev/null
        echo "$(git config --get remote.origin.url) $(git rev-parse HEAD)" \
            >> $BASE_PATH/libmicroros/built_packages
        popd > /dev/null
    done
popd > /dev/null
sort -o $BASE_PATH/libmicroros/built_packages $BASE_PATH/libmicroros/built_packages

# 실제로 적용된 설정값. colcon.meta에 적은 것이 정말 반영됐는지 여기서 확인한다.
cp firmware/mcu_ws/install/rmw_microxrcedds_c/include/rmw_microxrcedds_c/rmw_microxrcedds_c/config.h \
   $BASE_PATH/libmicroros/rmw_microxrcedds_config.h 2>/dev/null || \
cp $(find firmware/mcu_ws -path "*rmw_microxrcedds_c/config.h" | head -1) \
   $BASE_PATH/libmicroros/rmw_microxrcedds_config.h || true

######## colcon.meta 반영 확인 (사후) ########
# 적은 대로 정말 반영됐는지 산출물에서 되읽는다. 여기서 걸리지 않으면 문제가
# 런타임까지 내려간다.
grep -q "define RMW_UXRCE_TRANSPORT_CUSTOM" $BASE_PATH/libmicroros/rmw_microxrcedds_config.h
grep -q "undef RMW_UXRCE_ALLOW_DYNAMIC_ALLOCATIONS" $BASE_PATH/libmicroros/rmw_microxrcedds_config.h
grep -q "define RMW_UXRCE_MAX_PUBLISHERS 2" $BASE_PATH/libmicroros/rmw_microxrcedds_config.h
grep -q "define RMW_UXRCE_MAX_SUBSCRIPTIONS 1" $BASE_PATH/libmicroros/rmw_microxrcedds_config.h
grep -q "define RMW_UXRCE_MAX_HISTORY 1" $BASE_PATH/libmicroros/rmw_microxrcedds_config.h
echo "rmw 설정 반영 확인 통과"

######## 소유권 ########
if [ -n "$HOST_UID" ] && [ -n "$HOST_GID" ]; then
    chown -R $HOST_UID:$HOST_GID $BASE_PATH/libmicroros
fi

echo "=== libmicroros.a 완료 ==="
ls -l $BASE_PATH/libmicroros/libmicroros.a
