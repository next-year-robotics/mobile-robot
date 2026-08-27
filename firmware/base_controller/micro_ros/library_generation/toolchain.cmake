# libmicroros.a cross-build toolchain.
#
# 이 파일의 flag는 cmake/gcc-arm-none-eabi.cmake와 ABI가 같아야 한다. -mcpu / -mfpu /
# -mfloat-abi 세 가지가 어긋나면 링크는 통과하고 런타임에 깨진다. 최적화 수준은 달라도
# 된다. 애플리케이션은 -O0(Debug)이고 여기는 -O2다. micro-ROS를 -O0으로 빌드하면
# FLASH가 수백 KB 늘어난다. 이 라이브러리는 우리가 디버깅할 코드가 아니다.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER $ENV{TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER $ENV{TOOLCHAIN_PREFIX}g++)
set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")

# NUCLEO-F446RE: Cortex-M4F, single-precision FPU, hard float ABI.
set(TARGET_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# --gc-sections가 안 쓰는 심볼을 걷어 갈 수 있게 함수/데이터를 쪼갠다. 이게 없으면
# rcl/rclc 전체가 image로 끌려 들어온다.
set(SIZE_FLAGS "-O2 -ffunction-sections -fdata-sections -fno-common")

# micro-ROS가 POSIX를 가정하고 참조하는 두 가지를 막는다.
#   CLOCK_MONOTONIC : newlib에 없다. rcutils가 clock_gettime을 부르는 자리에서 쓴다.
#   __attribute__   : rosidl 생성 코드의 visibility attribute가 bare-metal에서 무의미하다.
set(MICROROS_FLAGS "-DCLOCK_MONOTONIC=0 -D'__attribute__(x)='")

set(CMAKE_C_FLAGS_INIT   "-std=c11 ${TARGET_FLAGS} ${SIZE_FLAGS} ${MICROROS_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 -fno-rtti -fno-exceptions ${TARGET_FLAGS} ${SIZE_FLAGS} ${MICROROS_FLAGS}" CACHE STRING "" FORCE)

set(__BIG_ENDIAN__ 0)
