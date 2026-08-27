cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME base_controller)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 실제 보드 IWDG reset HIL 전용. ON build에서만 SWD로 ControlTask stall을 주입할 수
# 있다. 기본 firmware에는 fault-injection 분기가 컴파일되지 않는다.
option(IWDG_STALL_TEST "Enable the SWD-triggered ControlTask stall hook" OFF)

# U3 양방향 smoke test / 보드레이트 스윕 전용. ON build에서만 SmokeTask와 u3_smoke.c가
# 들어간다. 기본 firmware는 USART1로 한 바이트도 내보내지 않는다.
option(U3_SMOKE_TEST "Enable the USART1 U3 smoke-test task" OFF)

# micro-ROS. production image의 기본값이 ON인 이유는 이것이 로봇의 주 통신 경로이기
# 때문이다. U3-Smoke preset만 OFF로 덮는다. 둘 다 USART1의 소유자를 자처하므로
# app_config.h의 _Static_assert가 동시 사용을 막는다.
option(MICRO_ROS "Link libmicroros.a and run MicroRosTask" ON)

set(MICRO_ROS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros/libmicroros)
if(MICRO_ROS AND NOT EXISTS ${MICRO_ROS_DIR}/libmicroros.a)
    # 산출물은 git에 없다(레시피만 있다). 조용히 링크 실패로 끌고 가지 않는다.
    message(FATAL_ERROR
        "libmicroros.a가 없다: ${MICRO_ROS_DIR}/libmicroros.a\n"
        "  ./micro_ros/build_libmicroros.sh 를 먼저 돌려라.\n"
        "  micro-ROS 없이 빌드하려면 -DMICRO_ROS=OFF.")
endif()

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # -- 애플리케이션 (CubeMX 재생성 영역 밖) --
    # HAL/RTOS 비의존 순수 모듈
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/line_builder.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/encoder_accumulator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/motor_output.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/motor_safety.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/protocol_ascii.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/rx_ring.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/control_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/speed_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/control_ack.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/health_policy.c
    # orchestration + STM32/RTOS adapter
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/app_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/platform_stm32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/platform_uart1.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/rtos_app.c
)

# micro-ROS 경로. 순수 모듈 셋(static_pool / base_kinematics / agent_link)은 host
# 단위 테스트가 있고, micro_ros_app.c만 rcl/rclc를 부른다.
if(MICRO_ROS)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/static_pool.c
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/base_kinematics.c
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/agent_link.c
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/micro_ros_app.c
    )
endif()

# U3 smoke test의 순수 모듈. 시험 build에서만 링크한다. production image에 죽은
# 코드를 남기지 않는다.
if(U3_SMOKE_TEST)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/u3_smoke.c
    )
endif()

# static-allocation 전용: FreeRTOS heap을 링크 대상에서 뺀다.
#
# 생성 파일 cmake/stm32cubemx/CMakeLists.txt의 FreeRTOS_Src에는 heap_4.c가 들어 있고
# 재생성 때마다 다시 들어온다. 그래서 그 파일을 고치는 대신 여기 사용자 영역에서 걸러
# 낸다. heap_4.c 자체가 configSUPPORT_DYNAMIC_ALLOCATION==0이면 #error로 빌드를
# 세우므로 이 필터가 빠지면 조용히 넘어가지 않고 즉시 드러난다.
get_target_property(FREERTOS_SOURCES FreeRTOS SOURCES)
list(FILTER FREERTOS_SOURCES EXCLUDE REGEX "portable/MemMang/heap_[0-9]+\\.c$")
set_target_properties(FreeRTOS PROPERTIES SOURCES "${FREERTOS_SOURCES}")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
)

# 애플리케이션 소스에만 강한 경고를 건다. HAL 드라이버는 이 옵션 아래에서 깨끗하지
# 않으므로 전역으로 걸지 않는다(툴체인 파일이 이미 -Wall을 전역으로 준다).
set(APP_WARNING_FLAGS -Wall -Wextra -Wshadow -Wformat=2 -Wundef)
set(APP_SOURCES
    App/Src/line_builder.c
    App/Src/encoder_accumulator.c
    App/Src/motor_output.c
    App/Src/motor_safety.c
    App/Src/protocol_ascii.c
    App/Src/rx_ring.c
    App/Src/control_core.c
    App/Src/control_ack.c
    App/Src/health_policy.c
    App/Src/app_main.c
    App/Src/platform_stm32.c
    App/Src/platform_uart1.c
    Core/Src/main.c
    Core/Src/stm32f4xx_it.c
)

if(U3_SMOKE_TEST)
    list(APPEND APP_SOURCES App/Src/u3_smoke.c)
endif()

# micro_ros_app.c는 이 목록에 넣지 않는다. libmicroros의 생성 헤더가 -Wundef와
# -Wformat=2 아래에서 깨끗하지 않고, 그건 우리가 고칠 코드가 아니다. 대신 아래에서
# 우리 코드에 대한 경고만 따로 건다.
if(MICRO_ROS)
    list(APPEND APP_SOURCES
        App/Src/static_pool.c
        App/Src/base_kinematics.c
        App/Src/agent_link.c
    )
    set_property(SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/micro_ros_app.c
                 APPEND PROPERTY COMPILE_OPTIONS -Wall -Wextra -Wshadow)
endif()

# rtos_app.c만 -Wundef를 뺀다. FreeRTOS 헤더가 정의되지 않은 config 매크로를
# #if로 검사하는 자리가 있어 application 코드의 문제가 아닌 경고가 뜬다.
set_property(SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/rtos_app.c
             APPEND PROPERTY COMPILE_OPTIONS -Wall -Wextra -Wshadow -Wformat=2)
foreach(app_source IN LISTS APP_SOURCES)
    set_property(SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/${app_source}
                 APPEND PROPERTY COMPILE_OPTIONS ${APP_WARNING_FLAGS})
endforeach()

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

if(IWDG_STALL_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE IWDG_STALL_TEST=1)
endif()

if(U3_SMOKE_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE U3_SMOKE_TEST=1)
endif()

if(MICRO_ROS)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE MICRO_ROS=1)
    # SYSTEM으로 넣어 생성 헤더의 경고가 우리 빌드 로그를 덮지 않게 한다.
    target_include_directories(${CMAKE_PROJECT_NAME} SYSTEM PRIVATE
        ${MICRO_ROS_DIR}/include
    )
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

if(MICRO_ROS)
    # 절대 경로로 건다. -L/-l 조합은 검색 경로에 있는 다른 libmicroros를 집어 갈 수
    # 있다. ABI가 다른 라이브러리를 링크하면 실패가 런타임까지 내려간다.
    target_link_libraries(${CMAKE_PROJECT_NAME} ${MICRO_ROS_DIR}/libmicroros.a)
endif()
