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
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/rtos_app.c
)

# static-allocation 전용: FreeRTOS heap을 링크 대상에서 뺀다.
#
# 생성 파일 cmake/stm32cubemx/CMakeLists.txt의 FreeRTOS_Src에는 heap_4.c가 들어
# 있고 재생성 때마다 다시 들어온다. 그래서 그 파일을 고치는 대신 여기(사용자 영역)
# 에서 걸러 낸다. heap_4.c 자체가 configSUPPORT_DYNAMIC_ALLOCATION==0이면 #error로
# 빌드를 세우므로, 이 필터가 빠지면 조용히 넘어가지 않고 즉시 드러난다.
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
    Core/Src/main.c
    Core/Src/stm32f4xx_it.c
)

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

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
