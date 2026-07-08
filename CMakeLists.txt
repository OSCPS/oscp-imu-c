cmake_minimum_required(VERSION 3.14)
project(oscp-imu-c
        LANGUAGES C
        DESCRIPTION "OSCP IMU C library"
        VERSION 0.2.0)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(OSCP_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(OSCP_ENABLE_SANITIZERS  "Build with AddressSanitizer + UndefinedBehaviorSanitizer" OFF)

# --- Warning flags -----------------------------------------------------------
add_library(oscp_warnings INTERFACE)
if(MSVC)
    target_compile_options(oscp_warnings INTERFACE /W4)
    if(OSCP_WARNINGS_AS_ERRORS)
        target_compile_options(oscp_warnings INTERFACE /WX)
    endif()
else()
    target_compile_options(oscp_warnings INTERFACE
            -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
    if(OSCP_WARNINGS_AS_ERRORS)
        target_compile_options(oscp_warnings INTERFACE -Werror)
    endif()
endif()

# --- Sanitizers --------------------------------------------------------------
add_library(oscp_sanitizers INTERFACE)
if(OSCP_ENABLE_SANITIZERS AND NOT MSVC)
    target_compile_options(oscp_sanitizers INTERFACE
            -fsanitize=address,undefined -fno-sanitize-recover=all
            -fno-omit-frame-pointer -g)
    target_link_options(oscp_sanitizers INTERFACE
            -fsanitize=address,undefined -fno-sanitize-recover=all)
endif()

# --- COBS dependency ---------------------------------------------------------
add_library(cobs STATIC third-party/cobs-c/cobs.c)
target_include_directories(cobs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/third-party/cobs-c)

# --- OSCP IMU C Library ------------------------------------------------------
add_library(oscp-imu-c STATIC oscp_imu.c)
target_include_directories(oscp-imu-c PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(oscp-imu-c
        PUBLIC  cobs
        PRIVATE oscp_warnings
        PUBLIC  oscp_sanitizers)

# --- Testing Dependency ------------------------------------------------------
include(FetchContent)
FetchContent_Declare(unity
        GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
        GIT_TAG        v2.6.1
        GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(unity)

target_compile_definitions(unity PUBLIC UNITY_SUPPORT_64)

if(MSVC)
    target_compile_options(unity PRIVATE /wd5045)
endif()

# --- Testing -----------------------------------------------------------------
include(CTest)
enable_testing()

add_executable(test_oscp_imu_c test/test_oscp_imu.c)
target_link_libraries(test_oscp_imu_c PRIVATE oscp-imu-c unity oscp_warnings)
target_include_directories(test_oscp_imu_c PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(MSVC)
    target_compile_options(test_oscp_imu_c PRIVATE /wd4310)
endif()

add_test(NAME test_oscp_imu_c COMMAND test_oscp_imu_c)
