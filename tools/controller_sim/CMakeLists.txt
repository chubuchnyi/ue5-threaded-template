cmake_minimum_required(VERSION 3.16)
project(controller_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# shared/ holds the wire protocol header used by both this sim and the UE plugin.
set(SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../shared")

# The plant/watchdog/statistics live in a small static lib so both the sim and
# the generator stub can reuse the timing + socket helpers.
add_library(motion_common STATIC net_time.cpp)
target_include_directories(motion_common PUBLIC ${SHARED_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

# controller_sim: the real thing under test.
add_executable(controller_sim main.cpp)
target_link_libraries(controller_sim PRIVATE motion_common)

# motion_gen: throwaway stand-in for the UE worker. Sends a 1 kHz sine to 5005
# and prints RTT from the echo on 5006, so Stage 1 is testable without UE.
add_executable(motion_gen motion_gen.cpp)
target_link_libraries(motion_gen PRIVATE motion_common)

if(WIN32)
    # ws2_32 = Winsock, winmm = timeBeginPeriod for a 1 ms scheduler tick.
    target_link_libraries(motion_common PUBLIC ws2_32 winmm)
endif()

if(MSVC)
    target_compile_options(motion_common PUBLIC /W4)
else()
    target_compile_options(motion_common PUBLIC -Wall -Wextra)
endif()
