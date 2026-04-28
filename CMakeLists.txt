cmake_minimum_required(VERSION 3.5)

include(cross.cmake)
set(OpenCV_DIR "/opt/ls_2k0300_env/opencv_4_10_build/lib/cmake/opencv4")

project(zhinengche
    VERSION 1.0.0
    LANGUAGES C CXX
    DESCRIPTION "Smart Car with Lane Detection and Zebra Crossing"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_compile_options(
    "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-O3;-pthread>"
    "$<$<COMPILE_LANGUAGE:C>:-Wall;-O3>"
)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Include Directories: ${OpenCV_INCLUDE_DIRS}")

add_library(common_lib STATIC)
target_include_directories(common_lib
    PUBLIC
        lib
        ${OpenCV_INCLUDE_DIRS}
)

file(GLOB_RECURSE COMMON_SOURCES "src/*.cpp" "src/*.c")
target_sources(common_lib PRIVATE ${COMMON_SOURCES})

add_subdirectory(main)
