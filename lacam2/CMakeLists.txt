cmake_minimum_required(VERSION 3.16)
file(GLOB SRCS "./src/*.cpp")
project(lacam2)
add_library(${PROJECT_NAME} STATIC ${SRCS})
# target_compile_options(${PROJECT_NAME} PUBLIC -O3 -Wall)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${PROJECT_NAME} PUBLIC -g -O0)  # Debug mode: include debugging information
else()
    target_compile_options(${PROJECT_NAME} PUBLIC -O3 -Wall)  # Release mode: optimize and enable warnings
endif()

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE ./include)
