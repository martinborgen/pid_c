cmake_minimum_required(VERSION 3.13)
project(pid_controller)

include_directories(
    ${PROJECT_SOURCE_DIR}/src
)

add_subdirectory(
    ${PROJECT_SOURCE_DIR}/test
)
