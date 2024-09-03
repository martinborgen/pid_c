cmake_minimum_required(VERSION 3.13)

include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(
    mytest 
    ${PROJECT_SOURCE_DIR}/test/mytest.c
    ${PROJECT_SOURCE_DIR}/test/acutest.h
    ${PROJECT_SOURCE_DIR}/src/pid_controller.h
    ${PROJECT_SOURCE_DIR}/src/pid_controller.c
    )