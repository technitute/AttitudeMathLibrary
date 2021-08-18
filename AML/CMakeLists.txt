cmake_minimum_required(VERSION 3.5)
project(AML)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

set(SRC_CPP_AML
    AMLVector3.cpp
    AMLMatrix33.cpp
    AMLDCM.cpp
    AMLEulerAngles.cpp
    AMLQuaternion.cpp
)

add_library(
    ${PROJECT_NAME} STATIC
    ${SRC_CPP_AML}
)

target_include_directories(
    ${PROJECT_NAME} PUBLIC 
    ${PROJECT_SOURCE_DIR}
)
