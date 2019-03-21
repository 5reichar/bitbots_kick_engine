cmake_minimum_required(VERSION 2.8.3)
project(bitbots_kick_engine)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

generate_messages(
    DEPENDENCIES
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories( cfg/cpp
    include
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(SOURCES
    src/bitbots_kick_engine/KickEngineBase.cpp
    src/bitbots_kick_engine/KickEngineNode.cpp
    src/bitbots_kick_engine/SplineKickEngine.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})

add_executable(KickEngineNode
    src/bitbots_kick_engine/KickEngineNode.cpp
)