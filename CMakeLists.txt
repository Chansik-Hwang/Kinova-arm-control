CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(EE3100704-projects)

set(RAISIM_PATH "$ENV{HOME}/Library/raisimLib/raisim/linux")
include_directories(${RAISIM_PATH}/include)
link_directories(${RAISIM_PATH}/lib)

add_executable(kinova examples/src/kinova.cpp)