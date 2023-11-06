cmake_minimum_required(VERSION 3.5)
project(EE3100704-examples LANGUAGES CXX)

#add_library(${PROJECT_NAME}
#        src/kinova.cpp
#)

add_executable(kinova src/kinova.cpp
        ../tools/src/cubicTrajectoryGenerator.cpp)
target_link_libraries(kinova
        raisim
        pthread
)


add_executable(canine src/canine.cpp
        ../tools/src/cubicTrajectoryGenerator.cpp)
target_link_libraries(canine
        raisim
        pthread
)

