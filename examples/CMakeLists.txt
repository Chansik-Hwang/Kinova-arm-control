cmake_minimum_required(VERSION 3.5)
project(EE3100704-examples LANGUAGES CXX)

#add_library(${PROJECT_NAME}
#        src/kinova.cpp
#)

add_executable(kinova src/kinova.cpp
        ../tools/src/Planning/cubicTrajectoryGenerator.cpp
        ../tools/src/Control/robotController.cpp
        ../tools/src/Others/setTime.cpp
        ../tools/src/Others/setObstacle.cpp
        ../tools/src/Control/Control.cpp
)
target_link_libraries(kinova
        raisim
        pthread
)


add_executable(canine src/canine.cpp
        ../tools/src/Planning/cubicTrajectoryGenerator.cpp
        ../tools/src/Control/robotController.cpp
        ../tools/src/Others/setTime.cpp
        ../tools/src/Others/setObstacle.cpp
)
target_link_libraries(canine
        raisim
        pthread
)

add_executable(robot src/robot.cpp
        ../tools/src/Planning/cubicTrajectoryGenerator.cpp
        ../tools/src/Control/robotController.cpp
        ../tools/src/Others/setTime.cpp
        ../tools/src/Others/setObstacle.cpp
)
target_link_libraries(robot
        raisim
        pthread
)
