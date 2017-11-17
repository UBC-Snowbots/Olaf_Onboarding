cmake_minimum_required(VERSION 2.8.3)
project(hole_tracker)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        )
find_package(sb_utils REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ipm
        CATKIN_DEPENDS roscpp
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${sb_utils_INCLUDE_DIRS}
        ./include
)

add_executable(hole_tracker_node
        src/hole_tracker_node.cpp
        src/HoleTrackerNode.cpp
        src/LidarDecision.cpp
        src/LidarObstacleManager.cpp
        src/LinearAlgebra.cpp
        src/RvizUtils.cpp
        include/HoleTrackerNode.h
        include/LidarDecision.h
        include/LidarObstacleManager.h
        include/LinearAlgebra.h
        include/RvizUtils.h
        )


target_link_libraries(hole_tracker_node
        ${catkin_LIBRARIES}
        ${sb_utils_LIBRARIES}
        )


if (CATKIN_ENABLE_TESTING)
    # gTest
    catkin_add_gtest(linear-algebra-test test/linear-algebra-test.cpp src/LinearAlgebra.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_link_libraries(linear-algebra-test ${catkin_LIBRARIES})

    catkin_add_gtest(lidar-obstacle-manager-test test/lidar-obstacle-manager-test.cpp src/LidarObstacleManager.cpp src/LinearAlgebra.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_link_libraries(lidar-obstacle-manager-test ${catkin_LIBRARIES})


    catkin_add_gtest(lidar-decision-test test/lidar-decision-test.cpp src/LidarDecision.cpp src/LinearAlgebra.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_link_libraries(lidar-decision-test ${catkin_LIBRARIES})
endif ()

