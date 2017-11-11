cmake_minimum_required(VERSION 2.8.3)
project(lidar)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        )
find_package(sb_utils REQUIRED)

# Export the Inverse Prespective Mapping Class (IPM) as a library
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

add_executable(LidarDecision
        src/lidar_node.cpp
        src/LidarNode.cpp
        src/LidarDecision.cpp
        src/LidarObstacleManager.cpp
        src/LinearAlgebra.cpp
        src/RvizUtils.cpp
        include/LidarNode.h
        include/LidarDecision.h
        include/LidarObstacleManager.h
        include/LinearAlgebra.h
        include/RvizUtils.h
        )


target_link_libraries(LidarDecision
        ${catkin_LIBRARIES}
        ${sb_utils_LIBRARIES}
        )


#if (CATKIN_ENABLE_TESTING)
#    # gTest
#    catkin_add_gtest(green-recognition-test test/green-recognition-test.cpp src/HSVFilter.cpp src/HSVFilterNode.cpp src/CircleDetection.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#    target_link_libraries(green-recognition-test ${catkin_LIBRARIES})
#endif()

