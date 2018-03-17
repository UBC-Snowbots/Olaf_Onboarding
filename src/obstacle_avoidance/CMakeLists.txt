cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoidance)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs
)
find_package(sb_utils REQUIRED)

add_definitions(-std=c++14)

catkin_package(
    INCLUDE_DIRS include
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${sb_utils_INCLUDE_DIRS}
    ./include
)

## Declare a C++ executable
# add_executable(sample_package_node src/sample_package_node.cpp)
add_executable(cone_avoider src/cone_avoider.cpp src/ConeAvoider.cpp include/ConeAvoider.h)

## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(cone_avoider ${catkin_LIBRARIES} ${sb_utils_LIBRARIES})


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    # Adding rostest to the package
    find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
    add_rostest_gtest(cone_avoider_rostest test/obstacle_avoidance_test.test test/cone_avoider_rostest.cpp)
    target_link_libraries(cone_avoider_rostest ${catkin_LIBRARIES})


endif()