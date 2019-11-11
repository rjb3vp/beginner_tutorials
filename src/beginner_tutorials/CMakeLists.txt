cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)




## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs genmsg message_generation tf rostest rosbag)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES SetRandomRange.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs std_srvs)


## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime rosbag)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#catkin_add_gtest(talkerTest test/talker_test.cpp)# src/listener.cpp src/talker.cpp)

add_rostest_gtest(talker_test test/talker_test.launch test/talker_test.cpp)# src/listener.cpp src/talker.cpp)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
