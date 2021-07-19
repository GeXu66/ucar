cmake_minimum_required(VERSION 3.0.2)
project(navigation_pkg)
set(CMAKE_CXX_STANDARD 11)

#message_generation is used for customize msg
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	message_generation
	geometry_msgs
	tf
)

#customize message
add_message_files(FILES WayPoint.msg)
generate_messages(DEPENDENCIES geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
	include
	${catkin_INCLUDE_DIRS} #include the dir "include" in every pkg
)

#add_executable(node_name src/file_name.cpp)
#target_link_libraries(node_name ${catkin_LIBRARIES}) #${catkin_LIBRARIES} to deal with an error
#add_dependencies(node_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(real_time_tracker src/real_time_tracker.cpp include/navigation_pkg/AStar.cpp)
target_link_libraries(real_time_tracker ${catkin_LIBRARIES})
add_dependencies(real_time_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(keyboard_node src/keyboard.cpp)
target_link_libraries(keyboard_node ${catkin_LIBRARIES})
add_dependencies(keyboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_tracker_node src/odom_tracker.cpp)
target_link_libraries(odom_tracker_node ${catkin_LIBRARIES})
add_dependencies(odom_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



