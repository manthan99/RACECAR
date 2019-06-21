cmake_minimum_required(VERSION 2.8.3)
project(teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

#add_message_files(
# FILES
#) 

 find_package(Boost REQUIRED COMPONENTS system)

add_definitions(-std=c++0x)


 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES teleop
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  #DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(manual src/manual.cpp)
target_link_libraries(manual ${catkin_LIBRARIES})
add_dependencies(manual teleop_generate_messages_cpp)


## Specify libraries to link a library or executable target against
