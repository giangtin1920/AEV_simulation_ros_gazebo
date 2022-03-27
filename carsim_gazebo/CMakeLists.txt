cmake_minimum_required(VERSION 2.8.3)
project(carsim_gazebo)
set(CMAKE_BUILD_TYPE Debug)

#find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  laser_geometry
  message_generation
)
#find_package(PCL 1.6 REQUIRED COMPONENTS common apps )

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ttcRadar_msg.msg
 )
# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIRECTORIES})




#MESSAGE("######################################################################PCL#####################################################################")
#MESSAGE("PCL_FOUND:" ${PCL_FOUND})
#MESSAGE("PCL_INCLUDE_DIRS:" ${PCL_LIBRARY_DIRS})
#MESSAGE("PCL_LIBRARIES:" ${PCL_LIBRARIES})
#MESSAGE("PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
#MESSAGE("PCL_VERSION:" ${PCL_VERSION})
#MESSAGE("PCL_COMPONENTS:" ${PCL_COMPONENTS})
#MESSAGE("PCL_DEFINITIONS:" ${PCL_DEFINITIONS})


#MESSAGE("######################################################################catkin#####################################################################")
#MESSAGE("catkin_LIBRARIES: " ${catkin_LIBRARIES})



#add_executable(laser_assembler src/laser_assembler.cpp )
#target_link_libraries(laser_assembler  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  )
