cmake_minimum_required(VERSION 2.8.3)
project(tb3_rescue)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
  CATKIN_DEPENDS roscpp 
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  )

#add_executable(mapsaver src/Color_tracker.cpp)
#target_link_libraries(mapsaver ${catkin_LIBRARIES})




