cmake_minimum_required(VERSION 2.8.3)
project(cil_slam)

# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_generation
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  FeatureLines.msg
  FeaturePoints.msg
  ImgInfo.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime 
  message_generation
  geometry_msgs 
  sensor_msgs
  std_msgs
  nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(feature_tracker
    src/feature_tracker_node.cc
    src/feature_tracker.cc
    )
target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(line_feature_tracker
    src/linefeature_tracker.cc
    src/linefeature_tracker_node.cc
    )
target_link_libraries(line_feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(visual_tracker
    src/visual_node.cc
    src/feature_tracker.cc
    src/linefeature_tracker.cc
    )
target_link_libraries(visual_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})
