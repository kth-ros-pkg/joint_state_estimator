cmake_minimum_required(VERSION 2.8.3)
project(dumbo_joint_state_estimator)

find_package(catkin REQUIRED COMPONENTS roscpp control_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS control_msgs
  INCLUDE_DIRS include
  LIBRARIES joint_state_estimator
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(joint_state_estimator src/JointStateEstimator.cpp)
target_link_libraries(joint_state_estimator ${catkin_LIBRARIES} orocos-bfl)

add_executable(joint_state_estimator_node src/JointStateEstimator.cpp src/joint_state_estimator_node.cpp)
target_link_libraries(joint_state_estimator_node ${catkin_LIBRARIES} orocos-bfl)

add_executable(test_joint_state_estimator src/test_joint_state_estimator.cpp)
target_link_libraries(test_joint_state_estimator ${catkin_LIBRARIES} orocos-bfl)


install(TARGETS joint_state_estimator joint_state_estimator_node test_joint_state_estimator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)