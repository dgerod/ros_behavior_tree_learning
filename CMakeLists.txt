cmake_minimum_required(VERSION 2.8.3)
project(ros_behavior_tree_learning)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy
                 message_runtime
                 std_msgs
                 tasks_toolkit
                 behavior_tree_learning
                 ros_behavior_tree_learning_comms
)
