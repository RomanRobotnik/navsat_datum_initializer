cmake_minimum_required(VERSION 2.8.3)
project(navsat_datum_initializer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  rcomponent
  message_generation
  actionlib_msgs
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    InitializerCommand.msg
    InitializerFeedback.msg
    InitializerResult.msg
)

## Generate services in the 'srv' folder
##add_service_files(
##    DIRECTORY srv
##    FILES
##    SetCommand.srv
##)

## Generate actions in the 'action' folder
add_action_files(
    FILES
    DatumInitializer.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    std_srvs
    rcomponent
  DEPENDS
)

###########
## Build ##
###########

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/${PROJECT_NAME}/datum_initializer.py
  src/${PROJECT_NAME}/datum_initializer_node.py
  src/${PROJECT_NAME}/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Modify this if launch or config folders are deleted
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
