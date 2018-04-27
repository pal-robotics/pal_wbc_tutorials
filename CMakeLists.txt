cmake_minimum_required(VERSION 2.8.3)
project(pal_wbc_tutorial)

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  pal_wbc_controller
  pal_wbc_utils
  pluginlib
  joint_state_controller
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pal_wbc_controller pal_wbc_utils pluginlib joint_state_controller roscpp
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/rrbot_stacks.cpp
    src/joint_pos_limit_kinematic_task.cpp
    src/reference_kinematic_task.cpp
)

add_executable(push_reference_pos_task_dynamic_reconfigure src/push_reference_pos_task_dynamic_reconfigure.cpp)
target_link_libraries(push_reference_pos_task_dynamic_reconfigure ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_reference_pos_task_topic src/push_reference_pos_task_topic.cpp)
target_link_libraries(push_reference_pos_task_topic ${PROJECT_NAME} ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

install(TARGETS ${PROJECT_NAME} push_reference_pos_task_dynamic_reconfigure
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS ${PROJECT_NAME} push_reference_pos_task_topic
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(FILES pal_wbc_tutorial_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(pal_wbc_tutorial REQUIRED)
    add_rostest_gtest(pal_wbc_tutorial_ddynamic_reconfigure_test_one test/pal_wbc_tutorial_ddynamic_reconfigure_test_one.test test/pal_wbc_tutorial_ddynamic_reconfigure_test_one.cpp)
    target_link_libraries(pal_wbc_tutorial_ddynamic_reconfigure_test_one ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

    add_rostest_gtest(pal_wbc_tutorial_ddynamic_reconfigure_test_two test/pal_wbc_tutorial_ddynamic_reconfigure_test_two.test test/pal_wbc_tutorial_ddynamic_reconfigure_test_two.cpp)
    target_link_libraries(pal_wbc_tutorial_ddynamic_reconfigure_test_two ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

    add_rostest_gtest(pal_wbc_tutorial_pop_and_push_task test/pal_wbc_tutorial_pop_and_push_task.test test/pal_wbc_tutorial_pop_and_push_task.cpp)
    target_link_libraries(pal_wbc_tutorial_pop_and_push_task ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

    add_rostest_gtest(pal_wbc_tutorial_topic_test test/pal_wbc_tutorial_topic_test.test test/pal_wbc_tutorial_topic_test.cpp)
    target_link_libraries(pal_wbc_tutorial_topic_test ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)
