cmake_minimum_required(VERSION 2.8.3)
project(pacman)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(Qt5Widgets_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5Widgets")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  message_generation
  Qt5Widgets
  Qt5Gui
  Qt5Core
)

add_message_files(FILES Num.msg)

generate_messages(DEPENDENCIES std_msgs)

#openGL
find_package(OpenGL REQUIRED)
list(APPEND libs ${OPENGL_LIBRARIES})

catkin_package(
   CATKIN_DEPENDS roscpp rospy roslib message_runtime
   DEPENDS Qt5Widgets Qt5Gui Qt5Core OpenGL
)

include_directories(include
 ${catkin_INCLUDE_DIRS}
 ${OPENGL_INCLUDE_DIRS}
)

set(pacman_SRCS
  src/main.cpp
  src/glwidget.cpp
  src/window.cpp
  src/listenmsgthread.cpp
  src/maps.cpp
)
set(pacman_HDRS
  include/pacman/glwidget.h
  include/pacman/window.h
  include/pacman/maps.h
  include/pacman/listenmsgthread.h
)

qt5_wrap_cpp(pacman_MOCS ${pacman_HDRS})

qt5_add_resources(RC_SRCS 
    resources/resources.qrc
    ${SHARED_RC}
)

add_executable(pacman_world ${pacman_SRCS} ${pacman_MOCS} ${RC_SRCS})
target_link_libraries(pacman_world Qt5::Widgets ${catkin_LIBRARIES} ${OPENGL_LIBRARIES})
add_dependencies(pacman_world pacman_generate_messages_cpp)

add_executable(pacman_controller src/pacman_controller.cpp)
target_link_libraries(pacman_controller ${catkin_LIBRARIES})
add_dependencies(pacman_controller pacman_generate_messages_cpp)
