cmake_minimum_required(VERSION 2.8.0)
project( ara )

find_package(Qt4 REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-I /usr/include/eigen3/")

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

set( srcs main.cpp
  Arm.cpp
  ArmStates.cpp
  ArmSim.cpp
  Visualizer.cpp
  ArmControlPanel.cpp
  SearchWidget.cpp
  SearchControlPanel.cpp )

set( maze_srcs maze_main.cpp
  MazeStates.cpp
  MazeWidget.cpp )

set( headers ArmSim.h
  Visualizer.h
  ArmControlPanel.h
  SearchWidget.h
  SearchControlPanel.h )

set( maze_headers MazeWidget.h )

Qt4_wrap_cpp( headers_moc ${headers} )
Qt4_wrap_cpp( maze_headers_moc ${maze_headers} )

add_executable(arm_ara
  ${srcs}
  ${headers_moc} )

target_link_libraries(
  arm_ara
  ${QT_LIBRARIES}
)

add_executable(maze_ara
  ${maze_srcs}
  ${maze_headers_moc}
)

target_link_libraries(
  maze_ara
  ${QT_LIBRARIES}
)
