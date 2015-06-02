cmake_minimum_required(VERSION 2.8.0)
project( ara )

find_package(Qt4 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-pthread -I /usr/include/eigen3/")

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
include_directories(../lcmtypes)

set( srcs main.cpp
  Arm.cpp
  ArmStates.cpp
  ArmSim.cpp
  Visualizer.cpp
  ArmControlPanel.cpp
  SearchWidget.cpp
  SearchControlPanel.cpp )

set( test_srcs tests.cpp
  Arm.cpp
  ArmStates.cpp )

set( rrt_srcs RRTStarPlanner.cpp
  ProbCogArm.cpp )

set( maze_srcs maze_main.cpp
  MazeStates.cpp
  MazeWidget.cpp )

set( robot_srcs robot_main.cpp
  ProbCogArm.cpp
  ProbCogSearchStates.cpp
  3DCollisionWorld.cpp)

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

add_executable(tests
  ${test_srcs}
)

add_executable(rrtstar
  ${rrt_srcs}
)

target_link_libraries(
  rrtstar
  ompl
)

add_executable(maze_ara
  ${maze_srcs}
  ${maze_headers_moc}
)

target_link_libraries(
  maze_ara
  ${QT_LIBRARIES}
)

add_executable(robot_ara
  ${robot_srcs}
  )

target_link_libraries(
  robot_ara
  lcm
  fcl
  ${Boost_LIBRARIES}
)
