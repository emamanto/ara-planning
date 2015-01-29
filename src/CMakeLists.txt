cmake_minimum_required(VERSION 2.8.0)
project( ara )

find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

set( srcs main.cpp
  ArmSim.cpp
  Visualizer.cpp )

set( headers ArmSim.h
  Visualizer.h )

Qt4_wrap_cpp( headers_moc ${headers} )

add_executable(ara
  ${srcs}
  ${headers_moc} )

target_link_libraries(
  ara
  ${QT_LIBRARIES}
)