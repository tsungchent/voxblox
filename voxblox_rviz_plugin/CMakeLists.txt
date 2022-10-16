cmake_minimum_required(VERSION 3.21)
project(voxblox_rviz_plugin)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(eigen_vendor REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 14)
set(rviz2_QT_VERSION 5.10.1)
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz2_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz2_QT_VERSION: ${rviz2_QT_VERSION}")
  find_package(Qt4 ${rviz2_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
  qt4_wrap_cpp(MOC_FILES
    ${INCLUDE_FILES_QT}
  )
else()
  message(STATUS "Using Qt5 based on the rviz2_QT_VERSION: ${rviz2_QT_VERSION}")
  find_package(Qt5 ${rviz2_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
  QT5_WRAP_CPP(MOC_FILES
    ${INCLUDE_FILES_QT}
  )
endif()

## Avoid Qt signals and slots defining "emit", "slots", etc.
add_definitions(-DQT_NO_KEYWORDS)

set(HEADER_FILES include/voxblox_rviz_plugin/voxblox_mesh_display.h include/voxblox_rviz_plugin/voxblox_mesh_visual.h)

set(SRC_FILES src/voxblox_mesh_display.cc src/voxblox_mesh_visual.cc )

add_library(${PROJECT_NAME}
  ${SRC_FILES}
  ${HEADER_FILES}
  ${MOC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
    rviz_common
    voxblox
    voxblox_msgs
    rviz_ogre_vendor
    eigen_vendor
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
)

##########
# EXPORT #
##########
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES plugin_description.xml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share)

install(
  DIRECTORY icons/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/icons)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

ament_package()
