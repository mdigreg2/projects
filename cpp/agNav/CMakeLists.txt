cmake_minimum_required(VERSION 2.8.2)
project(aggressive-navigation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  rviz
  #octomap_server
  #dyanmic_reconfigure
  #nodelet
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(octomap REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread )
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)
add_definitions(-DOCTOMAP_NODEBUGOUT)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${OCTOMAP_INCLUDE_DIRS}
  )

link_directories(${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${OCTOMAP_LIBRARY_DIRS}) 

# TODO: review necessary parameters
# generate_dynamic_reconfigure_options(cfg/OctomapServer.cfg)
# generate_dynamic_reconfigure_options(cfg/TraversabilityOctomapServer.cfg)

QT5_WRAP_CPP(MOC_FILES
  include/octomap_rviz_plugins/occupancy_grid_display.h
  OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_LEXICAL_CAST_INCLUDED 
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS OCTOMAP
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  )

add_library(octomap_rviz_plugins src/octomap_rviz_plugins/occupancy_grid_display.cpp src/octomap_rviz_plugins/point_cloud_transparency.cpp ${MOC_FILES})
target_link_libraries(octomap_rviz_plugins ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS} TraversabilityOcTree)

add_library(${PROJECT_NAME} src/traversability_octomap/TraverseOctomapServer.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

add_library(TraversabilityOcTree src/traversability_octomap/TraversabilityOcTree.cpp)
target_link_libraries(TraversabilityOcTree ${LINK_LIBS})

add_library(TraversabilityOctomapServer src/traversability_octomap/TraversabilityOctomapServer.cpp)
target_link_libraries(TraversabilityOctomapServer ${LINK_LIBS})

add_executable(octomap_pubsub example/octomap_pubsub.cpp)
target_link_libraries(octomap_pubsub ${catkin_LIBRARIES})

add_executable(octomap_service example/octomap_service.cpp)
target_link_libraries(octomap_service ${catkin_LIBRARIES})

add_executable(octomap_traversability_node_bis src/traversability_octomap/octomap_traversability_node_bis.cpp)
target_link_libraries(octomap_traversability_node_bis ${PROJECT_NAME} ${LINK_LIBS})
target_link_libraries(octomap_traversability_node_bis TraversabilityOctomapServer)
target_link_libraries(octomap_traversability_node_bis TraversabilityOcTree)

add_executable(octomap_traversability_node src/traversability_octomap/octomap_traversability_node.cpp)
target_link_libraries(octomap_traversability_node ${PROJECT_NAME} ${LINK_LIBS})
target_link_libraries(octomap_traversability_node TraversabilityOctomapServer)
target_link_libraries(octomap_traversability_node TraversabilityOcTree)

add_executable(planar_map src/planar_map/planar_map.cpp)
target_link_libraries(planar_map ${PROJECT_NAME} ${LINK_LIBS})
target_link_libraries(planar_map TraversabilityOcTree)

add_executable(traversability_height src/planar_map/traversability_height.cpp)
target_link_libraries(traversability_height ${PROJECT_NAME} ${LINK_LIBS})

install(DIRECTORY include/octomap_rviz_plugins/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS octomap_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Test scripts
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(kernel_comparison_test src/test/kernel_comparison_test.cpp)
  target_link_libraries(kernel_comparison_test TraversabilityOcTree ${catkin_LIBRARIES})
endif()
