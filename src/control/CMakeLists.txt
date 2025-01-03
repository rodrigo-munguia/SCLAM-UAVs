cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -fpermissive)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED aruco )
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(include)
ament_export_include_directories(include)


#add_executable(dataset_node src/dataset_node.cpp)
set(SOURCE_FILES 
src/control_component.cpp
src/control_commands.cpp
src/control_sensor_callbacks.cpp
../common/Transforms/Euler_to_Ra2b.cpp
../common/Vision/vision.cpp)


add_library(control_component SHARED ${SOURCE_FILES})


#target_include_directories(dataset_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(dataset_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#target_link_libraries( dataset_component ${OpenCV_LIBS} )

#message(WARNING "libraries: ${OpenCV_LIBS}")

target_compile_definitions(control_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

#install(TARGETS dataset_node
#  DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies(control_component
  "interfaces"  
  "rclcpp"
  "rclcpp_components"
  "std_msgs"   
  "sensor_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge"
  "tf2")


rclcpp_components_register_nodes(control_component "control::ControlQuad")
set(node_plugins "${node_plugins}control::ControlQuad;$<TARGET_FILE:control_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


install(TARGETS
  control_component  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
