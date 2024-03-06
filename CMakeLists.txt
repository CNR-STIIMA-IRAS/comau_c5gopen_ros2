cmake_minimum_required(VERSION 3.8)
project(comau_c5gopen_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(cnr_mqtt_client REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(Mosquitto IMPORTED_TARGET libmosquitto REQUIRED)

include_directories(include)

set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  cnr_mqtt_client
  Mosquitto
)

add_library(${PROJECT_NAME} SHARED src/comau_c5gopen_hw.cpp
                                   src/mqtt_driver.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${cnr_mqtt_client_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} ${HW_IF_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(${PROJECT_NAME} ${HW_IF_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
        include
)

ament_export_libraries(
        ${PROJECT_NAME}
)

ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}_plugin.xml)

ament_package()
