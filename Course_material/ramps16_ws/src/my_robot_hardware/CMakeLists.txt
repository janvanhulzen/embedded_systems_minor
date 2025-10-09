cmake_minimum_required(VERSION 3.14)
project(my_robot_hardware LANGUAGES CXX)

# ---- C++ standard & warnings ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# cmake_minimum_required(VERSION 3.8)
# project(my_robot_hardware)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# ---- ROS 2 deps ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ---- LibSerial via pkg-config ----
find_package(PkgConfig REQUIRED)
pkg_check_modules(LibSerial REQUIRED libserial)

# ---- Library (the hardware_interface plugin) ----
add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/arm_hardware_interface.cpp
)

# Public headers
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LibSerial_INCLUDE_DIRS}
)

# Use keyword signature here (IMPORTANT)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${LibSerial_LIBRARIES}
)

# Link dirs/flags for LibSerial

# Do NOT add target_link_directories (unnecessary, can provoke warnings)

#target_link_directories(${PROJECT_NAME} PRIVATE ${LibSerial_LIBRARY_DIRS}) # switch off
target_link_directories(${PROJECT_NAME} PRIVATE find_library(libserial)) 

# Pass any extra cflags from pkg-config if present:
target_compile_options(${PROJECT_NAME} PRIVATE ${LibSerial_CFLAGS_OTHER})

# IMPORTANT: use the *plain* signature to avoid mixing with ament's calls
#target_link_libraries(${PROJECT_NAME} ${LibSerial_LIBRARIES})

# ament will inject ROS 2 includes/defs/links
ament_target_dependencies(${PROJECT_NAME}
PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# ---- Plugin export (XML must exist at package top-level) ----
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_interface.xml)

# ---- Install ----
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES my_robot_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---- ament exports ----

ament_export_include_directories(include)

ament_export_libraries(
  ${PROJECT_NAME}
)

# libserial toegevoegd...   LibSerial

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
