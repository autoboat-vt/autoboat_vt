cmake_minimum_required(VERSION 3.10)
project(wind_arrow)

# Gazebo Harmonic uses gz-sim8 and gz-plugin2
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(ament_cmake REQUIRED)

set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

# Plugin source
add_library(WindArrow SHARED src/WindArrow.cc)
set_property(TARGET WindArrow PROPERTY CXX_STANDARD 17)

target_link_libraries(WindArrow
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-common5::gz-common5
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)

# If using rclcpp logging or ROS2 communication
find_package(rclcpp REQUIRED)
target_link_libraries(WindArrow rclcpp::rclcpp)

# Optional: expose plugin install path for SDF
install(TARGETS WindArrow
  LIBRARY DESTINATION lib
)

ament_package()
