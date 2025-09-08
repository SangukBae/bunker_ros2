cmake_minimum_required(VERSION 3.8)
project(bunker_description)

# Build system
find_package(ament_cmake REQUIRED)

# ------------------------------------------------------------------------------
# Install resources (only if they exist, to avoid configure-time errors)
# ------------------------------------------------------------------------------
set(DIRS_TO_INSTALL
  urdf
  meshes
  launch
  rviz
  config
)

foreach(dir IN LISTS DIRS_TO_INSTALL)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}
      PATTERN "*.pyc" EXCLUDE
      PATTERN ".DS_Store" EXCLUDE
    )
  endif()
endforeach()

# ament resource index entry (recommended for ROS 2 packages)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource/${PROJECT_NAME}")
  install(FILES resource/${PROJECT_NAME}
    DESTINATION share/ament_index/resource_index/packages
  )
endif()

# Install package manifest for introspection tools
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Finalize
ament_package()
