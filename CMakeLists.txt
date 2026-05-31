cmake_minimum_required(VERSION 3.8)
project(mujoco_sim_ros2)

set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# download dependencies
include(FetchContent)

set(MUJOCO_DEP_VERSION_lodepng
    b4ed2cd7ecf61d29076169b49199371456d4f90b
    CACHE STRING "Version of `lodepng` to be fetched."
)

# Fetch lodepng dependency.
if (NOT TARGET lodepng)
  FetchContent_Declare(
      lodepng
      GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
      GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )

  FetchContent_GetProperties(lodepng)
  if (NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
  endif ()
endif ()

# find dependencies
set(OpenGL_GL_PREFERENCE GLVND)
find_package(ament_cmake REQUIRED)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ament_index_cpp REQUIRED)
# tinyxml2 (the same one ros2_control's hardware_interface links). Discover it
# through the ROS tinyxml2_vendor package so it works across distros: Ubuntu
# 24.04 ships tinyxml2's CMake config, but 22.04 (Humble) does not. tinyxml2_vendor
# prepends a FindTinyXML2 module and ensures the tinyxml2::tinyxml2 imported
# target (used on the mujoco_sim link step below) exists on both.
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(mujoco_sim
    src/glfw_adapter.cc
    src/glfw_dispatch.cc
    src/platform_ui_adapter.cc
    src/simulate.cc
    src/main.cc)
target_link_libraries(mujoco_sim PUBLIC mujoco::mujoco glfw lodepng Threads::Threads)
# tinyxml2 ODR fix: libmujoco statically bundles tinyxml2 and exports its symbols, while
# ros2_control's libhardware_interface uses the system libtinyxml2. Loading a ros2_control
# plugin then makes hardware_interface's tinyxml2 objects bind to libmujoco's copy (different
# ABI), corrupting the heap (crash: "free(): invalid pointer" in XMLDocument's dtor). Force
# the system libtinyxml2 to be an early DT_NEEDED of mujoco_sim so it wins symbol resolution
# in the global scope and every tinyxml2 reference resolves to a single shared copy.
# --no-as-needed is required because mujoco_sim has no direct tinyxml2 references and the
# entry would otherwise be stripped; the flag + library path are passed as one -Wl token so
# the toolchain cannot reorder them.
target_link_options(mujoco_sim PRIVATE
    "-Wl,--no-as-needed,$<TARGET_FILE:tinyxml2::tinyxml2>,--as-needed")
ament_target_dependencies(mujoco_sim PUBLIC rclcpp pluginlib controller_manager)
target_include_directories(mujoco_sim PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>" PRIVATE src)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME})

install(TARGETS mujoco_sim
    DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp pluginlib mujoco)

ament_package()
