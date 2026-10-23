cmake_minimum_required(VERSION 3.8)
project(wheel_robot)

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleStatus.msg"
  "msg/StepApproachStatus.msg"
  "msg/StepDetection.msg"
  DEPENDENCIES std_msgs
)

# The generic C++ type-support library loads the selected DDS implementation
# (Fast DDS on this system) with dlopen().  Give the dispatcher an explicit
# sibling-library search path so a terminal opened before the latest build does
# not fail to find the newly generated implementation library.
set_target_properties(${PROJECT_NAME}__rosidl_typesupport_cpp PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

set(
  ORBBEC_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/OrbbecSDK_ROS2/orbbec_camera/SDK"
  CACHE PATH
  "Path to the Orbbec legacy SDK bundled with OrbbecSDK_ROS2 main branch"
)
if(NOT EXISTS "${ORBBEC_SDK_ROOT}/include/libobsensor/ObSensor.hpp")
  message(FATAL_ERROR
    "Orbbec legacy SDK not found. Run ${CMAKE_CURRENT_SOURCE_DIR}/tools/fetch_orbbec_sdk.sh"
  )
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(ORBBEC_SDK_PLATFORM arm64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(ORBBEC_SDK_PLATFORM x64)
else()
  message(FATAL_ERROR "Unsupported Orbbec SDK architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(ORBBEC_SDK_LIB_DIR "${ORBBEC_SDK_ROOT}/lib/${ORBBEC_SDK_PLATFORM}")
find_library(
  ORBBEC_SDK_LIBRARY
  NAMES OrbbecSDK
  PATHS "${ORBBEC_SDK_LIB_DIR}"
  NO_DEFAULT_PATH
)
if(NOT ORBBEC_SDK_LIBRARY)
  # The compact deployment keeps the real ARM64 runtime file but Git/SFTP may
  # omit the unversioned linker symlink. CMake can link the versioned file.
  find_file(
    ORBBEC_SDK_LIBRARY
    NAMES libOrbbecSDK.so.1.10.37
    PATHS "${ORBBEC_SDK_LIB_DIR}"
    NO_DEFAULT_PATH
  )
endif()
if(NOT ORBBEC_SDK_LIBRARY)
  message(FATAL_ERROR
    "Orbbec SDK library not found in ${ORBBEC_SDK_LIB_DIR}. "
    "Expected libOrbbecSDK.so or libOrbbecSDK.so.1.10.37."
  )
endif()

# OrbbecSDK 1.10.37 declares these private runtime dependencies, but the ARM64
# linker still needs their location while resolving symbols from the versioned
# SDK library. Keep the original vendor binaries beside libOrbbecSDK and link
# them explicitly so clean builds do not depend on stale files in build/.
find_library(
  ORBBEC_LIVE555_LIBRARY
  NAMES live555
  PATHS "${ORBBEC_SDK_LIB_DIR}"
  NO_DEFAULT_PATH
)
find_library(
  ORBBEC_USB_LIBRARY
  NAMES ob_usb
  PATHS "${ORBBEC_SDK_LIB_DIR}"
  NO_DEFAULT_PATH
)
if(NOT ORBBEC_LIVE555_LIBRARY OR NOT ORBBEC_USB_LIBRARY)
  message(FATAL_ERROR
    "Orbbec ARM64 dependencies are incomplete in ${ORBBEC_SDK_LIB_DIR}. "
    "Expected liblive555.so and libob_usb.so."
  )
endif()

add_executable(astra_camera_node
  src/astra_camera_node.cpp
)
target_compile_features(astra_camera_node PUBLIC cxx_std_17)
target_compile_options(astra_camera_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(astra_camera_node
  ${OpenCV_LIBRARIES}
  ${ORBBEC_SDK_LIBRARY}
  ${ORBBEC_LIVE555_LIBRARY}
  ${ORBBEC_USB_LIBRARY}
)
target_include_directories(astra_camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  "${ORBBEC_SDK_ROOT}/include"
)
set_target_properties(astra_camera_node PROPERTIES
  BUILD_RPATH "${ORBBEC_SDK_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN"
)
ament_target_dependencies(astra_camera_node
  camera_info_manager
  rclcpp
  sensor_msgs
)

add_executable(depth_distance_node
  src/depth_distance_node.cpp
)
target_compile_features(depth_distance_node PUBLIC cxx_std_17)
target_compile_options(depth_distance_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(depth_distance_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_distance_node ${OpenCV_LIBRARIES})
ament_target_dependencies(depth_distance_node
  rclcpp
  sensor_msgs
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(depth_distance_node "${cpp_typesupport_target}")
set_target_properties(depth_distance_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN/.."
)

# Python application nodes are kept in scripts/ to match the workspace layout.
install(PROGRAMS
  scripts/icn.py
  scripts/icn_adaptive.py
  scripts/adaptive_runtime.py
  scripts/race_runtime.py
  scripts/race_posture.py
  scripts/race_operator_console.py
  scripts/stand_gate.py
  scripts/stand_only.py
  scripts/stand_only_core.py
  scripts/step_detector_node.py
  scripts/step_approach_test.py
  scripts/step_preview.py
  scripts/depth_preview.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  astra_camera_node
  depth_distance_node
  DESTINATION lib/${PROJECT_NAME}
)

file(GLOB ORBBEC_SDK_RUNTIME_LIBS "${ORBBEC_SDK_LIB_DIR}/*.so*")
install(FILES
  ${ORBBEC_SDK_RUNTIME_LIBS}
  DESTINATION lib/${PROJECT_NAME}
)

# Future launch files and YAML parameters are installed automatically.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_roundabout_frame_follow
    test/test_roundabout_frame_follow.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_visual_step_strategy
    test/test_visual_step_strategy.py
    TIMEOUT 30
  )
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_stand_gate.py")
    ament_add_pytest_test(
      test_stand_gate
      test/test_stand_gate.py
      TIMEOUT 30
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_stand_only_core.py")
    ament_add_pytest_test(
      test_stand_only_core
      test/test_stand_only_core.py
      TIMEOUT 30
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_menu_contract.py")
    ament_add_pytest_test(
      test_robot_menu_contract
      test/test_robot_menu_contract.py
      TIMEOUT 30
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_checksum_manifest.py")
    ament_add_pytest_test(
      test_checksum_manifest
      test/test_checksum_manifest.py
      TIMEOUT 30
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_competition_inventory.py")
    ament_add_pytest_test(
      test_competition_inventory
      test/test_competition_inventory.py
      TIMEOUT 30
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_live_race_tuning.py")
    ament_add_pytest_test(
      test_live_race_tuning
      test/test_live_race_tuning.py
      TIMEOUT 30
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_live_race_contract.py")
    ament_add_pytest_test(
      test_live_race_contract
      test/test_live_race_contract.py
      TIMEOUT 30
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_live_tuning_file_reload.py")
    ament_add_pytest_test(
      test_live_tuning_file_reload
      test/test_live_tuning_file_reload.py
      TIMEOUT 30
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_adaptive_runtime.py")
    ament_add_pytest_test(
      test_adaptive_runtime
      test/test_adaptive_runtime.py
      TIMEOUT 30
    )
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
