cmake_minimum_required(VERSION 3.5)
project(innovusion)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-format-extra-args -Wno-format)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(INNOVUSION_SDK_PATH
  driver/falcon/sdk
)

IF(WIN32)
  MESSAGE(STATUS "Now is windows, Not support yet")
ELSEIF(APPLE)
  MESSAGE(STATUS "Now is Apple systens.")
  set(INNOVUSION_SDK_LIB_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/${INNOVUSION_SDK_PATH}/lib/macos-aarch64
  )
ELSEIF(UNIX)
  MESSAGE(STATUS "Now is UNIX-like OS's.")

  if((${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    OR(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "amd64"))
    set(INNOVUSION_SDK_LIB_PATH
      ${CMAKE_CURRENT_SOURCE_DIR}/${INNOVUSION_SDK_PATH}/lib/linux-x86
    )
  elseif((${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    OR(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64"))
    set(INNOVUSION_SDK_LIB_PATH
      ${CMAKE_CURRENT_SOURCE_DIR}/${INNOVUSION_SDK_PATH}/lib/linux-arm
    )
  endif()
ENDIF()

# publisher
add_executable(publisher src/publisher.cc driver/falcon/driver_falcon.cc ${INNOVUSION_SDK_PATH}/src/sdk_common/converter/cframe_converter.cpp)

target_include_directories(publisher
  PRIVATE
  include/innovusion
  driver/falcon
  ${INNOVUSION_SDK_PATH}
  ${INNOVUSION_SDK_PATH}/apps
  ${INNOVUSION_SDK_PATH}/src
  ${PCL_INCLUDE_DIRS}
)

target_link_directories(publisher
  PRIVATE
  ${INNOVUSION_SDK_LIB_PATH})

target_link_libraries(publisher
  innolidarsdk
  innolidarsdkclient
  innolidarsdkcommon
  innolidarutils
  innolidarwsutils
  ${PCL_LIBRARIES}
)

ament_target_dependencies(publisher
  rclcpp
  PCL
  std_msgs
  sensor_msgs
  message_filters
  pcl_msgs
  pcl_conversions
)

IF(APPLE)
  add_custom_command(TARGET publisher
    POST_BUILD COMMAND
    ${CMAKE_INSTALL_NAME_TOOL} -change libinnolidarsdk.so.0 @rpath/libinnolidarsdk.so publisher
    COMMAND
    ${CMAKE_INSTALL_NAME_TOOL} -add_rpath "@executable_path/" publisher
  )
ELSE()
  set_target_properties(publisher PROPERTIES
    BUILD_WITH_INSTALL_RPATH FALSE
    LINK_FLAGS "-Wl,-rpath,$ORIGIN/")
ENDIF()

# subscriber
add_executable(subscriber src/subscriber.cc)

target_include_directories(subscriber
  PRIVATE
  include/innovusion
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(subscriber
  ${PCL_LIBRARIES}
)

ament_target_dependencies(subscriber
  "PCL"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  message_filters
  pcl_msgs
  pcl_conversions
)

# install group
install(TARGETS
  subscriber
  publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ${INNOVUSION_SDK_LIB_PATH}/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
