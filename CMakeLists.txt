cmake_minimum_required(VERSION 3.8)
project(ruka_ball)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)  

if(DEFINED ENV{<CYPHAL_DIR>})
    set(CYPHAL_DIR $ENV{CYPHAL_DIR})
else()
    get_filename_component(CYPHAL_DIR
                           "../libcxxcanard"
                           ABSOLUTE)
endif()
message(STATUS "${CMAKE_CURRENT_LIST_DIR} | Using <${CYPHAL_DIR}> as directory of libcxxcanard")

add_subdirectory(${CYPHAL_DIR} ${PROJECT_BINARY_DIR}/build/libcxxcanard)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  std_msgs
  rclcpp
  rclcpp_lifecycle
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(ruka_cv src/ruka_cv.cpp)

target_include_directories(ruka_cv PUBLIC
${CYPHAL_DIR}
${COMMON_LIBS_DIR}
#$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include
)

target_include_directories(ruka_cv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(ruka_cv PUBLIC libcxxcanard)

ament_target_dependencies(
  ruka_cv PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


target_compile_features(ruka_cv PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ruka_cv
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
