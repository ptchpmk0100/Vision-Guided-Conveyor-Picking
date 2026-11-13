cmake_minimum_required(VERSION 3.16)
project(twin_condition_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(yaml-cpp REQUIRED)

if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()
if(NOT ONNXRUNTIME_ROOT)
  message(FATAL_ERROR "ONNXRUNTIME_ROOT must name the unpacked ONNX Runtime root")
endif()
if(NOT EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h" OR
   NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
  message(FATAL_ERROR "ONNXRUNTIME_ROOT is invalid: ${ONNXRUNTIME_ROOT}")
endif()

add_library(twin_condition_core
  src/crop.cpp
  src/classifier.cpp
  src/extrinsic.cpp
)
target_compile_options(twin_condition_core PRIVATE -Wall -Wextra)
target_include_directories(twin_condition_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
  PRIVATE
    "${ONNXRUNTIME_ROOT}/include"
)
target_link_libraries(twin_condition_core
  PUBLIC ${OpenCV_LIBS} yaml-cpp
  PRIVATE "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so"
)
set_target_properties(twin_condition_core PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
)

add_executable(condition_node src/condition_node.cpp)
target_compile_options(condition_node PRIVATE -Wall -Wextra)
target_include_directories(condition_node PRIVATE "${ONNXRUNTIME_ROOT}/include")
target_link_libraries(condition_node twin_condition_core)
ament_target_dependencies(condition_node rclcpp sensor_msgs vision_msgs)
set_target_properties(condition_node PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
)

install(TARGETS twin_condition_core condition_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_path(RAPIDJSON_INCLUDE_DIR rapidjson/document.h REQUIRED)

  ament_add_gtest(test_crop test/test_crop.cpp)
  target_compile_options(test_crop PRIVATE -Wall -Wextra)
  target_include_directories(test_crop PRIVATE test "${RAPIDJSON_INCLUDE_DIR}")
  target_compile_definitions(test_crop PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    EXTRINSIC_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../twin_action_demo/config/overhead_camera_extrinsic.yaml"
  )
  target_link_libraries(test_crop twin_condition_core)
  set_target_properties(test_crop PROPERTIES BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib")

  ament_add_gtest(test_classifier test/test_classifier.cpp)
  target_compile_options(test_classifier PRIVATE -Wall -Wextra)
  target_include_directories(test_classifier PRIVATE test "${RAPIDJSON_INCLUDE_DIR}")
  target_compile_definitions(test_classifier PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    EXTRINSIC_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../twin_action_demo/config/overhead_camera_extrinsic.yaml"
  )
  target_link_libraries(test_classifier twin_condition_core)
  set_target_properties(test_classifier PROPERTIES BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib")
endif()

ament_export_include_directories(include)
ament_package()
