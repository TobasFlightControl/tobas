cmake_minimum_required(VERSION 3.10)
project(tobas_img_processing)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

set(AMENT_DEPENDENCIES
    rclcpp
	rclcpp_components
	ffmpeg_encoder_decoder
	ffmpeg_image_transport_msgs
	sensor_msgs
	tobas_ic_drivers
	tobas_node
	tobas_msgs
	tobas_msgs_adapter
	OpenCV
	cv_bridge
	PkgConfig
)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavcodec
    libswresample
    libswscale
    libavutil
	libavformat)

file(GLOB_RECURSE NODE_CPP_FILES CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR} nodes/*.cpp)
add_library(${PROJECT_NAME}_node SHARED ${NODE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
ament_target_dependencies(${PROJECT_NAME}_node LIBAV)

rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "CompressedImageViewer" EXECUTABLE compressed_image_viewer)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "CxGb400PublisherNode" EXECUTABLE cx_gb400_publisher)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "FFmpegToROSMsgConverter" EXECUTABLE ffmpeg_to_ros_msg_converter)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "H264Decompressor" EXECUTABLE h264_decompressor)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "MjpgCompressor" EXECUTABLE mjpg_compressor)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "MjpgDecompressor" EXECUTABLE mjpg_decompressor)
rclcpp_components_register_node(${PROJECT_NAME}_node PLUGIN "VideoDevPublisherNode" EXECUTABLE video_dev_publisher)

install(
	TARGETS ${PROJECT_NAME}_node
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)
install(TARGETS ${targets_to_install} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEPENDENCIES})

ament_package()
