cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)
set (CMAKE_CXX_STANDARD 11)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	genmsg
	cv_bridge
	image_transport
	autoware_msgs
	sensor_msgs
	message_generation
	wato_common
	)
find_package(OpenCV REQUIRED)

# Runtime dependency
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
	roscpp
	std_msgs
	genmsg
	cv_bridge
	image_transport
	autoware_msgs
	sensor_msgs
	message_generation
	wato_common
)

include_directories(
	/home/henry/WATO/prediction/prediction-ros/devel/include
	/home/henry/WATO/prediction/prediction-ros/src/wato_common/include
    /home/henry/WATO/prediction/prediction-ros/src/frame_publisher/include
    /home/henry/WATO/prediction/prediction-ros/src/detections_publ/include
	${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_library(
	wato_common
	/home/henry/WATO/prediction/prediction-ros/src/wato_common/src/detection_label.cpp
	)
target_link_libraries(
	wato_common
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_library(
	frame_publisher
	/home/henry/WATO/prediction/prediction-ros/src/frame_publisher/src/frame_publisher.cpp
	)
target_link_libraries(
	frame_publisher
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    wato_common
    )
add_dependencies(frame_publisher wato_common_generate_messages_cpp)

## Build talker
SET(GCC_COVERAGE_LINK_FLAGS
"-lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_cvv -lopencv_dpm -lopencv_face -lopencv_photo -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_img_hash -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_viz -lopencv_phase_unwrapping -lopencv_surface_matching -lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_dnn -lopencv_plot -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ml -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_flann -lopencv_xobjdetect -lopencv_imgcodecs -lopencv_objdetect -lopencv_xphoto -lopencv_imgproc -lopencv_core"
)
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

add_executable(talker src/talker.cpp include)
target_link_libraries(
	talker
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	frame_publisher
	)

## Build subscriber
add_executable(subscriber src/subscriber.cpp include)
target_link_libraries(
	subscriber
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	)

## Build my_detection_publisher
add_executable(
	my_detection_publisher
	src/my_detection_publisher.cpp
	include
	)
target_link_libraries(
	my_detection_publisher
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	wato_common
	)