add_library(artslam_core_io_library SHARED
        pointcloud_io.cpp
        kitti_reader.cpp
        mulran_io.cpp)

target_link_libraries(artslam_core_io_library
        ${Boost_LOG_LIBRARIES}
        ${Boost_LOG_SETUP_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES})