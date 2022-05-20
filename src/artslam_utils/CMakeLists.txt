add_library(artslam_core_utils_library SHARED
        dispatcher.cpp
        types_converter.cpp
        radar_utils.cpp
        eigen_utils.cpp)

target_link_libraries(artslam_core_utils_library ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})