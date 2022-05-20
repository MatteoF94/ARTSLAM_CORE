add_executable(artslam_core_io_test artslam_core_io_test.cpp)
target_link_libraries(artslam_core_io_test
        artslam_core_io_library
        artslam_core_utils_library
        ${Boost_LOG_LIBRARIES}
        ${Boost_LOG_SETUP_LIBRARIES}
        )

add_executable(artslam_core_types_test artslam_core_types_test.cpp)
target_link_libraries(artslam_core_types_test
        artslam_core_types_library
        )

add_executable(artslam_core_g2o_test artslam_core_g2o_test.cpp)
target_link_libraries(artslam_core_g2o_test
        ${G2O_TYPES_DATA}
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_SOLVER_PCG}
        ${G2O_SOLVER_CSPARSE}   # be aware of that CSPARSE is released under LGPL
        ${G2O_SOLVER_CHOLMOD}   # be aware of that cholmod is released under GPL
        ${G2O_TYPES_SLAM3D}
        ${G2O_TYPES_SLAM3D_ADDONS}
        )
