
find_package(GTest)

set(TEST_SOURCES
        test_main.cpp
        test_pose_estimation.cpp test_estimate_3d_point.cpp test_ceres_elements.cpp)

add_executable(traact_vision_test ${TEST_SOURCES})

set_target_properties(traact_vision_test PROPERTIES CXX_STANDARD 17)


target_link_libraries(traact_vision_test PUBLIC
        traact_vision
        traact_spatial
        GTest::gtest
        Ceres::ceres
        )


target_include_directories(traact_vision_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src        
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}
        )

install(TARGETS traact_vision_test
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})