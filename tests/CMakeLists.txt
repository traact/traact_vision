
set(TEST_SOURCES
        test_main.cpp
        test_perspective.cpp test_pose_estimation.cpp test_estimate_3d_point.cpp test_ceres_elements.cpp test_FL_OutsideInTracking.cpp)

set(CMAKE_CUDA_STANDARD 14)

add_executable(traact_vision_test ${TEST_SOURCES})

set_target_properties(traact_vision_test PROPERTIES CXX_STANDARD 17)

target_link_libraries(traact_vision_test PUBLIC
        ${CONAN_PACKAGE_NAME}
        )


# conan weirdness ...
if (TARGET CONAN_PKG::GTest)
    target_link_libraries(traact_vision_test PUBLIC CONAN_PKG::GTest)
else ()
    target_link_libraries(traact_vision_test PUBLIC CONAN_PKG::gtest)
endif ()


target_include_directories(traact_vision_test
        PRIVATE
        ${PROJECT_BINARY_DIR}/
        ${PROJECT_SOURCE_DIR}/src
        )
