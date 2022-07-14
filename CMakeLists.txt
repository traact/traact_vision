cmake_minimum_required(VERSION 3.16)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

list(APPEND CMAKE_MODULE_PATH ${CONAN_TRAACT_RUN_ENV_ROOT}/cmake)

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

include(traact_default_library_setup)

set(VISION_SRC src/traact/vision.h src/traact/traact_vision_export.h
        src/traact/component/vision/BasicVisionPattern.h
        src/traact/vision/SubPixelEdgeDectection.h
        src/traact/vision.cpp src/traact/math/perspective.cpp src/traact/math/perspective.h src/traact/vision_datatypes.h src/traact/opencv/OpenCVUtils.h src/traact/math/ceres/PointReprojectionError.h src/traact/math/utils.h src/traact/vision/UndistortionHelper.cpp src/traact/vision/UndistortionHelper.h
        src/traact/util/GenerateMultiCameraBATestData.cpp src/traact/util/GenerateMultiCameraBATestData.h src/traact/math/ceres/TargetReprojectionError.h src/traact/math/ceres/DistanceError3D3D.h src/traact/vision/Image.cpp src/traact/vision/Image.h src/traact/vision/BruteForcePosePnP.cpp src/traact/vision/BruteForcePosePnP.h src/traact/vision/outside_in/Point3DCandidate.cpp src/traact/vision/outside_in/Point3DCandidate.h src/traact/vision/outside_in/TrackingCamera.cpp src/traact/vision/outside_in/TrackingCamera.h src/traact/vision/outside_in/PointEstimation.cpp src/traact/vision/outside_in/PointEstimation.h src/traact/vision/outside_in/FindTargetPoints.cpp src/traact/vision/outside_in/FindTargetPoints.h src/traact/vision/bundle_adjustment/BundleAdjustment.cpp src/traact/vision/bundle_adjustment/BundleAdjustment.h src/traact/vision/bundle_adjustment/BACamera.cpp src/traact/vision/bundle_adjustment/BACamera.h src/traact/vision/bundle_adjustment/BAData.cpp src/traact/vision/bundle_adjustment/BAData.h src/traact/vision/bundle_adjustment/BATarget.cpp src/traact/vision/bundle_adjustment/BATarget.h
        src/traact/vision/bundle_adjustment/cost_function/CeresRefNPointReprojectionError.h
        src/traact/vision/bundle_adjustment/cost_function/CeresRefTargetNPointLengthError.h
        src/traact/vision/bundle_adjustment/cost_function/CeresTargetNPointReprojectionError.h
        src/traact/vision/bundle_adjustment/cost_function/CostFunctionUtils.h src/traact/vision/GPUImage.cpp src/traact/vision/GPUImage.h src/traact/vision/GpuUndistortionHelper.cpp src/traact/vision/GpuUndistortionHelper.h)


add_library(${CONAN_PACKAGE_NAME} SHARED ${VISION_SRC})

conan_target_link_libraries(${CONAN_PACKAGE_NAME})

install(DIRECTORY "src/traact" # source directory
        DESTINATION "include" # target directory
        FILES_MATCHING # install only matched files
        PATTERN "*.h" # select header files
        )

include(traact_default_library_final)

if (WITH_TESTS)
    add_subdirectory(tests)
endif ()

