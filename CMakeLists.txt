cmake_minimum_required(VERSION 3.16)

set(TARGET_NAME traact_vision)

project(${TARGET_NAME} VERSION "0.0.0" LANGUAGES CXX)

find_package(traact_base)
list(APPEND CMAKE_MODULE_PATH ${traact_base_INCLUDE_DIR})

include(traact_default_library_setup)

set(VISION_PUBLIC_HEADER
src/traact/vision.h     
    include/traact/component/vision/BasicVisionPattern.h
    include/traact/vision/SubPixelEdgeDectection.h
    include/traact/math/perspective.h 
    include/traact/vision_datatypes.h 
    include/traact/opencv/OpenCVUtils.h 
    include/traact/math/ceres/PointReprojectionError.h 
    include/traact/math/utils.h 
    include/traact/vision/UndistortionHelper.h
    include/traact/util/GenerateMultiCameraBATestData.h 
    include/traact/math/ceres/TargetReprojectionError.h 
    include/traact/math/ceres/DistanceError3D3D.h 
    src/traact/vision/Image.h 
    include/traact/vision/BruteForcePosePnP.h 
    include/traact/vision/outside_in/Point3DCandidate.h 
    include/traact/vision/outside_in/TrackingCamera.h 
    include/traact/vision/outside_in/PointEstimation.h 
    include/traact/vision/outside_in/FindTargetPoints.h 
    include/traact/vision/bundle_adjustment/BundleAdjustment.h 
    include/traact/vision/bundle_adjustment/BACamera.h 
    include/traact/vision/bundle_adjustment/BAData.h 
    include/traact/vision/bundle_adjustment/BATarget.h
    include/traact/vision/bundle_adjustment/cost_function/CeresRefNPointReprojectionError.h
    include/traact/vision/bundle_adjustment/cost_function/CeresRefTargetNPointLengthError.h
    include/traact/vision/bundle_adjustment/cost_function/CeresTargetNPointReprojectionError.h
    include/traact/vision/bundle_adjustment/cost_function/CostFunctionUtils.h 
    include/traact/vision/GPUImage.h 
    include/traact/vision/GpuUndistortionHelper.h
    include/traact/util/KinectLookupTable.h
    )

set(VISION_SRC 
    src/traact/vision.cpp 
    src/traact/math/perspective.cpp 
    src/traact/vision/UndistortionHelper.cpp 
    src/traact/util/GenerateMultiCameraBATestData.cpp 
    src/traact/vision/BruteForcePosePnP.cpp 
    src/traact/vision/outside_in/Point3DCandidate.cpp 
    src/traact/vision/outside_in/TrackingCamera.cpp 
    src/traact/vision/outside_in/PointEstimation.cpp 
    src/traact/vision/outside_in/FindTargetPoints.cpp 
    src/traact/vision/bundle_adjustment/BundleAdjustment.cpp 
    src/traact/vision/bundle_adjustment/BACamera.cpp 
    src/traact/vision/bundle_adjustment/BAData.cpp 
    src/traact/vision/bundle_adjustment/BATarget.cpp 
    src/traact/vision/Image.cpp
    src/traact/vision/GPUImage.cpp 
    src/traact/vision/GpuUndistortionHelper.cpp 
    src/traact/util/KinectLookupTable.cpp 
    )


add_library(${TARGET_NAME} SHARED ${VISION_SRC})

find_package(traact_spatial)
find_package(OpenCV)
find_package(Ceres)




target_link_libraries(${TARGET_NAME} PUBLIC traact_core::traact_core traact_spatial::traact_spatial opencv::opencv Ceres::ceres)
generate_export_header(${TARGET_NAME} EXPORT_FILE_NAME ${CMAKE_CURRENT_SOURCE_DIR}/include/traact/traact_vision_export.h)

target_include_directories(${TARGET_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

include(traact_default_library_final)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (WITH_TESTS)
    add_subdirectory(tests)
endif ()