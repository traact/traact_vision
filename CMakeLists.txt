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
        src/traact/vision/FLOutsideInTracking.cpp src/traact/vision/FLOutsideInTracking.h src/traact/util/GenerateMultiCameraBATestData.cpp src/traact/util/GenerateMultiCameraBATestData.h src/traact/math/ceres/TargetReprojectionError.h src/traact/math/ceres/DistanceError3D3D.h src/traact/vision/Image.cpp src/traact/vision/Image.h src/traact/vision/BruteForcePosePnP.cpp src/traact/vision/BruteForcePosePnP.h)


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

