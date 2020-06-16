cmake_minimum_required(VERSION 3.9)

option(WITH_TESTS "Build Tests" ON)
option(WITH_COVERAGE "" ON)

set(CMAKE_CXX_STANDARD 17)

# public header
set(LIB_HEADERS include/traact/vision.h
        include/traact/component/vision/BasicVisionPattern.h)

set(PLUGIN_HEADERS
        include/traact/component/vision/OpenCvVideoCapture.h
        )


# rest of source code
set(LIB_SOURCES src/vision.cpp)

set(PLUGIN_SOURCES src/VisionPlugin.cpp
        include/traact/component/vision/OpenCvVideoCapture.cpp include/traact/component/vision/OpenCvWindow.cpp include/traact/component/vision/OpenCvWindow.h)


# conan is required hard, library name, version and description are configured from conan
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

conan_basic_setup(TARGETS)

if (WITH_COVERAGE)
    #add_compile_options(-g -O0 -Wall -W -Wshadow -Wunused-variable -Wunused-parameter -Wunused-function -Wunused -Wno-system-headers -Wno-deprecated -Woverloaded-virtual -Wwrite-strings -fprofile-arcs -ftest-coverage)
    add_link_options(-fprofile-arcs -ftest-coverage)
endif ()

if (CMAKE_BUILD_TYPE EQUAL "DEBUG")
    add_definitions(-DTBB_USE_DEBUG=1)
endif ()

add_definitions(-DTBB_PREVIEW_FLOW_GRAPH_FEATURES=1)
add_definitions(-DTBB_PREVIEW_FLOW_GRAPH_NODES=1)

add_link_options( -fno-gnu-unique )

add_library(${CONAN_PACKAGE_NAME} SHARED
        ${LIB_HEADERS}
        ${LIB_SOURCES}
        )

conan_target_link_libraries(${CONAN_PACKAGE_NAME})

set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES VERSION ${CONAN_PACKAGE_VERSION})
#set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES PUBLIC_HEADER ${LIB_HEADERS})

target_include_directories(${CONAN_PACKAGE_NAME} PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(${CONAN_PACKAGE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

IF (NOT WIN32)
    MESSAGE(STATUS "Set boost::ublas alignment to 16")
    set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES COMPILE_DEFINITIONS "BOOST_UBLAS_BOUNDED_ARRAY_ALIGN=__attribute__ ((aligned (16)))")
ENDIF (NOT WIN32)

install(TARGETS ${CONAN_PACKAGE_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


# Plugin -----------------------
add_library(traact_vision_plugin SHARED
        ${PLUGIN_HEADERS}
        ${PLUGIN_SOURCES}
        )

target_link_libraries(traact_vision_plugin
        ${CONAN_PACKAGE_NAME})

#set_target_properties(traact_spatial_plugin PROPERTIES VERSION ${CONAN_PACKAGE_VERSION})
#set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES PUBLIC_HEADER ${LIB_HEADERS})

target_include_directories(${CONAN_PACKAGE_NAME} PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(${CONAN_PACKAGE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

IF (NOT WIN32)
    MESSAGE(STATUS "Set boost::ublas alignment to 16")
    set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES COMPILE_DEFINITIONS "BOOST_UBLAS_BOUNDED_ARRAY_ALIGN=__attribute__ ((aligned (16)))")
ENDIF (NOT WIN32)

install(TARGETS traact_vision_plugin
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#-----------------------------


if (WITH_TESTS)
    #add_subdirectory(tests)
endif ()

