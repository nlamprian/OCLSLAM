cmake_minimum_required ( VERSION 2.8.12.2 )
project ( OCLSLAM )
set ( FNAME "oclslam" )
set ( PROJECT_VERSION_MAJOR 0 )
set ( PROJECT_VERSION_MINOR 1 )
set ( PROJECT_VERSION_PATCH 0 )

option ( BUILD_EXAMPLES "Build examples" OFF )
option ( BUILD_TESTS "Build tests" OFF )

list ( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package ( OpenCL REQUIRED )
find_package ( OpenGL REQUIRED )

add_definitions ( -std=c++0x )

set ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -static-libstdc++" )
# set ( WARNINGS "-Wall -Wextra" )
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNINGS}" )

set ( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )
set ( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib )

set ( COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/include )

include_directories ( ${COMMON_INCLUDES} 
                      ${OPENCL_INCLUDE_DIR} )

# set ( CLUtils_ROOT <set-to-your-local-path-if-necessary> )
find_package ( CLUtils QUIET )
if ( NOT CLUtils_FOUND )
    message ( STATUS "CLUtils not found:" )
    message ( STATUS " - Library will be downloaded from source" )
    add_subdirectory ( external/CLUtils )
endif ( NOT CLUtils_FOUND )

# set ( GuidedFilter_ROOT <set-to-your-local-path-if-necessary> )
find_package ( GuidedFilter QUIET )
if ( NOT GuidedFilter_FOUND )
    message ( STATUS "GuidedFilter not found:" )
    message ( STATUS " - Library will be downloaded from source" )
    add_subdirectory ( external/GuidedFilter )
else ( NOT GuidedFilter_FOUND )
    file ( 
        COPY ${GuidedFilter_INCLUDE_DIR}/GuidedFilter/kernels/GF 
        DESTINATION ${PROJECT_BINARY_DIR}/kernels 
    )
endif ( NOT GuidedFilter_FOUND )

# set ( RBC_ROOT <set-to-your-local-path-if-necessary> )
find_package ( RBC QUIET )
if ( NOT RBC_FOUND )
    message ( STATUS "RandomBallCover not found:" )
    message ( STATUS " - Library will be downloaded from source" )
    add_subdirectory ( external/RandomBallCover )
else ( NOT RBC_FOUND )
    file ( 
        COPY ${RBC_INCLUDE_DIR}/RBC/kernels/RBC 
        DESTINATION ${PROJECT_BINARY_DIR}/kernels 
    )
endif ( NOT RBC_FOUND )

find_package ( Eigen 3.2.0 QUIET )
if ( NOT EIGEN_FOUND )
    message ( STATUS "Eigen not found:" )
    message ( STATUS " - Library will be downloaded from source" )
    add_subdirectory ( external/Eigen )
endif ( NOT EIGEN_FOUND )

# set ( ICP_ROOT <set-to-your-local-path-if-necessary> )
find_package ( ICP QUIET )
if ( NOT ICP_FOUND )
    message ( STATUS "ICP not found:" )
    message ( STATUS " - Library will be downloaded from source" )
    add_subdirectory ( external/ICP )
else ( NOT ICP_FOUND )
    file ( 
        COPY ${ICP_INCLUDE_DIR}/ICP/kernels/ICP 
        DESTINATION ${PROJECT_BINARY_DIR}/kernels 
    )
endif ( NOT ICP_FOUND )

# set ( OCTOMAP_ROOT <set-to-your-local-path-if-necessary> )
find_package ( octomap QUIET )
if ( NOT OCTOMAP_FOUND )
    message ( STATUS "octomap not found:" )
    message ( STATUS " - Library will be downloaded from source" )
    add_subdirectory ( external/octomap )
endif ( NOT OCTOMAP_FOUND )

add_subdirectory ( src )
add_subdirectory ( kernels )
add_subdirectory ( docs )

if ( BUILD_EXAMPLES )
    add_subdirectory ( examples )
endif ( BUILD_EXAMPLES )

if ( BUILD_TESTS )
    enable_testing (  )
    
    find_package ( GTest QUIET )
    if ( NOT GTEST_FOUND )
        message ( STATUS "gtest not found:" )
        message ( STATUS " - Library will be downloaded from source" )
        add_subdirectory ( external/gtest )
    endif ( NOT GTEST_FOUND )
    
    add_subdirectory ( tests )
endif ( BUILD_TESTS )
