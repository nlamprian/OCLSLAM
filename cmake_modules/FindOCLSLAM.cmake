# Try to find OCLSLAM
#
# The following variables are optionally searched for defaults:
#   OCLSLAM_ROOT         - Root directory of OCLSLAM source tree
#
# Once done, this will define:
#   OCLSLAM_FOUND        - system has OCLSLAM
#   OCLSLAM_INCLUDE_DIR  - the OCLSLAM include directory
#   OCLSLAM_LIBRARIES    - link these to use OCLSLAM

find_path ( 
    OCLSLAM_INCLUDE_DIR
    NAMES OCLSLAM/common.hpp OCLSLAM/data_types.hpp OCLSLAM/algorithms.hpp OCLSLAM/tests/helper_funcs.hpp 
    HINTS ${OCLSLAM_ROOT}/include 
          /usr/include
          /usr/local/include 
    DOC "The directory where OCLSLAM headers reside"
)

find_library ( 
    OCLSLAM_LIB_ALGRITHMS 
    NAMES oclslamAlgorithms 
    PATHS ${OCLSLAM_ROOT}/build/lib 
          ${OCLSLAM_ROOT}/../OCLSLAM-build/lib 
          /usr/lib/OCLSLAM 
          /usr/local/lib/OCLSLAM 
    DOC "The OCLSLAM algorithms library"
)

find_library ( 
    OCLSLAM_LIB_HELPERFUNCS 
    NAMES oclslamHelperFuncs  
    PATHS ${OCLSLAM_ROOT}/build/lib 
          ${OCLSLAM_ROOT}/../OCLSLAM-build/lib 
          /usr/lib/OCLSLAM 
          /usr/local/lib/OCLSLAM 
    DOC "The OCLSLAM helper functions library"
)

include ( FindPackageHandleStandardArgs )

find_package_handle_standard_args ( 
    OCLSLAM 
    FOUND_VAR OCLSLAM_FOUND
    REQUIRED_VARS OCLSLAM_INCLUDE_DIR OCLSLAM_LIB_ALGRITHMS OCLSLAM_LIB_HELPERFUNCS 
)

if ( OCLSLAM_FOUND )
    set ( OCLSLAM_LIBRARIES ${OCLSLAM_LIB_ALGRITHMS} ${OCLSLAM_LIB_HELPERFUNCS} )
    message ( STATUS "Found OCLSLAM:" )
    message ( STATUS " - Includes: ${OCLSLAM_INCLUDE_DIR}" )
    message ( STATUS " - Libraries: ${OCLSLAM_LIBRARIES}" )
else ( OCLSLAM_FOUND )
    set ( OCLSLAM_LIBRARIES )
endif ( OCLSLAM_FOUND )

mark_as_advanced ( 
    OCLSLAM_INCLUDE_DIR
    OCLSLAM_LIB_ALGRITHMS 
    OCLSLAM_LIB_HELPERFUNCS 
)
