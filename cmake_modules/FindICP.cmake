# Try to find ICP
#
# The following variables are optionally searched for defaults:
#   ICP_ROOT         - Root directory of ICP source tree
#
# Once done, this will define:
#   ICP_FOUND        - system has ICP
#   ICP_INCLUDE_DIR  - the ICP include directory
#   ICP_LIBRARIES    - link these to use ICP

find_path ( 
    ICP_INCLUDE_DIR
    NAMES ICP/common.hpp ICP/data_types.hpp ICP/algorithms.hpp ICP/tests/helper_funcs.hpp 
    HINTS ${ICP_ROOT}/include 
          /usr/include
          /usr/local/include 
    DOC "The directory where ICP headers reside"
)

find_library ( 
    ICP_LIB_ALGORITHMS 
    NAMES ICPAlgorithms 
    PATHS ${ICP_ROOT}/build/lib 
          ${ICP_ROOT}/../ICP-build/lib 
          /usr/lib/ICP 
          /usr/local/lib/ICP 
    DOC "The ICP algorithms library"
)

find_library ( 
    ICP_LIB_HELPERFUNCS 
    NAMES ICPHelperFuncs  
    PATHS ${ICP_ROOT}/build/lib 
          ${ICP_ROOT}/../ICP-build/lib 
          /usr/lib/ICP 
          /usr/local/lib/ICP 
    DOC "The ICP helper functions library"
)

include ( FindPackageHandleStandardArgs )

find_package_handle_standard_args ( 
    ICP 
    FOUND_VAR ICP_FOUND
    REQUIRED_VARS ICP_INCLUDE_DIR ICP_LIB_ALGORITHMS ICP_LIB_HELPERFUNCS 
)

if ( ICP_FOUND )
    set ( ICP_LIBRARIES ${ICP_LIB_ALGORITHMS} ${ICP_LIB_HELPERFUNCS} )
    message ( STATUS "Found ICP:" )
    message ( STATUS " - Includes: ${ICP_INCLUDE_DIR}" )
    message ( STATUS " - Libraries: ${ICP_LIBRARIES}" )
else ( ICP_FOUND )
    set ( ICP_LIBRARIES )
endif ( ICP_FOUND )

mark_as_advanced ( 
    ICP_INCLUDE_DIR
    ICP_LIB_ALGORITHMS 
    ICP_LIB_HELPERFUNCS 
)
