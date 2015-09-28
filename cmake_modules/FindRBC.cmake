# Try to find RBC
#
# The following variables are optionally searched for defaults:
#   RBC_ROOT         - Root directory of RBC source tree
#
# Once done, this will define:
#   RBC_FOUND        - system has RBC
#   RBC_INCLUDE_DIR  - the RBC include directory
#   RBC_LIBRARIES    - link these to use RBC

find_path ( 
    RBC_INCLUDE_DIR
    NAMES RBC/common.hpp RBC/data_types.hpp RBC/algorithms.hpp RBC/tests/helper_funcs.hpp 
    HINTS ${RBC_ROOT}/include 
          /usr/include
          /usr/local/include 
    DOC "The directory where RBC headers reside"
)

find_library ( 
    RBC_LIB_ALGORITHMS 
    NAMES RBCAlgorithms 
    PATHS ${RBC_ROOT}/build/lib 
          /usr/lib/RBC 
          /usr/local/lib/RBC 
    DOC "The RBC algorithms library"
)

find_library ( 
    RBC_LIB_HELPERFUNCS 
    NAMES RBCHelperFuncs 
    PATHS ${RBC_ROOT}/build/lib
          ${RBC_ROOT}/../RBC-build/lib 
          /usr/lib/RBC 
          /usr/local/lib/RBC 
    DOC "The RBC helper functions library"
)

include ( FindPackageHandleStandardArgs )

find_package_handle_standard_args ( 
    RBC 
    FOUND_VAR RBC_FOUND
    REQUIRED_VARS RBC_INCLUDE_DIR RBC_LIB_ALGORITHMS RBC_LIB_HELPERFUNCS 
)

if ( RBC_FOUND )
    set ( RBC_LIBRARIES ${RBC_LIB_ALGORITHMS} ${RBC_LIB_HELPERFUNCS} )
    message ( STATUS "Found RBC:" )
    message ( STATUS " - Includes: ${RBC_INCLUDE_DIR}" )
    message ( STATUS " - Libraries: ${RBC_LIBRARIES}" )
else ( RBC_FOUND )
    set ( RBC_LIBRARIES )
endif ( RBC_FOUND )

mark_as_advanced ( 
    RBC_INCLUDE_DIR
    RBC_LIB_ALGORITHMS 
    RBC_LIB_HELPERFUNCS 
)
