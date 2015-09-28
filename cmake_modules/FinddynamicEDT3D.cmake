# Try to find dynamicEDT3D
#
# The following variables are optionally searched for defaults:
#   DYNAMICEDT3D_ROOT         - Root directory of dynamicEDT3D source tree
#
# Once done, this will define:
#   DYNAMICEDT3D_FOUND        - system has dynamicEDT3D
#   DYNAMICEDT3D_INCLUDE_DIR  - the dynamicEDT3D include directory
#   DYNAMICEDT3D_LIB_DIR      - the dynamicEDT3D lib directory
#   DYNAMICEDT3D_LIBRARIES    - link these to use dynamicEDT3D

find_path ( 
    DYNAMICEDT3D_INCLUDE_DIR
    NAMES dynamicEDT3D/bucketedqueue.h dynamicEDT3D/bucketedqueue.hxx dynamicEDT3D/dynamicEDT3D.h dynamicEDT3D/dynamicEDTOctomap.h dynamicEDT3D/point.h 
    HINTS ${DYNAMICEDT3D_ROOT}/dynamicEDT3D/include 
          /usr/include 
          /usr/local/include 
    DOC "The directory where dynamicEDT3D headers reside"
)

find_path ( 
    DYNAMICEDT3D_LIB_DIR
    NAMES libdynamicedt3d.a libdynamicedt3d.so
    HINTS ${DYNAMICEDT3D_ROOT}/lib 
          /usr/lib 
          /usr/local/lib 
    DOC "The directory where dynamicEDT3D libraries reside"
)

find_library ( 
    DYNAMICEDT3D_LIB_DYNAMICEDT3D 
    NAMES dynamicedt3d 
    PATHS ${DYNAMICEDT3D_ROOT}/lib 
          /usr/lib 
          /usr/local/lib 
    DOC "The dynamicEDT3D library"
)

include ( FindPackageHandleStandardArgs )

find_package_handle_standard_args ( 
    dynamicEDT3D 
    FOUND_VAR DYNAMICEDT3D_FOUND
    REQUIRED_VARS DYNAMICEDT3D_INCLUDE_DIR DYNAMICEDT3D_LIB_DIR DYNAMICEDT3D_LIB_DYNAMICEDT3D 
)

if ( DYNAMICEDT3D_FOUND )
    set ( DYNAMICEDT3D_LIBRARIES ${DYNAMICEDT3D_LIB_DYNAMICEDT3D} )
    message ( STATUS "Found dynamicEDT3D:" )
    message ( STATUS " - Includes: ${DYNAMICEDT3D_INCLUDE_DIR}" )
    message ( STATUS " - Libraries: ${DYNAMICEDT3D_LIBRARIES}" )
else ( DYNAMICEDT3D_FOUND )
    set ( DYNAMICEDT3D_LIBRARIES )
endif ( DYNAMICEDT3D_FOUND )

mark_as_advanced ( 
    DYNAMICEDT3D_INCLUDE_DIR
    DYNAMICEDT3D_LIB_DIR
    DYNAMICEDT3D_LIB_DYNAMICEDT3D 
)
