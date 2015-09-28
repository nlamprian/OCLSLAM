# Try to find octomap
#
# The following variables are optionally searched for defaults:
#   OCTOMAP_ROOT         - Root directory of octomap source tree
#
# Once done, this will define:
#   OCTOMAP_FOUND        - system has octomap
#   OCTOMAP_INCLUDE_DIR  - the octomap include directory
#   OCTOMAP_LIB_DIR      - the octomap lib directory
#   OCTOMAP_LIBRARIES    - link these to use octomap

find_path ( 
    OCTOMAP_INCLUDE_DIR
    NAMES octomap/AbstractOccupancyOcTree.h octomap/AbstractOcTree.h octomap/ColorOcTree.h octomap/CountingOcTree.h octomap/MapCollection.h octomap/MapCollection.hxx octomap/MapNode.h octomap/MapNode.hxx octomap/math/Pose6D.h octomap/math/Quaternion.h octomap/math/Utils.h octomap/math/Vector3.h octomap/MCTables.h octomap/OccupancyOcTreeBase.h octomap/OccupancyOcTreeBase.hxx octomap/octomap_deprecated.h octomap/octomap.h octomap/octomap_timing.h octomap/octomap_types.h octomap/octomap_utils.h octomap/OcTreeBase.h octomap/OcTreeBaseImpl.h octomap/OcTreeBaseImpl.hxx octomap/OcTreeBaseSE.h octomap/OcTreeBaseSE.hxx octomap/OcTreeDataNode.h octomap/OcTreeDataNode.hxx octomap/OcTree.h octomap/OcTreeIterator.hxx octomap/OcTreeKey.h octomap/OcTreeLUTdefs.h octomap/OcTreeLUT.h octomap/OcTreeNode.h octomap/OcTreeStamped.h octomap/Pointcloud.h octomap/ScanGraph.h 
    HINTS ${OCTOMAP_ROOT}/octomap/include 
          /opt/ros/jade/include 
          /opt/ros/indigo/include 
          /opt/ros/hydro/include 
          /usr/include 
          /usr/local/include 
    DOC "The directory where octomap headers reside"
)

find_path ( 
    OCTOMAP_LIB_DIR
    NAMES liboctomap.a liboctomap.so liboctomath.a liboctomath.so 
    HINTS ${OCTOMAP_ROOT}/lib 
          /opt/ros/jade/lib 
          /opt/ros/indigo/lib 
          /opt/ros/hydro/lib 
          /usr/lib 
          /usr/local/lib 
    DOC "The directory where octomap libraries reside"
)

find_library ( 
    OCTOMAP_LIB_OCTOMAP 
    NAMES octomap
    PATHS ${OCTOMAP_ROOT}/lib 
          /opt/ros/jade/lib 
          /opt/ros/indigo/lib 
          /opt/ros/hydro/lib 
          /usr/lib 
          /usr/local/lib 
    DOC "The octomap library"
)

find_library ( 
    OCTOMAP_LIB_OCTOMATH 
    NAMES octomath
    PATHS ${OCTOMAP_ROOT}/lib 
          /opt/ros/jade/lib 
          /opt/ros/indigo/lib 
          /opt/ros/hydro/lib 
          /usr/lib 
          /usr/local/lib 
    DOC "The octomath library"
)

include ( FindPackageHandleStandardArgs )

find_package_handle_standard_args ( 
    octomap 
    FOUND_VAR OCTOMAP_FOUND
    REQUIRED_VARS OCTOMAP_INCLUDE_DIR OCTOMAP_LIB_DIR OCTOMAP_LIB_OCTOMAP OCTOMAP_LIB_OCTOMATH 
)

if ( OCTOMAP_FOUND )
    set ( OCTOMAP_LIBRARIES ${OCTOMAP_LIB_OCTOMAP} ${OCTOMAP_LIB_OCTOMATH} )
    message ( STATUS "Found octomap:" )
    message ( STATUS " - Includes: ${OCTOMAP_INCLUDE_DIR}" )
    message ( STATUS " - Libraries: ${OCTOMAP_LIBRARIES}" )
else ( OCTOMAP_FOUND )
    set ( OCTOMAP_LIBRARIES )
endif ( OCTOMAP_FOUND )

mark_as_advanced ( 
    OCTOMAP_INCLUDE_DIR
    OCTOMAP_LIB_DIR
    OCTOMAP_LIB_OCTOMAP 
    OCTOMAP_LIB_OCTOMATH 
)
