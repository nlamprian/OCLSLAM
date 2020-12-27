include ( ExternalProject )

set ( GuidedFilter_TARGET GuidedFilter PARENT_SCOPE )

ExternalProject_Add ( 
    GuidedFilter DEPENDS ${CLUtils_TARGET} 
    GIT_REPOSITORY https://github.com/nlamprian/GuidedFilter.git
    PREFIX ${CMAKE_CURRENT_BINARY_DIR} 
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCLUtils_ROOT=${CLUtils_ROOT} # -DBUILD_TESTS=ON 
    INSTALL_COMMAND "" 
    STEP_TARGETS COPY_KERNELS 
)

ExternalProject_Get_Property ( GuidedFilter SOURCE_DIR )
ExternalProject_Get_Property ( GuidedFilter BINARY_DIR )

ExternalProject_Add_Step (
    GuidedFilter COPY_KERNELS 
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${SOURCE_DIR}/kernels/ ${PROJECT_BINARY_DIR}/kernels/GuidedFilter 
    COMMAND ${CMAKE_COMMAND} -E remove ${PROJECT_BINARY_DIR}/kernels/GuidedFilter/CMakeLists.txt 
    COMMENT "Copying GuidedFilter kernels to ${PROJECT_BINARY_DIR}/kernels/GuidedFilter"
    DEPENDEES download 
)

set ( GuidedFilter_INCLUDE_DIR ${SOURCE_DIR}/include PARENT_SCOPE )
set ( GuidedFilter_LIBS_DIR ${BINARY_DIR}/lib PARENT_SCOPE )

set ( 
    GuidedFilter_LIBRARIES 
    ${BINARY_DIR}/lib/libGFAlgorithms.a 
    ${BINARY_DIR}/lib/libGFMath.a 
    ${BINARY_DIR}/lib/libGFHelperFuncs.a 
    PARENT_SCOPE 
)

set ( GuidedFilter_FOUND TRUE PARENT_SCOPE )

set ( GuidedFilter_ROOT ${SOURCE_DIR} PARENT_SCOPE )

install ( SCRIPT ${BINARY_DIR}/cmake_install.cmake )
