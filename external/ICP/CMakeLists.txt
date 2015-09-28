include ( ExternalProject )

set ( ICP_TARGET ICP PARENT_SCOPE )

ExternalProject_Add ( 
    ICP DEPENDS ${CLUtils_TARGET} ${GuidedFilter_TARGET} ${RBC_TARGET} ${EIGEN_TARGET} 
    GIT_REPOSITORY https://github.com/pAIgn10/ICP.git 
    PREFIX ${CMAKE_CURRENT_BINARY_DIR} 
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCLUtils_ROOT=${CLUtils_ROOT} -DGuidedFilter_ROOT=${GuidedFilter_ROOT} -DRBC_ROOT=${RBC_ROOT} -DEIGEN_ROOT=${EIGEN_ROOT} # -DBUILD_TESTS=ON 
    INSTALL_COMMAND "" 
    STEP_TARGETS COPY_KERNELS 
)

ExternalProject_Get_Property ( ICP SOURCE_DIR )
ExternalProject_Get_Property ( ICP BINARY_DIR )

ExternalProject_Add_Step (
    ICP COPY_KERNELS 
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${SOURCE_DIR}/kernels/ ${PROJECT_BINARY_DIR}/kernels/ICP 
    COMMAND ${CMAKE_COMMAND} -E remove ${PROJECT_BINARY_DIR}/kernels/ICP/CMakeLists.txt 
    COMMENT "Copying ICP kernels to ${PROJECT_BINARY_DIR}/kernels/ICP"
    DEPENDEES download 
)

set ( ICP_INCLUDE_DIR ${SOURCE_DIR}/include PARENT_SCOPE )
set ( ICP_LIBS_DIR ${BINARY_DIR}/lib PARENT_SCOPE )

set ( 
    ICP_LIBRARIES 
    ${BINARY_DIR}/lib/libICPAlgorithms.a 
    ${BINARY_DIR}/lib/libICPHelperFuncs.a 
    PARENT_SCOPE 
)

set ( ICP_FOUND TRUE PARENT_SCOPE )

set ( ICP_ROOT ${SOURCE_DIR} PARENT_SCOPE )

install ( SCRIPT ${BINARY_DIR}/cmake_install.cmake )
