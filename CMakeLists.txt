cmake_minimum_required(VERSION 3.10)

project(FRCSysID LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-Wno-subobject-linkage)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "GTSAM package found at: ${GTSAM_DIR}")
find_package(Python3 COMPONENTS NumPy)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${GTSAM_INCLUDE_DIRS})

file(GLOB_RECURSE SRC_FILES
  ${PROJECT_SOURCE_DIR}/src/*.cc
)

file(GLOB_RECURSE HEADER_FILES
  ${PROJECT_SOURCE_DIR}/include/*.hh
  ${PROJECT_SOURCE_DIR}/include/*.h
)

add_executable(sysid_app ${SRC_FILES} ${HEADER_FILES})

if(Python3_NumPy_FOUND)
  target_link_libraries(sysid_app INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(sysid_app INTERFACE WITHOUT_NUMPY)
endif()


target_link_libraries(sysid_app PRIVATE
  gtsam
  Python3::Python
)

target_include_directories(sysid_app PRIVATE 
  ${PROJECT_SOURCE_DIR}/include)


