cmake_minimum_required(VERSION 3.2)

# local project name
project (viewer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#https://github.com/copperspice/copperspice/issues/82
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

# Build flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -s")
message(STATUS "CMAKE_C_FLAGS=${CMAKE_C_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s")
message(STATUS "CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")

# find dependencies
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)

include_directories(${OpenCV_INCLUDE_DIRS})

set(CISTOF_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcistof.so)

set(CAMMETADATA_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcamera_metadata.so.0)

add_executable(${PROJECT_NAME} src/viewer.cpp)

target_link_libraries(${PROJECT_NAME} ${CISTOF_LIB} ${CMAKE_DL_LIBS})
target_link_libraries(${PROJECT_NAME} ${CAMMETADATA_LIB} ${CMAKE_DL_LIBS})

target_link_libraries(${PROJECT_NAME} pthread)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
