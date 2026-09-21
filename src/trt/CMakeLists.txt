cmake_minimum_required(VERSION 3.0.2)
project(trt)
include_directories(/usr/local/cuda-11.4/include)
link_directories(/usr/local/cuda-11.4/targets/aarch64-linux/lib)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find catkin and ROS packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

find_package(Eigen3 REQUIRED)

# Optionally find CUDA and TensorRT
find_package(CUDA)
find_library(NVINFER_LIB nvinfer)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES trt_core
    CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

if(CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})
endif()

## Build a small library with the TrtManager + logger so multiple executables can link it
add_library(trt_core
    src/trt_manager.cpp
    src/trt_logger.cpp
)

target_include_directories(trt_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(trt_core PUBLIC ${catkin_LIBRARIES})

if(CUDA_FOUND)
    target_link_libraries(trt_core PUBLIC ${CUDA_LIBRARIES})
endif()

if(NVINFER_LIB)
    target_link_libraries(trt_core PUBLIC ${NVINFER_LIB})
else()
    message(WARNING "TensorRT library 'nvinfer' not found. You may need to install TensorRT or set NVINFER_LIB path.")
endif()

if(NOT CUDA_FOUND)
    message(WARNING "CUDA not found via find_package(CUDA). Ensure CUDA is installed if you need GPU support.")
endif()

## Note: standalone executables `unified_trt_node` and `trt_warmup_node`
## were removed to keep only the shared `trt_core` library. If you
## need to restore them later, add the corresponding `add_executable`
## blocks back and link against `trt_core`.