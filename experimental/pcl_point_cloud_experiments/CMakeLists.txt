cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(preprocessor)
SET(CMAKE_BUILD_TYPE "Release")
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters features gpu_containers gpu_utils gpu_octree gpu_features cuda_common cuda_features)
find_package(Boost COMPONENTS system filesystem REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(preprocessor preprocessor.cpp)
target_link_libraries(preprocessor ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_GPU_CONTAINERS_LIBRARIES} ${PCL_GPU_UTILS_LIBRARIES} ${PCL_GPU_OCTREE_LIBRARIES} ${PCL_GPU_FEATURES_LIBRARIES} ${PCL_CUDA_COMMON_LIBRARIES} ${PCL_CUDA_FEATURES_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

