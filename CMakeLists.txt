cmake_minimum_required(VERSION 3.10)
project(PCLTool)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED)
message(STATUS "TBB: ${TBB_INCLUDE_DIRS}")
message(STATUS "TBB: ${TBB_DIR}")
message(STATUS "TBB: ${TBB_LIBRARIES}")

include_directories(${PCL_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS} 
                    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "OpenCV: ${OpenCV_LIBRARIES}")
add_executable(voxel_filter voxel_filter.cpp)
target_link_libraries(voxel_filter ${PCL_LIBRARIES})

add_executable(depth_img_generate
               depth_img_generate.cpp
                    LX_tools/nerf_pre.cpp 
                    LX_tools/lx_tool.cpp
                    PointCloudCart2Sph/PointCloudCart2Sph.cpp)
target_link_libraries(depth_img_generate ${PCL_LIBRARIES} ${OpenCV_LIBS} -ltbb -lcrypto)


