#pragma once
#ifndef PCL_TYPE_BASE_H
#define PCL_TYPE_BASE_H
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/pcl_macros.h>
// #include <opencv2/core/hal/interface.h>

#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
struct EIGEN_ALIGN16 PointXYZRGBHM
{
	PCL_ADD_POINT4D;  // 添加XYZ
	unsigned char r;      // 添加RGB
	unsigned char g;
	unsigned char b;
    int hit;      // 添加LABLE
	int miss;// 添加强度

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // 确保新的分配器内存是对齐的// make sure our new allocators are aligned
};                    

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBHM,
(float,x,x)
(float,y,y)
(float,z,z)
(unsigned char, b, blue)
(unsigned char, g, green)
(unsigned char, r, red)
(int, hit, scalar_hit)
(int ,miss, scalar_miss)

)

struct EIGEN_ALIGN16 PointXYZRGBHM
{
	PCL_ADD_POINT4D;  // 添加XYZ
	int hit;// 添加强度
	int miss;      // 添加LABLE
	unsigned char b;
	unsigned char g;
	unsigned char r;
    inline PointXYZRGBHM (const PointXYZRGBHM &p)
    {
      x = p.x; y = p.y; z = p.z; r=p.r;g=p.g;b=p.b;data[3] = 1.0f;
	  hit = p.hit;miss=p.miss;
    }

    inline PointXYZRGBHM ()
    {
      x = y = z  =0.0f;
	  hit = 0;miss=0;
      data[3] = 1.0f;
    }

    inline PointXYZRGBHM (float _x, float _y, float _z, int _hit, int _miss)
	{
	  x = _x; y = _y; z = _z; miss = _miss;hit =_hit;
	  data[3] = 1.0f;
	}
    friend std::ostream& operator << (std::ostream& os, const PointXYZRGBHM& p);
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // 确保新的分配器内存是对齐的// make sure our new allocators are aligned
};                    

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBHM,
(float,x,x)
(float,y,y)
(float,z,z)
(unsigned char, b, blue)
(unsigned char, g, green)
(unsigned char, r, red)
(int, hit, scalar_hit)
(int ,miss, scalar_miss)
)

#endif
