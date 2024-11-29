#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>
#include "../PointCloudCart2Sph/PointCloudCart2Sph.h"
class SmoothPointCloud
{
    public:
        void smooth_pointcloud(std::vector<pcl::PointCloud<PointT>::Ptr> &pointclouds_list);
        
};