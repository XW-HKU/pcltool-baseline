#ifndef NERF_PRE_H
#define NERF_PRE_H

#include <iostream>
#include <time.h>
#include <thread>
#include <fstream>
#include <map>
#include <unordered_map>

// #include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

// #include "poly_projection.h"

class VoxelDownsampleFilter
{
    public:
    struct MyHash {
        template <typename T>
        std::size_t operator()(const Eigen::Matrix<T,2,1> & vec2) const {
            size_t && hash_seed = (vec2(0)) + 0x9e3779b9;
            return (hash_seed ^ ((vec2(1)) + 0x9e3779b9 + (hash_seed << 6) + (hash_seed >> 2)));
        }

        template <typename T>
        std::size_t operator()(const Eigen::Matrix<T,3,1> & vec3) const {
            size_t && hash_seed = vec3(0) + 0x9e3779b9;
            for (int i = 1; i < 3; i++) {
                // auto elem = vec3(i);
                hash_seed ^= vec3(i) + 0x9e3779b9 + (hash_seed << 6) + (hash_seed >> 2);
            }
            return hash_seed;
        }

        // std::size_t operator()(const int * vec3) const {
        //     size_t && hash_seed = vec3[0] + 0x9e3779b9;
        //     for (int i = 1; i < 3; i++) {
        //         // auto elem = vec3(i);
        //         hash_seed ^= vec3[i] + 0x9e3779b9 + (hash_seed << 6) + (hash_seed >> 2);
        //     }
        //     return hash_seed;
        // }

        /****/

        // std::size_t operator()(const EdgeInd & vec2) const {
        //     size_t && hash_seed = (vec2.x_) + 0x9e3779b9;
        //     return (hash_seed ^ ((vec2.y_) + 0x9e3779b9 + (hash_seed << 6) + (hash_seed >> 2)));
        // }

        // std::size_t operator()(const TriInd & vec3) const {
        //     size_t && hash_seed = vec3.x_ + 0x9e3779b9;
        //     hash_seed ^= vec3.y_ + 0x9e3779b9 + (hash_seed << 6) + (hash_seed >> 2);
        //     hash_seed ^= vec3.z_ + 0x9e3779b9 + (hash_seed << 6) + (hash_seed >> 2);
        //     return hash_seed;
        // }
    };

    // Eigen::Vector3d voxel_size3_;
    std::unordered_map<Eigen::Vector3i, bool, MyHash>  data_map_;
    
    VoxelDownsampleFilter(float resolution) : resolution_(resolution) {};

    // template <typename PointT>
    void filt(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_out)
    {
        cloud_out->points.clear();
        cloud_out->points.reserve(cloud_in->points.size());
        // Eigen::Vector3i ref_coord;
        for (auto pt : cloud_in->points)
        {
            // Eigen::Vector3d && ref_coord = pt / resolution_;
            // double ref_x = pt.x / resolution_;
            Eigen::Vector3i voxel_index (round(pt.x / resolution_), round(pt.y / resolution_), round(pt.z / resolution_));
            auto && it = data_map_.find(voxel_index);
            if (it == data_map_.end())
            {
                data_map_.insert(std::make_pair(voxel_index, true));
                cloud_out->points.push_back(pt);
            }
        }
        std::vector<pcl::PointXYZRGBL, Eigen::aligned_allocator<pcl::PointXYZRGBL> > ().swap(cloud_in->points);
        cloud_out->points.shrink_to_fit();
    };
    private:
    double resolution_;
};


class NERF_PRE
{
    public:
    struct CamPosIndScore
    {
        uint32_t cam_pos_ind;
        float score;
        CamPosIndScore (uint32_t cam_pos_ind, float score)
        {
            this->cam_pos_ind = cam_pos_ind;
            this->score = score;
        };
    };
    std::string lx_file_name_;
    std::string root_path_;
    // CamParams<double> camParams_;
    bool need_undistort_ = false;
    Eigen::Matrix4d T_i_l_;
    NERF_PRE()
    {
        T_i_l_ << 1, 0, 0, -0.011,
                0, 1, 0, -0.02329,
                0, 0, 1, 0.04412,
                0, 0, 0, 1;
    };
    
    // 载入Section图像
    // void loadSectionImage(std::string image_path,
    //                     std::map<int, cv::Mat>& left_images,
    //                     std::map<int, cv::Mat>& right_images);
    
    Eigen::Vector3i hsv2rgb(const Eigen::Vector3f &hsv);
    Eigen::Vector3f rgb2hsv(const Eigen::Vector3i &rgb);
    void showPointCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud);
    // void reprojectpt(const pcl::PointXYZRGBL & point, 
    //                 cv::Mat& image0, 
    //                 cv::Mat& image1,
    //                 Eigen::Matrix4d T_w_i,
    //                 CamParams<double> cam_params,
    //                 pcl::PointXYZRGBL & reprojected_point);
    // int undistort_imgs(double, const unsigned int);
    
    std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> run(std::string lx_file_name, float downsample_size,std::vector<Eigen::Matrix4d> &pose_list);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr run2(std::string lx_file_name,std::vector<Eigen::Matrix4d> &pose_list);
};

#endif