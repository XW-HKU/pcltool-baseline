#include "nerf_pre.h"
#include "tinyxml.h"
// #include "tinystr.h"
#include <boost/filesystem.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <math.h>
// #include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <filesystem>
#include <pcl/filters/statistical_outlier_removal.h>

#include "lx_tools.h"

std::map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> NERF_PRE::run(std::string lx_file_name, float downsample_size, std::vector<Eigen::Matrix4d> &pose_list)
{
    std::map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> temp;
    printf("start offline render develop.\n");
    auto start_t = clock();
    std::vector<pcl::PointXYZRGBL> pclPointsVec;
    std::vector<PointXYZRGBI>      pointsVec;
    uint32_t                       numberOfIntensity;
    // CamParams<double> camParams_;
    std::vector<Eigen::Matrix4d> cam_pose;

    float voxelSize = 0.05f;
    float voxelSizeHalf = voxelSize / 2.0f;
    // get dir name using std's std::string

    lx_file_name_ = lx_file_name; // "/home/xw/XW/Bags/Panora/2023-08-22-10-41-46/MANIFOLD_2023-08-22-10-41-46.lx";
    // std::string lx_file_name_ = "/home/xw/XW/Bags/Panora/Panora_0414_stjohn_raw/RawData/2023-04-15-04-48-2floor3room/MANIFOLD_2023-04-15-04-48-3ro.lx";
    // std::string lx_file_name_ = "/home/xw/XW/Bags/Panora/2023-06-16-11-42-07/MANIFOLD_2023-06-16-11-42-07.lx";

    auto readStart = clock();
    LxTools lxreader;
    bool bf = lxreader.readSumFile(lx_file_name_, pclPointsVec, pointsVec, cam_pose, numberOfIntensity);
    // while (cam_pose.size() > 720)
    {
        // cam_pose.pop_back();
        cam_pose.pop_back();
    }

    if (!bf) {
        std::cout << "readSumFile failed" << std::endl;
        return temp;
    }
    printf ("LX points contains: %d, used time: %0.4lf s\n", pointsVec.size(), (double)(clock() - readStart) / (CLOCKS_PER_SEC));
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGBL>);
    cloud->points.reserve(pointsVec.size());
    for (int i = 0; i < pclPointsVec.size(); ++i)
    {
        // if (abs(pclPointsVec[i].x) > 13.0 && abs(pclPointsVec[i].x) < 22.0 && (pclPointsVec[i].y) > 5.0 && abs(pclPointsVec[i].z) < 2.0)
        if (pclPointsVec[i].label < cam_pose.size())
            cloud->points.push_back(pclPointsVec[i]);
            cloud_->points.push_back(pclPointsVec[i]);
    }
    unsigned int cloud_num = cloud->points.size();
    start_t = clock();printf("start downsample the cloud, downsample size %f...\n", downsample_size);
    
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBL>);
    double ds_size = downsample_size;
    VoxelDownsampleFilter ds_filter(ds_size);
    
    // ds_filter.filt(cloud, cloud_filtered);
    *cloud_filtered = *cloud;
    
    unsigned int ds_num = cloud_filtered->points.size();
    printf("downsample the cloud size from %d to %d used time: %0.4lf s\n", cloud_num, ds_num, (double)(clock() - start_t) / (CLOCKS_PER_SEC));
    // cloud_filtered->clear();
    // *cloud_filtered = *cloud_;
    std::cout<<"cloud_ size "<<cloud->points.size()<<std::endl;
    std::cout<<"cloud_filtered size "<<cloud_filtered->points.size()<<std::endl;
    std::map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> map;
    for(auto const &point : cloud_filtered->points)
    {
        int label = point.label;
        if (map.find(label) == map.end())
        {
            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_frame(new pcl::PointCloud<pcl::PointXYZRGBL>);
            map[label] = cloud_frame;
            map[label]->push_back(point);
        }
        map[label]->push_back(point);
    }
    std::vector<Eigen::Vector3d> poe_list;
    pose_list = cam_pose;
    std::cout<<"read lx done"<<std::endl;
    return map;

}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr NERF_PRE::run2(std::string lx_file_name, std::vector<Eigen::Matrix4d> &pose_list)
{
    std::map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> temp;
    printf("start offline render develop.\n");
    auto start_t = clock();
    std::vector<pcl::PointXYZRGBL> pclPointsVec;
    std::vector<PointXYZRGBI>      pointsVec;
    uint32_t                       numberOfIntensity;
    // CamParams<double> camParams_;
    std::vector<Eigen::Matrix4d> cam_pose;

    float voxelSize = 0.05f;
    float voxelSizeHalf = voxelSize / 2.0f;
    // get dir name using std's std::string

    lx_file_name_ = lx_file_name; // "/home/xw/XW/Bags/Panora/2023-08-22-10-41-46/MANIFOLD_2023-08-22-10-41-46.lx";
    // std::string lx_file_name_ = "/home/xw/XW/Bags/Panora/Panora_0414_stjohn_raw/RawData/2023-04-15-04-48-2floor3room/MANIFOLD_2023-04-15-04-48-3ro.lx";
    // std::string lx_file_name_ = "/home/xw/XW/Bags/Panora/2023-06-16-11-42-07/MANIFOLD_2023-06-16-11-42-07.lx";

    auto readStart = clock();
    LxTools lxreader;
    bool bf = lxreader.readSumFile(lx_file_name_, pclPointsVec, pointsVec, cam_pose, numberOfIntensity);
    // while (cam_pose.size() > 720)
    {
        // cam_pose.pop_back();
        cam_pose.pop_back();
    }

    if (!bf) {
        std::cout << "readSumFile failed" << std::endl;
        // return temp;
    }
    printf ("LX points contains: %d, used time: %0.4lf s\n", pointsVec.size(), (double)(clock() - readStart) / (CLOCKS_PER_SEC));
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGBL>);
    cloud->points.reserve(pointsVec.size());
    for (int i = 0; i < pclPointsVec.size(); ++i)
    {
        // if (abs(pclPointsVec[i].x) > 13.0 && abs(pclPointsVec[i].x) < 22.0 && (pclPointsVec[i].y) > 5.0 && abs(pclPointsVec[i].z) < 2.0)
        if (pclPointsVec[i].label < cam_pose.size())
            cloud->points.push_back(pclPointsVec[i]);
            cloud_->points.push_back(pclPointsVec[i]);
    }
    unsigned int cloud_num = cloud->points.size();
    // start_t = clock();printf("start downsample the cloud, downsample size %f...\n", downsample_size);
    
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBL>);
    // double ds_size = downsample_size;
    // VoxelDownsampleFilter ds_filter(ds_size);
    
    // ds_filter.filt(cloud, cloud_filtered);
    *cloud_filtered = *cloud;
    
    unsigned int ds_num = cloud_filtered->points.size();
    printf("downsample the cloud size from %d to %d used time: %0.4lf s\n", cloud_num, ds_num, (double)(clock() - start_t) / (CLOCKS_PER_SEC));
    // cloud_filtered->clear();
    // *cloud_filtered = *cloud_;
    std::cout<<"cloud_ size "<<cloud->points.size()<<std::endl;
    std::cout<<"cloud_filtered size "<<cloud_filtered->points.size()<<std::endl;
    // std::map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> map;
    // for(auto const &point : cloud_filtered->points)
    // {
    //     int label = point.label;
    //     if (map.find(label) == map.end())
    //     {
    //         pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_frame(new pcl::PointCloud<pcl::PointXYZRGBL>);
    //         map[label] = cloud_frame;
    //         map[label]->push_back(point);
    //     }
    //     map[label]->push_back(point);
    // }
    pose_list = cam_pose;
    return cloud_filtered;

}
