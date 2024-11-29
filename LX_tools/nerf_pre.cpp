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
#include "../SmoothPointCloud/SmoothPointCloud.h"
std::vector<std::string> splitString(const std::string& str, char delimiter) {
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;

  while (std::getline(ss, token, delimiter)) {
    tokens.push_back(token);
  }

  return tokens;
}


std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> NERF_PRE::run(std::string lx_file_name, float downsample_size, std::vector<Eigen::Matrix4d> &pose_list)
{
    std::unordered_map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> temp;
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
    auto split_lx_file_name = splitString(lx_file_name_, '/');

    auto lx_name = split_lx_file_name[split_lx_file_name.size() - 1];
    lx_name = splitString(lx_name, '.')[0]+".ply";
    auto save_path = "/home/mt_eb1/LYX/filter_noise_point/Table/" + lx_name;
    std::cout<<"save path "<<save_path<<std::endl;
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
        std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> A;
        return A;
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
    }
    pcl::io::savePLYFileBinary("/home/mt_eb1/LYX/filter_noise_point/Table/No_Smooth_"+lx_name, *cloud);

    if(pcl::io::loadPLYFile(save_path, *cloud) != -1)
    {
        ;
    }else{
        std::unordered_map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> map;
        for(auto const &point : cloud->points)
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
        std::map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr > make_unordered_map_order;
        for(auto it = map.begin(); it != map.end(); it++)
        {
            make_unordered_map_order[it->first] = it->second;
        }
        std::vector<Eigen::Matrix4d> pose_for_unordered_map;
        std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> pointcloud_list;
        for(auto it = make_unordered_map_order.begin(); it != make_unordered_map_order.end(); it++)
        {
            std::cout<<"label "<<it->first<<std::endl;
            std::cout<<"pointcloud size "<<it->second->points.size()<<std::endl;    
            pose_for_unordered_map.emplace_back(cam_pose[it->first]);
            pointcloud_list.emplace_back(it->second);
        }
        pose_list = pose_for_unordered_map;
        std::cout<<"read lx done"<<std::endl;
        return pointcloud_list;      
    }











    pcl::KdTreeFLANN<PointT> kdtree;float nearest_K_Search = 0.03;float curr_point_weight = 0.1; auto others_weight = 1-curr_point_weight;
    //并行化滤波
    std::mutex mtx;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> smooth_pointcloud_list;
    kdtree.setInputCloud(cloud);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->size()),
    [&] (tbb::blocked_range<size_t> r){
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr smooth_pointcloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
        auto t2 = std::chrono::high_resolution_clock::now();
        for(size_t i = r.begin(); i != r.end(); i++){
            const auto & point = cloud->points[i];

            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;
            if(kdtree.radiusSearch(point, nearest_K_Search, pointIdxNKNSearch, pointNKNSquaredDistance) > 1){
                
                PointT temp_point;
                for(int i = 0; i < pointIdxNKNSearch.size(); i++){
                    temp_point.x += cloud->points[pointIdxNKNSearch[i]].x;
                    temp_point.y += cloud->points[pointIdxNKNSearch[i]].y;
                    temp_point.z += cloud->points[pointIdxNKNSearch[i]].z;

                }
                //均值点
                temp_point.x = temp_point.x/(pointIdxNKNSearch.size());
                temp_point.y = temp_point.y/(pointIdxNKNSearch.size());
                temp_point.z = temp_point.z/(pointIdxNKNSearch.size());


                //加权平均
                temp_point.x = curr_point_weight*point.x + others_weight*temp_point.x;
                temp_point.y = curr_point_weight*point.y + others_weight*temp_point.y;
                temp_point.z = curr_point_weight*point.z + others_weight*temp_point.z;
                temp_point.r = point.r;temp_point.g = point.g;temp_point.b = point.b;temp_point.label = point.label;
                smooth_pointcloud->points.push_back(temp_point);
            }else{
                smooth_pointcloud->points.push_back(point);
            }
        }

        mtx.lock();
        smooth_pointcloud_list.emplace_back(smooth_pointcloud);
        // mtx.unlock();
        auto t3 = std::chrono::high_resolution_clock::now();
        // std::cout<<"r.begin() "<<r.begin()<<" r.end() "<<r.end();
        // std::cout<<" smooth a pointcloud spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t3 - t2).count()<<" ms"<<std::endl;
        mtx.unlock();
    });

    cloud->clear();
    for(auto & pointcloud : smooth_pointcloud_list){
        *cloud += *pointcloud;
    }
    pcl::io::savePLYFileBinary(save_path, *cloud);



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
    // std::cout<<"cloud_ size "<<cloud->points.size()<<std::endl;
    // std::cout<<"cloud_filtered size "<<cloud_filtered->points.size()<<std::endl;
    std::unordered_map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> map;
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
    std::map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr > make_unordered_map_order;
    for(auto it = map.begin(); it != map.end(); it++)
    {
        make_unordered_map_order[it->first] = it->second;
    }
    std::vector<Eigen::Matrix4d> pose_for_unordered_map;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> pointcloud_list;
    for(auto it = make_unordered_map_order.begin(); it != make_unordered_map_order.end(); it++)
    {
        // std::cout<<"label "<<it->first<<std::endl;
        // std::cout<<"pointcloud size "<<it->second->points.size()<<std::endl;    
        pose_for_unordered_map.emplace_back(cam_pose[it->first]);
        pointcloud_list.emplace_back(it->second);
    }
    pose_list = pose_for_unordered_map;
    std::cout<<"read lx done"<<std::endl;
    
    return pointcloud_list;

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
            // cloud_->points.push_back(pclPointsVec[i]);
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
