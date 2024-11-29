#pragma once
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp> 
#include <filesystem>
#include <execution>
#include <vector>
#include <numeric> // for std::iota
#include <eigen3/Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <atomic>

typedef pcl::PointXYZRGBL PointT;

struct pointXYZRGBHM{
    Eigen::Vector4f xyz;
    Eigen::Vector3i rgb;
    int hit = 0;
    int miss = 0;
    int selected;
    pointXYZRGBHM(Eigen::Vector4f xyz_, Eigen::Vector3i rgb_, int hit_, int miss_):xyz(xyz_), rgb(rgb_), hit(hit_), miss(miss_){}
    pointXYZRGBHM(){
        xyz = Eigen::Vector4f(0,0,0,1);
        rgb = Eigen::Vector3i(0,0,0);
        hit = 0;
        miss = 0;
    }
    pointXYZRGBHM(PointT point){
        xyz = Eigen::Vector4f(point.x, point.y, point.z, 1);
        rgb = Eigen::Vector3i(point.r, point.g, point.b);
        hit = 0;
        miss = 0;
    }
    bool operator=(const pointXYZRGBHM& other){
        xyz = other.xyz;
        rgb = other.rgb;
        hit = other.hit;
        miss = other.miss;
        return true;
    }
};

class PointCloudCart2Sph{ 
    public:
        double eps = 0.00000001;
        struct Sphgrid{
            bool occupied = false;
            int index = 0;
            int size = 0;
            //test
            float x = 0;
            float y = 0;
            float z = 0;
            //test
            std::vector<pointXYZRGBHM* > points;
            Sphgrid(int index_):index(index_){}    
            bool operator==(const Sphgrid& other) const {
                return index == other.index ;
                }
            float bar_depth=0;
            float max_depth=-1;
            float min_depth=999999;
        };

        PointCloudCart2Sph(std::vector<pcl::PointCloud<PointT>::Ptr> &pointcloud_list_, std::vector<Eigen::Matrix4f> &pose_matrix_, float theta_resolution_, float phi_resolution_){
            ori_pointcloud_list = pointcloud_list_;pose_matrix = pose_matrix_;theta_resolution = theta_resolution_;phi_resolution = phi_resolution_;
            theta_size = 2*M_PI/theta_resolution+eps;phi_size = M_PI/phi_resolution+eps;
            ori_pointcloudIndex_pointIndex_status.resize(ori_pointcloud_list.size());
            for(int ori_pointcloud_index =0 ;ori_pointcloud_index<ori_pointcloud_list.size();ori_pointcloud_index++){
                int ori_pointcloud_size = ori_pointcloud_list[ori_pointcloud_index]->points.size();
                for(int ori_point_index = 0;ori_point_index < ori_pointcloud_size;ori_point_index++){
                    ori_pointcloudIndex_pointIndex_status[ori_pointcloud_index].push_back(0);
                }
            }

        }
        float get_point_in_sph_theta(const PointT& point);
        float get_point_in_sph_phi(const PointT& point);
        float get_point_in_sph_r(const PointT& point); 
        int get_point_in_sph_theta_index(const PointT& point){
            float theta = get_point_in_sph_theta(point);
            return theta/theta_resolution+eps;
        }
        int get_point_in_sph_phi_index(const PointT& point){
            float phi = get_point_in_sph_phi(point);
            return phi/phi_resolution+eps;
        }
        int index_in_vector_of_sphgrid(const float &theta, const float &phi){
            int theta_index = theta/theta_resolution+eps;
            int phi_index = phi/phi_resolution+eps;
            return phi_index*theta_size + theta_index;
        } 

        std::vector<Sphgrid> curr_frame_pointcloud_sphgrid(pcl::PointCloud<PointT>::Ptr& curr_frame);
        // cv::Mat PointCloudCart2Sph::get_depth_image_show();
        void get_depth_image_show();
        void delete_virual_pointcloud();
        void save_pointcloud();

    private:
        std::vector<pcl::PointCloud<PointT>::Ptr> ori_pointcloud_list;
        std::vector<std::vector<int>> ori_pointcloudIndex_pointIndex_status;
        std::vector<Eigen::Matrix4f> pose_matrix;
        Eigen::Matrix4f recover_pose;
        float theta_resolution;
        float phi_resolution;
        int theta_size;
        int phi_size;
};