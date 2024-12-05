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
            float x = 999999;
            float y = 999999;
            float z = 999999;
            //test
            std::vector<pointXYZRGBHM* > points;
            Sphgrid(int index_):index(index_){}    
            bool operator==(const Sphgrid& other) const {
                return index == other.index ;
                }
            float bar_depth=0;
            float max_depth=-1;
            float min_depth=999999;

            PointT mindepth_point;
        };

        PointCloudCart2Sph(std::vector<pcl::PointCloud<PointT>::Ptr> &pointcloud_list_, std::vector<Eigen::Matrix4f> &pose_matrix_, float theta_resolution_, float phi_resolution_){
            ori_pointcloud_list = pointcloud_list_;pose_matrix = pose_matrix_;theta_resolution = theta_resolution_;phi_resolution = phi_resolution_;
            theta_size = 2*M_PI/theta_resolution+eps;phi_size = M_PI/phi_resolution+eps;
            ori_pointcloudIndex_pointIndex_status.resize(ori_pointcloud_list.size());
            // ori_pointcloudIndex_pointIndex_status_automic.resize(ori_pointcloud_list.size());
            for(int ori_pointcloud_index =0 ;ori_pointcloud_index<ori_pointcloud_list.size();ori_pointcloud_index++){
                int ori_pointcloud_size = ori_pointcloud_list[ori_pointcloud_index]->points.size();
                std::cout<<ori_pointcloud_index<<std::endl;
                ori_pointcloudIndex_pointIndex_status[ori_pointcloud_index].resize(ori_pointcloud_size, 0); 
                // ori_pointcloudIndex_pointIndex_status_automic[ori_pointcloud_index].resize(ori_pointcloud_size);
                // for(int ori_point_index = 0;ori_point_index < ori_pointcloud_size;ori_point_index++){
                //     std::atomic<int> status(0);
                //     ori_pointcloudIndex_pointIndex_status[ori_pointcloud_index].push_back(0);
                //     // ori_pointcloudIndex_pointIndex_status_automic[ori_pointcloud_index].emplace_back(&status);
                // }
            }
            std::cout<<"finish ori_pointcloudIndex_pointIndex_status_automic"<<std::endl;
            float phi_resolution_degree = phi_resolution*180/M_PI; //-7°~52° mid_360
            min_phi = (90-47)/phi_resolution_degree;
            max_phi = (90+2)/phi_resolution_degree;

            pcl::PointCloud<PointT>::Ptr PointInSphGrid_(new pcl::PointCloud<PointT>);
            PointInSphGrid = PointInSphGrid_;

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

        std::vector<Sphgrid> curr_frame_pointcloud_sphgrid(pcl::PointCloud<PointT>::Ptr& curr_frame, int pose_num, float& max_deep);
        // cv::Mat PointCloudCart2Sph::get_depth_image_show();
        void get_depth_image_show();
        void delete_virual_pointcloud();



        void save_pointcloud();

    private:
        std::vector<pcl::PointCloud<PointT>::Ptr> ori_pointcloud_list;
        std::vector<std::vector<int>> ori_pointcloudIndex_pointIndex_status;
        // std::vector<std::vector<std::atomic<int>* >> ori_pointcloudIndex_pointIndex_status_automic;
        std::vector<Eigen::Matrix4f> pose_matrix;
        Eigen::Matrix4f recover_pose;
        Eigen::Matrix4f LastPose=Eigen::Matrix4f::Identity();
        float theta_resolution;
        float phi_resolution;
        int theta_size;
        int phi_size;
        int min_phi;
        int max_phi;
        PointT local_pose;
        ///home/mt_eb1/LYX/DataFromServe/HongKongData/MT20241129-100544 /home/mt_eb1/LYX/filter_noise_point/Table
        std::string base_path = "/home/mt_eb1/LYX/filter_noise_point/Table";
        std::string img_save_path= base_path;std::string img_save_file = "/curr_frame_depth/"; 
        std::string pointcloud_save_path= base_path;std::string pointcloud_save_file="/out/";
        int process_num = 0;
        
        int get_testpoint_sphindex(PointT& test_point, int& pose_num);
        void obvious_point(int test_point_sphindex, PointT local_MapPoint, int& pose_num);//
        pcl::PointCloud<PointT>::Ptr PointInSphGrid;
        
};

struct SphTools{
    float ComputeAngle(const PointT& p0,const PointT& p1){

        float dot = p0.x*p1.x + p0.y*p1.y + p0.z*p1.z;
        float len0 = sqrt(p0.x*p0.x + p0.y*p0.y + p0.z*p0.z);
        float len1 = sqrt(p1.x*p1.x + p1.y*p1.y + p1.z*p1.z);
        // float angle = acos(dot/(len0*len1));
        return acos(dot/(len0*len1));
    }
};