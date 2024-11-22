#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp> 
#include <filesystem>
#include <execution>
#include <vector>
#include <numeric> // for std::iota
#include <eigen3/Eigen/Dense>
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
            int theta_index;
            int phi_index;
            std::vector<pointXYZRGBHM* > points;
            Sphgrid(int theta_index_, int phi_index_):theta_index(theta_index_), phi_index(phi_index_){}    
            bool operator==(const Sphgrid& other) const {
                return theta_index == other.theta_index && phi_index == other.phi_index;
                }
            float depth;
        };
        
        void compute_Sphgrid_depth(std::vector<PointCloudCart2Sph::Sphgrid> &sphgrid);
        // cv::Mat depth_image_show = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
        cv::Mat get_depth_image_show(std::vector<PointCloudCart2Sph::Sphgrid> &sphgrid);
        std::unordered_map<int, std::string> umap;
        PointCloudCart2Sph(pcl::PointCloud<PointT>::Ptr &pointcloud_, Eigen::Matrix4f &pose_matrix_, float theta_resolution_, float phi_resolution_){
            std::cout<<"pointcloud_cart2sph init"<<std::endl;
            theta_resolution = theta_resolution_;
            phi_resolution = phi_resolution_;
            pose_matrix = pose_matrix_;
            XAxis = pose_matrix.block<3,1>(0,0);
            YAxis = pose_matrix.block<3,1>(0,1);
            ZAxis = pose_matrix.block<3,1>(0,2);
            displacement = pose_matrix.block<3,1>(0,3);
            ori_pointcloud = pointcloud_;
            pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
            std::cout<<"pointcloud_ size "<<pointcloud_->points.size()<<std::endl;
            pcl::transformPointCloud(*ori_pointcloud, *out, pose_matrix);
            // pointcloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
            // *pointcloud = *pointcloud_;
            for(auto & point : out->points){
                pointcloud.emplace_back(pointXYZRGBHM(point));
            }

            theta_size = 2*M_PI/theta_resolution+eps;
            phi_size = M_PI/phi_resolution+eps;
            std::cout<<"pointcloud_ size "<<pointcloud_->points.size()<<std::endl;
            std::cout<<"test_pointcloud_ "<<pointcloud.size()<<std::endl;
        }
        
        std::vector<Sphgrid> local_pointcloud_sphgrid();
        std::vector<Sphgrid> curr_frame_pointcloud_sphgrid();
        int index_in_vector_of_sphgrid(const float &theta, const float &phi){
            int theta_index = theta/theta_resolution+eps;
            int phi_index = phi/phi_resolution+eps;
            return phi_index*theta_size + theta_index;
        }
        float get_point_in_sph_theta(const pointXYZRGBHM& point);
        float get_point_in_sph_phi(const pointXYZRGBHM& point);
        float get_point_in_sph_r(const pointXYZRGBHM& point);

        void  update_pointcloud(const Eigen::Matrix4f &relative_pose){
            for(auto & point : pointcloud){
                point.xyz = relative_pose*point.xyz;
                point.xyz.w() = 1;
            }
        }
        void  update_pointcloud2(const Eigen::Matrix4f &curr_pose){
            pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
            pcl::transformPointCloud(*ori_pointcloud, *out, curr_pose);
            if(out->points.size() != pointcloud.size()){
                std::cout<<"pointcloud size not equal"<<std::endl;
            }
            for(int i=0;i<out->points.size();i++){
                pointcloud[i].xyz = Eigen::Vector4f(out->points[i].x, out->points[i].y, out->points[i].z, 1);
            }
        }

        void get_curr_frame_pointcloud(pcl::PointCloud<PointT>::Ptr curr_frame,const Eigen::Matrix4f &curr_pose);

        void run(int picture_num);
        void delete_virual_pointcloud();
        std::vector<pcl::PointCloud<PointT>::Ptr> get_result_pointcloud();
        void high_delete_virual_pointcloud();
        std::vector<pcl::PointCloud<PointT>::Ptr> get_result_pointcloud2();
    private:
        std::vector<pointXYZRGBHM> pointcloud;//已变换到curr_frame坐标系下
        std::vector<pointXYZRGBHM> curr_frame_pointcloud;
        std::vector<pointXYZRGBHM> filter_pointcloud;
        pcl::PointCloud<PointT>::Ptr ori_pointcloud;

        Eigen::Matrix4f pose_matrix;
        Eigen::Vector3f XAxis;
        Eigen::Vector3f YAxis;
        Eigen::Vector3f ZAxis;
        Eigen::Vector3f displacement;
        float theta_resolution;
        float phi_resolution;
        int theta_size;
        int phi_size;
};