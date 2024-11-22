#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp> 
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <filesystem>
#include <execution>
#include <vector>
#include <numeric> // for std::iota
#include "LX_tools/nerf_pre.h"
#include "PointCloudCart2Sph/PointCloudCart2Sph.h"

#define MIN_DEPTH 0.5
#define EPS 0.0001


namespace fs = std::filesystem;
float rad_resolution = 0.001;
float height_resolution = 0.001;
float max_height = 2;
float min_height = -1;
struct pointcloud_map_in_cam
{
    // float rad_resolution = 0.2;
    // float height_resolution = 0.02;
    // float max_height = 2;
    // float min_height = -1;
    int Mat_width =  M_PI/rad_resolution;
    int Mat_height = (max_height - min_height)/height_resolution;
    cv::Mat pointcloud_map_in_cam_mat = cv::Mat(Mat_height, Mat_width, CV_32FC1, cv::Scalar(0));
    
};
// {
//     /* data */
// };
int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_cloud_file>" << std::endl;
        return -1;
    }
    std::vector<Eigen::Matrix4f> transform_list;

    // check if the input file is a .lx file
    bool read_lx = false;
    std::string cloud_filename = argv[1];
    if (cloud_filename.find(".lx") != std::string::npos) {
        read_lx = true;
    }
    fs::path cloud_path = fs::path(cloud_filename);
    fs::path base_dir = cloud_path.parent_path();

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if(read_lx){
        
        std::vector<Eigen::Matrix4d> poe_list_temp;
        NERF_PRE nerf_pre;
        std::vector<std::string> parts;
        std::string part;
        // pcl::PointCloud<pcl::PointXYZRGBL>::Ptr all_points(new pcl::PointCloud<pcl::PointXYZRGBL>);
        // pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointcloud_list_map=nerf_pre.run2(cloud_path, poe_list_temp);
        // std::cout<<"read lx done"<<std::endl;
        // for(auto & point : pointcloud_list_map->points){
        //     PointT point_xyz;
        //     point_xyz.x = point.x;
        //     point_xyz.y = point.y;
        //     point_xyz.z = point.z;
        //     cloud->points.push_back(point_xyz);
        //     all_points->points.push_back(point);
        // }
        // for(auto& matrix : poe_list_temp){
        //     Eigen::Matrix4f matrixf = matrix.cast<float>();
        //     transform_list.push_back(matrixf);
        // }
        
        // std::map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> pointcloud_list_map=nerf_pre.run(cloud_path, 0.0001, poe_list_temp);
        // for(auto pose : poe_list_temp){
        //     Eigen::Matrix4f posef = pose.cast<float>();
        //     transform_list.push_back(posef);
        // }
        // pcl::PointCloud<PointT>::Ptr all_points(new pcl::PointCloud<pcl::PointXYZRGBL>);
        // for(auto it = pointcloud_list_map.begin(); it != pointcloud_list_map.end(); it++){
        //     pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_temp = it->second;
            
        //     for(auto& point : cloud_temp->points){
        //         PointT point_xyz;
        //         point_xyz.x = point.x;
        //         point_xyz.y = point.y;
        //         point_xyz.z = point.z;
        //         point_xyz.r = point.r;
        //         point_xyz.g = point.g;
        //         point_xyz.b = point.b;
        //         point_xyz.label = point.label;
        //         // all_points->push_back(point);
        //         cloud->push_back(point_xyz);
        //     }
        // }

        auto pointcloud_list_map_ = nerf_pre.run(cloud_path, 0.0001, poe_list_temp);
        for(auto& pointcloud : pointcloud_list_map_){
            for(auto& point : pointcloud->points){
                cloud->emplace_back(point);
            }
        }
        
        // pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointcloud_list_map=nerf_pre.run2(cloud_path, poe_list_temp);
        // *cloud = *pointcloud_list_map;

        // int pic_num = 0;
        // std::cout<<"cloud size "<<cloud->points.size()<<std::endl;  
        // for(auto pose : poe_list_temp){
        //     Eigen::Matrix4f posef = pose.cast<float>();
        //     transform_list.push_back(posef);
        // }
        // PointCloudCart2Sph pointcloud_cart2sph(cloud, transform_list[0], rad_resolution, height_resolution);
        // std::cout<<"pointcloud_cart2sph init done"<<std::endl;
        // pointcloud_cart2sph.run(pic_num);
        // Eigen::Matrix4f last_pose = transform_list[0];
        // pic_num++;
        // for(pic_num;pic_num < transform_list.size();pic_num++){
        //     auto curr_pose = transform_list[pic_num];
        //     auto relative_pose = curr_pose.inverse()*last_pose;
        //     pointcloud_cart2sph.update_pointcloud(relative_pose);
        //     pointcloud_cart2sph.run(pic_num);
        //     // pic_num++;
        //     last_pose = curr_pose;
        // }

        int pic_num = 0;
        std::cout<<"cloud size "<<cloud->points.size()<<std::endl;  
        for(auto pose : poe_list_temp){
            Eigen::Matrix4f posef = pose.cast<float>();
            transform_list.push_back(posef);
        }
        // ///test
        // for(auto pose : transform_list){
        //     pcl::PointCloud<PointT>::Ptr test(new pcl::PointCloud<PointT>);
        //     auto start = std::chrono::high_resolution_clock::now();

        //     pcl::transformPointCloud(*cloud, *test, pose);
        //     auto end = std::chrono::high_resolution_clock::now();
        //     std::cout<<"transformPointCloud spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (end - start).count()<<" ms"<<std::endl;
        // }
        // return -1;
        // ///test
        PointCloudCart2Sph pointcloud_cart2sph(cloud, transform_list[0], rad_resolution, height_resolution);
        // std::cout<<"pointcloud_cart2sph init done"<<std::endl;
        // pointcloud_cart2sph.run(pic_num);
        Eigen::Matrix4f last_pose = transform_list[0];
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr init_frame = pointcloud_list_map_[0];
        std::cout<<"init_frame size "<<init_frame->points.size()<<std::endl;
        pointcloud_cart2sph.get_curr_frame_pointcloud(init_frame, last_pose);
        std::cout<<"get_curr_frame_pointcloud "<<std::endl;
        // pointcloud_cart2sph.delete_virual_pointcloud();
        pointcloud_cart2sph.high_delete_virual_pointcloud();
        std::cout<<"delete_virual_pointcloud "<<std::endl;
        init_frame->clear();
        //初始化结束：全局点云的获取、构造PointCloudCart2Sph类，进行第一帧点云的处理
        pic_num++;
        for(pic_num;pic_num < transform_list.size();pic_num++){
            std::cout<<"pic_num "<<pic_num<<std::endl;

            auto curr_pose = transform_list[pic_num];auto curr_frame = pointcloud_list_map_[pic_num];

            auto start = std::chrono::high_resolution_clock::now();

            // auto relative_pose = curr_pose*last_pose.inverse();
            // pointcloud_cart2sph.update_pointcloud(relative_pose);
            pointcloud_cart2sph.update_pointcloud2(curr_pose);
            auto end0 = std::chrono::high_resolution_clock::now();
            auto duration0 = std::chrono::duration_cast<std::chrono::milliseconds> (end0 - start);

            pointcloud_cart2sph.get_curr_frame_pointcloud(curr_frame, curr_pose);
            auto end1 = std::chrono::high_resolution_clock::now();
            auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds> (end1 - end0);

            // pointcloud_cart2sph.delete_virual_pointcloud();
            pointcloud_cart2sph.high_delete_virual_pointcloud();
            auto end2 = std::chrono::high_resolution_clock::now();
            auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds> (end2 - end1);
            last_pose = curr_pose;

            std::cout<<"update_pointcloud spend time "<<duration0.count()<<" ms"<<std::endl;
            std::cout<<"get_curr_frame_pointcloud spend time "<<duration1.count()<<" ms"<<std::endl;
            std::cout<<"delete_virual_pointcloud spend time "<<duration2.count()<<" ms"<<std::endl;
            if(pic_num == 10) break;
        }
        auto filter_point = pointcloud_cart2sph.get_result_pointcloud();
        std::string path_save ="/home/mt_eb1/LYX/filter_noise_point/Table/";
        pcl::transformPointCloud(*filter_point[1], *filter_point[1], last_pose.inverse());
        pcl::io::savePLYFile(path_save+"ori.ply", *filter_point[1]);
        cloud->clear();
        pcl::transformPointCloud(*filter_point[0], *filter_point[0], last_pose.inverse());
        pcl::io::savePLYFile(path_save+"filter_point.ply", *filter_point[0]);
        return -1;
        // pcl::io::savePLYFileASCII((base_dir / "tf_ori.ply").string(), *cloud);
        // return -1;
        // for(auto& matrix : poe_list_temp){
        //     Eigen::Matrix4f matrixf = matrix.cast<float>();
        //     transform_list.push_back(matrixf);
        // }
        std::cout<<"trans lx done"<<std::endl;
        //保存点云来观测
        // pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tmep_points(new pcl::PointCloud<pcl::PointXYZRGBL>);

        // Eigen::Affine3f tf_Y = Eigen::Affine3f::Identity();Eigen::Affine3f tf_Z = Eigen::Affine3f::Identity();
        // Eigen::Matrix3f rotation_Y = Eigen::AngleAxisf(-M_PI / 2.0f, Eigen::Vector3f::UnitY()).matrix();
        // Eigen::Matrix3f rotation_Z = Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitZ()).matrix(); 
        // tf_Y.rotate(rotation_Y);
        // pcl::transformPointCloud(*all_points, *tmep_points, tf_Y);
        // pcl::io::savePLYFileASCII((base_dir / "tf_Y.ply").string(), *tmep_points);
        // tf_Z.rotate(rotation_Z);
        // pcl::transformPointCloud(*all_points, *tmep_points, tf_Z);
        // pcl::io::savePLYFileASCII((base_dir / "tf_Z.ply").string(), *tmep_points);
        // auto rota_ZY = rotation_Z * rotation_Y;
        // Eigen::Affine3f tf_ZY = Eigen::Affine3f::Identity();
        // tf_ZY.rotate(rota_ZY);
        // pcl::transformPointCloud(*all_points, *tmep_points, tf_ZY);
        // pcl::io::savePLYFileASCII((base_dir / "rota_ZY.ply").string(), *tmep_points);
        // return -1;
        // pcl::io::savePCDFileASCII((base_dir / "rota_ZYall_points_.pcd").string(), *all_points);

    }
    else{
        if (pcl::io::loadPLYFile<PointT>(cloud_filename, *cloud) == -1) {
            PCL_ERROR("Couldn't read file \n");
            return -1;
        }
    }
    std::cout << "Loaded " << cloud->width * cloud->height 
              << " data points from " << cloud_filename 
              << " with the following fields: " 
              << pcl::getFieldsList(*cloud) << std::endl;

    std::ifstream camfile(base_dir / "cameras.txt");
    std::cout << "camfile path: " << base_dir / "cameras.txt" << std::endl;
    if (!camfile.is_open()) {
        std::cerr << "Could not open cameras.txt" << std::endl;
        return -1;
    }

    

    std::string cam_model;
    int cam_width, cam_height, idx;
    float cam_fx, cam_fy, cam_cx, cam_cy;

    if (!(camfile >> idx >> cam_model >> cam_width >> cam_height >> cam_fx >> cam_fy >> cam_cx >> cam_cy)) {
        std::cerr << "Error reading camera intrinsics" << std::endl;
        return -1;
    }
    camfile.close();

    float dscale = 0.5;

    int width = cam_width * dscale;
    int height = cam_height * dscale;
    float fx = cam_fx * dscale;
    float fy = cam_fy * dscale;
    float cx = cam_cx * dscale;
    float cy = cam_cy * dscale;

    std::vector<std::string> img_name_list;
    if(read_lx==false){
        std::ifstream imgfile(base_dir / "images.txt");
        std::cout << "imgfile path: " << base_dir / "images.txt" << std::endl;
        if (!imgfile.is_open()) {
            std::cerr << "Could not open images.txt" << std::endl;
            return -1;
        }

        std::string line;
        int line_number = 0;
    
        while (std::getline(imgfile, line)) {
            line_number++;
            if (line_number % 2 == 1) { // 奇数行
                std::istringstream iss(line);
                int index, nul;
                float qw, qx, qy, qz, tx, ty, tz;
                std::string img_name;
                if (!(iss >> index >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> nul >> img_name)) {
                    std::cerr << "Error reading line " << line_number << std::endl;
                    return -1;
                }

                Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                Eigen::Quaternionf q(qw, qx, qy, qz);
                Eigen::Matrix3f rotation = q.toRotationMatrix();
                transform.block<3, 3>(0, 0) = rotation;
                transform(0, 3) = tx;
                transform(1, 3) = ty;
                transform(2, 3) = tz;
            
                transform_list.push_back(transform);
                img_name_list.push_back(img_name);

                // break;
            }
        }
        imgfile.close();
    }else{
        for(int i = 0; i < transform_list.size(); i++){
            img_name_list.push_back(std::to_string(i));
        }
    }
    fs::path depth_dir;
    if(read_lx==false)
        depth_dir = base_dir.parent_path().parent_path() / "depths";
    else
        depth_dir = base_dir.parent_path().parent_path() / "LX_depths";
    if (!fs::exists(depth_dir)) {
        fs::create_directory(depth_dir);
    }

    int num_ = 0;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, transform_list.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t ii = r.begin(); ii != r.end(); ++ii) {
                pcl::PointCloud<PointT>::Ptr cloud_tf(new pcl::PointCloud<PointT>);
                pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
                cv::Mat depth_image(height, width, CV_32FC1, cv::Scalar(0));
                cv::Mat depth_mat(height, width, CV_32SC1, cv::Scalar(0));
                
                Eigen::Matrix4f transform;
                // transform.setIdentity();
                transform = transform_list[ii];

                auto img_name = img_name_list[ii];
                std::string img_name_no_suffix;
                if(read_lx==false)
                    img_name_no_suffix = img_name.substr(0, img_name.find_last_of("."));
                else
                    img_name_no_suffix = img_name;
                std::cout << "Processing image " << img_name_no_suffix << std::endl;

                cloud_tf->points.clear();
                if(read_lx){
                    // if(num_ <3){
                    //     cloud->width = cloud->points.size();
                    //     cloud->height = 1;

                    //     pcl::io::savePLYFileBinary((depth_dir / (img_name_no_suffix + "tfcloud.ply")).string(), *cloud);
                    // }
                    pcl::transformPointCloud(*cloud, *cloud_tf, transform);

                    // pcl::transformPointCloud(*cloud_temp, *cloud_tf, transform.inverse());
                    
                    // if(num_ <3){
                    //     cloud_tf->width = cloud_tf->points.size();
                    //     cloud_tf->height = 1;

                    // pcl::io::savePLYFileBinary((depth_dir / (img_name_no_suffix + "tfcloud_tf.ply")).string(), *cloud_tf);
                    // }
                    // num_++;
                    for (auto& point : cloud_tf->points) {
                        auto pt_raw = point;
                        point.z = pt_raw.x;
                        point.x = - pt_raw.y;
                        point.y = - pt_raw.z;
                        point.r = pt_raw.r;
                        point.g = pt_raw.g;
                        point.b = pt_raw.b;
                        point.label = pt_raw.label;

                        // point.x = -point.x;
                        // point.y = -point.y;
                    }
                    std::cout<<"save "<<(depth_dir / (img_name_no_suffix + ".ply")).string()<<" done"<<std::endl;
                    if (cloud_tf->points.empty()) {
                        std::cerr << "Point cloud is empty!" << std::endl;
                        return -1;
                    }
                    cloud_tf->width = cloud_tf->points.size();
                    cloud_tf->height = 1;
                    pcl::io::savePLYFile((depth_dir / (img_name_no_suffix + "tfcloud_tf.ply")).string(), *cloud_tf);
                    // cloud_tf->width = cloud_tf->points.size();
                    // cloud_tf->height = 1;
                    // pcl::io::savePCDFileASCII((depth_dir / (img_name_no_suffix + "tf_Ztf_X.pcd")).string(), *cloud_tf);
                    // pcl::io::savePCDFileBinary ((depth_dir / (img_name_no_suffix + "tf_Ztf_X.pcd")).string(), *cloud_tf);

                }else{
                    // Eigen::Affine3f tf_X = Eigen::Affine3f::Identity();Eigen::Affine3f tf_Z = Eigen::Affine3f::Identity();
                    // Eigen::Matrix3f rotation_Y = Eigen::AngleAxisf(-M_PI / 2.0f, Eigen::Vector3f::UnitY()).matrix(); // 绕Z轴旋转90度
                    // Eigen::Matrix3f rotation_Z = Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitZ()).matrix(); // 绕Z轴旋转90度
                    // auto rota_ZY = rotation_Z * rotation_Y;
                    // // tf_X.rotate(rotation_X);tf_Z.rotate(rotation_Z);
                    // tf_X.rotate(rota_ZY);
                    // pcl::transformPointCloud(*cloud, *cloud_tf, tf_X);
                    // // pcl::transformPointCloud(*cloud_temp, *cloud_tf, tf_X);

                    // cloud_tf->width = cloud_tf->points.size();
                    // cloud_tf->height = 1;
                    // pcl::io::savePCDFileASCII((depth_dir / (img_name_no_suffix + "rota_ZY.pcd")).string(), *cloud_tf);
                    
                    // cloud_tf->clear();
                    pcl::transformPointCloud(*cloud, *cloud_tf, transform);

                    for (auto& point : cloud_tf->points) {
                        auto pt_raw = point;
                        point.z = pt_raw.x;
                        point.x = - pt_raw.y;
                        point.y = - pt_raw.z;
                        // point.z = pt_raw.x;
                        // point.x = - pt_raw.y;
                        // point.y = - pt_raw.z;
                        // point.x = -point.x;
                        // point.y = -point.y;
                    }

                    // cloud_tf->width = cloud_tf->points.size();
                    // cloud_tf->height = 1;
                    // std::string ply_filename = depth_dir / (img_name_no_suffix + ".ply");
                    // pcl::io::savePLYFileBinary(ply_filename, *cloud_tf);
                    // std::cout<<"save "<<ply_filename<<" done"<<std::endl; 
                }
                float min_depth = std::numeric_limits<float>::max();
                float max_depth = -std::numeric_limits<float>::max();
                // for (const auto& point : cloud_tf->points) {
                //     if (point.z < min_depth) {
                //         min_depth = point.z;
                //     }
                //     if (point.z > max_depth) {
                //         max_depth = point.z;
                //     }
                // }
                // std::cout << "Min depth: " << min_depth << std::endl;
                // std::cout << "Max depth: " << max_depth << std::endl;

                int i = 0;
                for (const auto& point : cloud_tf->points) {
                    i++;
                    if (point.z < MIN_DEPTH) continue;

                    int u0 = static_cast<int>(fx * point.x / point.z + cx);
                    int v0 = static_cast<int>(fy * point.y / point.z + cy);

                    int du = fx * 0.01 / point.z;
                    int dv = fy * 0.01 / point.z;
                    float dz = 0.1;

                    for (int m = -dv; m <= dv; m++) {
                        for (int n = -du; n <= du; n++) {
                            int u = u0 + n;
                            int v = v0 + m;//确定了点在图像上的位置
                            if (u >= 0 && u < width && v >= 0 && v < height) {
                                if (depth_image.at<float>(v, u) < EPS) {
                                    depth_mat.at<int>(v, u) = 1;
                                    depth_image.at<float>(v, u) = point.z;
                                } else {
                                    float du = 2.0 * point.z / fx;
                                    if (fabs(point.z - depth_image.at<float>(v, u)) < du) {
                                        depth_mat.at<int>(v, u)++;
                                        depth_image.at<float>(v, u) = point.z / depth_mat.at<int>(v, u) + depth_image.at<float>(v, u) * (depth_mat.at<int>(v, u) - 1) / depth_mat.at<int>(v, u);
                                    } else if (point.z < (depth_image.at<float>(v, u) - du)) {
                                        depth_mat.at<int>(v, u) = 1;
                                        depth_image.at<float>(v, u) = point.z;
                                    }
                                }
                            }
                        }
                    }
                }

                float z_max = 0.0;
                float z_min = 100.0;
                for (int i = 0; i < height; i++) {
                    for (int j = 0; j < width; j++) {
                        float depth = depth_image.at<float>(i, j);
                        if (depth < EPS) continue;
                        if (depth > z_max) z_max = depth;
                        if (depth < z_min) z_min = depth;
                        depth_image.at<float>(i, j) = 1.0 / depth;
                    }
                }

                printf("z_max: %f, z_min: %f\n", z_max, z_min);

                fs::path depth_path = depth_dir / (img_name_no_suffix + ".yml");
                cv::FileStorage fs(depth_path.string(), cv::FileStorage::WRITE);

                fs << "depth_image" << depth_image;
                fs.release();

                cv::Mat depth_image_show = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
                for (int i = 0; i < height; i++) {
                    for (int j = 0; j < width; j++) {
                        float depth = depth_image.at<float>(i, j);
                        if (depth < EPS) continue;
                        depth_image_show.at<uint8_t>(i, j) = ((1.0 / depth * 1.0) / (z_max - z_min)) * 255.0;
                    }
                }

                cv::imwrite((depth_dir / (img_name_no_suffix + ".png")).string(), depth_image_show);
            }
        });


    return 0;
}
