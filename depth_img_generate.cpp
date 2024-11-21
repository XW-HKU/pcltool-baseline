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
#define MIN_DEPTH 0.5
#define EPS 0.0001

namespace fs = std::filesystem;
typedef pcl::PointXYZRGBL PointT;
float rad_resolution = 0.2;
float height_resolution = 0.02;
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
void test(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointcloud){
    pointcloud_map_in_cam A;
    for(auto &point : pointcloud->points){
        
    }
}

int get_point_height_index_in_Mat_height(const PointT &point){
    int height ;
    if(point.y <0 )height = -point.y;
    else height = point.y;
    if(height > 0 ) height = height + max_height;
    else height = height - min_height;
    if(height < 0 or height > (max_height - min_height)) return false;

    return height/height_resolution;

}

int get_point_rad_index_in_Mat_width(const PointT &point,const Eigen::Matrix4d &pose_matrix){
    Eigen::Vector3d point_xyz;
    point_xyz << point.x, point.y, point.z;
    point_xyz = point_xyz.normalized();
    Eigen:: Vector3d ZAxis ;//因为Z为深度方向
    ZAxis.x() = pose_matrix(2,0);
    ZAxis.y() = pose_matrix(2,1);
    ZAxis.z() = pose_matrix(2,2);
    ZAxis = ZAxis.normalized();

    Eigen:: Vector3d Y_axis ;//确定顺时逆时针方向
    Y_axis.x() = pose_matrix(1,0);
    Y_axis.y() = pose_matrix(1,1);
    Y_axis.z() = pose_matrix(1,2);
    Y_axis = Y_axis.normalized();
    double dotProduct = point_xyz.dot(ZAxis);
    double cosTheta = std::clamp(dotProduct, -1.0, 1.0);
    Eigen::Vector3d crossProduct = point_xyz.cross(ZAxis);
    double direction = crossProduct.dot(Y_axis);
    double angle = std::atan2(crossProduct.norm(), dotProduct);
    if (direction < 0) {
        angle = 2 * M_PI - angle;
    }
    return angle/rad_resolution;
}
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
        
        std::map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> pointcloud_list_map=nerf_pre.run(cloud_path, 0.0001, poe_list_temp);
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr all_points(new pcl::PointCloud<pcl::PointXYZRGBL>);
        for(auto it = pointcloud_list_map.begin(); it != pointcloud_list_map.end(); it++){
            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_temp = it->second;
            
            for(auto& point : cloud_temp->points){
                PointT point_xyz;
                point_xyz.x = point.x;
                point_xyz.y = point.y;
                point_xyz.z = point.z;
                point_xyz.r = point.r;
                point_xyz.g = point.g;
                point_xyz.b = point.b;
                point_xyz.label = point.label;
                // all_points->points.push_back(point);
                cloud->points.push_back(point_xyz);
            }
        }
        for(auto& matrix : poe_list_temp){
            Eigen::Matrix4f matrixf = matrix.cast<float>();
            transform_list.push_back(matrixf);
        }
        std::cout<<"trans lx done"<<std::endl;
        //保存点云来观测
        // all_points->width = all_points->points.size();
        // all_points->height = 1;
        // Eigen::Affine3f tf_X = Eigen::Affine3f::Identity();Eigen::Affine3f tf_Z = Eigen::Affine3f::Identity();
        // Eigen::Matrix3f rotation_Y = Eigen::AngleAxisf(-M_PI / 2.0f, Eigen::Vector3f::UnitY()).matrix(); // 绕Z轴旋转90度
        // Eigen::Matrix3f rotation_Z = Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitZ()).matrix(); // 绕Z轴旋转90度
        // auto rota_ZY = rotation_Z * rotation_Y;
        // tf_X.rotate(rota_ZY);
        // pcl::io::savePCDFileASCII((base_dir / "all_points.pcd").string(), *all_points);
        // pcl::transformPointCloud(*all_points, *all_points, tf_X);
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
