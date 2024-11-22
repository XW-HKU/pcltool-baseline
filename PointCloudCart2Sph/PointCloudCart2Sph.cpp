#include "PointCloudCart2Sph.h"
float PointCloudCart2Sph::get_point_in_sph_theta(const pointXYZRGBHM& point){
    const float& x = point.xyz.x();
    const float& y = point.xyz.y();
    float theta = atan2(y, x);
    theta = M_PI - theta;
    return theta;
}
float PointCloudCart2Sph::get_point_in_sph_phi(const pointXYZRGBHM& point){
    const float& x = point.xyz.x();
    const float& y = point.xyz.y();
    const float& z = point.xyz.z();
    float r = sqrt(x*x + y*y + z*z);
    float phi = acos(z/r);
    float phi_theta = phi/M_PI*180;
    // if(phi < 0 )
    // for(int index=0;index<30;index++)
    //     std::cout<<"phi "<<phi_theta<<std::endl;
    return phi;
}
void PointCloudCart2Sph:: compute_Sphgrid_depth(std::vector<PointCloudCart2Sph::Sphgrid> &sphgrid){
    for(auto &grid : sphgrid){
        if(grid.occupied == false) continue;
        float min_r = 0;
        for(auto &point : grid.points){
            float r = sqrt(point->xyz.x()*point->xyz.x() + point->xyz.y()*point->xyz.y() + point->xyz.z()*point->xyz.z());
            min_r += r;
        }
        grid.depth = min_r/grid.points.size();
        
    }
}
cv::Mat PointCloudCart2Sph::get_depth_image_show(std::vector<PointCloudCart2Sph::Sphgrid> &sphgrid) {
    int occupied_num = 0;
    double max_depth = 30;
    cv::Mat depth_image_show = cv::Mat(phi_size, theta_size, CV_8UC1, cv::Scalar(0));
    for(int i=0;i<phi_size;i++){
        for(int j=0;j<theta_size;j++){
            int index = i*theta_size+j;
            if(sphgrid[index].occupied == false){
                depth_image_show.at<uchar>(i, j) = 255;
                occupied_num++;            }
            else{
                double depth = sphgrid[index].depth;    
                if(depth < 0.01){
                    depth_image_show.at<uchar>(i, j) = 0;
                }else if(depth > max_depth){
                    depth_image_show.at<uchar>(i, j) = 0;
                }else{
                    depth_image_show.at<uchar>(i, j) = depth/max_depth*255;
                }
            }
        }
    }
    
    std::cout<<"occupied_num "<<occupied_num<<std::endl;    
    return depth_image_show;
}
std::vector<PointCloudCart2Sph::Sphgrid> PointCloudCart2Sph::local_pointcloud_sphgrid(){
    std::vector<PointCloudCart2Sph::Sphgrid> LocalPointcloudSphgrid;
    for(int i=0;i<phi_size;i++){
        for(int j=0;j<theta_size;j++){
            LocalPointcloudSphgrid.push_back(PointCloudCart2Sph::Sphgrid(i, j));
        }
    }
    // int test_num = 0;
    for(auto &point : pointcloud){

        float theta = get_point_in_sph_theta(point);
        float phi = get_point_in_sph_phi(point);
        int index = index_in_vector_of_sphgrid(theta, phi);

        LocalPointcloudSphgrid[index].points.push_back(&point);

        if(LocalPointcloudSphgrid[index].occupied == false){
            LocalPointcloudSphgrid[index].theta_index = theta;
            LocalPointcloudSphgrid[index].phi_index = phi;
            LocalPointcloudSphgrid[index].occupied = true;
        }
    }
    return LocalPointcloudSphgrid;
}

std::vector<PointCloudCart2Sph::Sphgrid> PointCloudCart2Sph::curr_frame_pointcloud_sphgrid(){
    std::vector<PointCloudCart2Sph::Sphgrid> CurrFramePointcloudSphgrid;
    for(int i=0;i<phi_size;i++){
        for(int j=0;j<theta_size;j++){
            CurrFramePointcloudSphgrid.push_back(PointCloudCart2Sph::Sphgrid(i, j));
        }
    }
    for(auto &point : curr_frame_pointcloud){
        float theta = get_point_in_sph_theta(point);
        float phi = get_point_in_sph_phi(point);
        int index = index_in_vector_of_sphgrid(theta, phi);
        CurrFramePointcloudSphgrid[index].points.push_back(&point);
        if(CurrFramePointcloudSphgrid[index].occupied == false){
            CurrFramePointcloudSphgrid[index].theta_index = theta;
            CurrFramePointcloudSphgrid[index].phi_index = phi;
            CurrFramePointcloudSphgrid[index].occupied = true;
        }
    }
    compute_Sphgrid_depth(CurrFramePointcloudSphgrid);//计算每个sphgrid的平均深度

    return CurrFramePointcloudSphgrid;
}
void PointCloudCart2Sph::get_curr_frame_pointcloud(pcl::PointCloud<PointT>::Ptr curr_frame,const Eigen::Matrix4f &curr_pose){
    curr_frame_pointcloud.clear();
    pcl::transformPointCloud(*curr_frame, *curr_frame, curr_pose);
    for(auto & point : curr_frame->points){
        curr_frame_pointcloud.emplace_back(pointXYZRGBHM(point));
    }
    // std::cout<<"curr_frame_pointcloud size "<<curr_frame_pointcloud.size()<<std::endl;
}
void PointCloudCart2Sph::run(int picture_num){
    std::vector<PointCloudCart2Sph::Sphgrid> sphgrid = local_pointcloud_sphgrid();
    // std::cout<<"sphgrid size "<<sphgrid.size()<<std::endl;
    compute_Sphgrid_depth(sphgrid);
    std::cout<<"compute_Sphgrid_depth done"<<std::endl;
    cv::Mat depth_image_show = get_depth_image_show(sphgrid);
    cv::String path = "/home/mt_eb1/LYX/filter_noise_point/Table/"+std::to_string(picture_num)+"test.png";
    cv::imwrite(path, depth_image_show);
    std::cout<<picture_num<<" done "<<std::endl;
    // cv::waitKey(0);
}

void PointCloudCart2Sph::delete_virual_pointcloud(){
    std::vector<PointCloudCart2Sph::Sphgrid> map_sphgrid = local_pointcloud_sphgrid();
    std::vector<PointCloudCart2Sph::Sphgrid> curr_frame_sphgrid = curr_frame_pointcloud_sphgrid();
    // std::cout<<"map_sphgrid size "<<map_sphgrid.size()<<std::endl;
    // std::cout<<"curr_frame_sphgrid size "<<curr_frame_sphgrid.size()<<std::endl;

    for(int i=0;i<curr_frame_sphgrid.size();i++){
        if(curr_frame_sphgrid[i].occupied == true){
            int curr_frame_sphgrid_poins_size_in_sphgrid = curr_frame_sphgrid[i].points.size();
            float sum_depth = 0;
            for(auto &point : curr_frame_sphgrid[i].points){
                float r = sqrt(point->xyz.x()*point->xyz.x() + point->xyz.y()*point->xyz.y() + point->xyz.z()*point->xyz.z());
                sum_depth += r;
            }
            float bar_depth = sum_depth/curr_frame_sphgrid_poins_size_in_sphgrid;

            for(auto &point : map_sphgrid[i].points){
                float r = sqrt(point->xyz.x()*point->xyz.x() + point->xyz.y()*point->xyz.y() + point->xyz.z()*point->xyz.z());
                if(r < bar_depth){
                    point->miss +=1;
                }else{
                    point->hit +=1;
                }
            }
        }
    }
}

void PointCloudCart2Sph::high_delete_virual_pointcloud(){
    std::vector<PointCloudCart2Sph::Sphgrid> curr_frame_sphgrid = curr_frame_pointcloud_sphgrid();
    // int test_num = 0;
    std::vector<pointXYZRGBHM> save_pointcloud;
    for(auto &point : pointcloud){

        float theta = get_point_in_sph_theta(point);
        float phi = get_point_in_sph_phi(point);
        int index = index_in_vector_of_sphgrid(theta, phi);

        if(curr_frame_sphgrid[index].occupied == true){
            float r = sqrt(point.xyz.x()*point.xyz.x() + point.xyz.y()*point.xyz.y() + point.xyz.z()*point.xyz.z());
            float curr_frame_sphgrid_depth = curr_frame_sphgrid[index].depth;
            if((curr_frame_sphgrid_depth - r)>0.01 && (curr_frame_sphgrid_depth - r)<0.06 || (r - curr_frame_sphgrid_depth)>0.01 && (r - curr_frame_sphgrid_depth)<0.06){
                filter_pointcloud.emplace_back(point);
            }else{
                save_pointcloud.emplace_back(point);
            }
        }
    }
    pointcloud = save_pointcloud;


}

std::vector<pcl::PointCloud<PointT>::Ptr> PointCloudCart2Sph::get_result_pointcloud(){
    pcl::PointCloud<PointT>::Ptr result_pointcloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr save_pointcloud(new pcl::PointCloud<PointT>);

    // for(auto& point : pointcloud){
    //     if(point.hit > 0.5*point.miss){
    //         PointT point_xyz;
    //         point_xyz.x = point.xyz.x();
    //         point_xyz.y = point.xyz.y();
    //         point_xyz.z = point.xyz.z();
    //         result_pointcloud->points.push_back(point_xyz);
    //     }
    // }

    //test
    std::cout<<"pointcloud size "<<pointcloud.size()<<std::endl;    
    int select_point = 0;
    int sum_miss = 0;
    int sum_hit = 0;
    for(auto& point : pointcloud){
        int miss_num = point.miss;
        int hit_num = point.hit;
        if(miss_num !=0 && hit_num !=0){
            select_point++;
            sum_miss += miss_num;
            sum_hit += hit_num;
        }
        if(miss_num > 0.7*hit_num){
            PointT point_xyz;
            point_xyz.x = point.xyz.x();
            point_xyz.y = point.xyz.y();
            point_xyz.z = point.xyz.z();
            point_xyz.r = point.rgb(0);
            point_xyz.g = point.rgb(1);
            point_xyz.b = point.rgb(2);
            result_pointcloud->points.push_back(point_xyz);
        }else{
            PointT point_xyz;
            point_xyz.x = point.xyz.x();
            point_xyz.y = point.xyz.y();
            point_xyz.z = point.xyz.z();
            point_xyz.r = point.rgb(0);
            point_xyz.g = point.rgb(1);
            point_xyz.b = point.rgb(2);
            save_pointcloud->points.push_back(point_xyz);
        }
    }
    float bar_miss = float(sum_miss)/float(select_point);
    float bar_hit = float(sum_hit)/float(select_point);
    std::cout<<"bar_miss "<<bar_miss<<std::endl;
    std::cout<<"bar_hit "<<bar_hit<<std::endl;
    std::cout<<"miss_num > hit_num "<<result_pointcloud->points.size()<<std::endl; 
    std::vector<pcl::PointCloud<PointT>::Ptr> pointcloud_list;
    pointcloud_list.push_back(result_pointcloud);
    pointcloud_list.push_back(save_pointcloud);
    return pointcloud_list;
}

