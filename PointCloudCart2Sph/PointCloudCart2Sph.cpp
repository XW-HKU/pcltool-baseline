#include "PointCloudCart2Sph.h"
#include <pcl/features/normal_3d.h> //For computeCovarianceMatrix
#include <filesystem> 
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/common/pca.h>

namespace fs = std::filesystem;

float PointCloudCart2Sph::get_point_in_sph_theta(const PointT& point){
    const float& x = point.x;
    const float& y = point.y;
    float theta = atan2(y, x);
    theta = M_PI - theta;
    return theta;
}
float PointCloudCart2Sph::get_point_in_sph_phi(const PointT& point){
    const float& x = point.x;
    const float& y = point.y;
    const float& z = point.z;
    float r = sqrt(x*x + y*y + z*z);
    float phi = acos(z/r);
    // if(phi < 0 )

    return phi;
}
float PointCloudCart2Sph::get_point_in_sph_r(const PointT& point){
    
    return sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
}

std::vector<PointCloudCart2Sph::Sphgrid> PointCloudCart2Sph:: curr_frame_pointcloud_sphgrid (pcl::PointCloud<PointT>::Ptr& curr_frame, int i,float& max_deep){
    std::vector<PointCloudCart2Sph::Sphgrid> CurrFramePointcloudSphgrid;
    int temp = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < phi_size; i++){
        for(int j = 0; j < theta_size; j++){
            CurrFramePointcloudSphgrid.emplace_back(PointCloudCart2Sph::Sphgrid(0));
        }
        
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    int point_index = 0;
    for(auto & point : curr_frame->points){

        // if(ori_pointcloudIndex_pointIndex_status[i][point_index]>2){
        //     continue;
        // }  

        float theta = get_point_in_sph_theta(point);
        float phi = get_point_in_sph_phi(point);
        int index = index_in_vector_of_sphgrid(theta, phi);
        if(CurrFramePointcloudSphgrid[index].occupied == false){
            CurrFramePointcloudSphgrid[index].occupied = true;
            // CurrFramePointcloudSphgrid[index].depth = 0;
        }
        float depth = get_point_in_sph_r(point);
        if(CurrFramePointcloudSphgrid[index].max_depth < depth){
            CurrFramePointcloudSphgrid[index].max_depth = depth;
        }
        if(CurrFramePointcloudSphgrid[index].min_depth > depth){
            CurrFramePointcloudSphgrid[index].min_depth = depth;
            CurrFramePointcloudSphgrid[index].x = point.x;
            CurrFramePointcloudSphgrid[index].y = point.y;
            CurrFramePointcloudSphgrid[index].z = point.z;
        }
        CurrFramePointcloudSphgrid[index].bar_depth += depth;
        //test
        // CurrFramePointcloudSphgrid[index].x += point.x;
        // CurrFramePointcloudSphgrid[index].y += point.y;
        // CurrFramePointcloudSphgrid[index].z += point.z;
        //test
        CurrFramePointcloudSphgrid[index].size += 1;
        // if(CurrFramePointcloudSphgrid[index].occupied == false){
        //     CurrFramePointcloudSphgrid[index].occupied = true;
        // }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    
    for(auto & grid : CurrFramePointcloudSphgrid){
        if(grid.occupied == false) continue;
        if(grid.max_depth >max_deep){
            max_deep = grid.max_depth;
        }
        grid.bar_depth = grid.bar_depth/grid.size;
        grid.mindepth_point.x = grid.x;grid.mindepth_point.y=grid.y;grid.mindepth_point.z=grid.z;
        //test
        // grid.x = grid.x/grid.size;
        // grid.y = grid.y/grid.size;
        // grid.z = grid.z/grid.size;
        //test

        // std::cout<<"grid.depth "<<grid.depth<<std::endl;
    }

    auto t3 = std::chrono::high_resolution_clock::now();
    // std::cout<<"init CurrFramePointcloudSphgrid spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t1 - t0).count()<<" ms"<<std::endl;
    // std::cout<<"fill CurrFramePointcloudSphgrid spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t2 - t1).count()<<" ms"<<std::endl;
    // std::cout<<"compute CurrFramePointcloudSphgrid depth spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t3 - t2).count()<<" ms"<<std::endl;
    return CurrFramePointcloudSphgrid;    

}
  auto reduction = [](int x, int y) {
    return x + y;
  };
void PointCloudCart2Sph::delete_virual_pointcloud(){
    // Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
    // int stop_index = 89;
    std :: mutex mtx;
    // std :: mutex save_lock;
    // std:: mutex count_lock;
    // PointT testpoint;testpoint.x = 1.414068;testpoint.y = 0.288488;testpoint.z = 0.182881;
    // int TestPointSphIndex = get_testpoint_sphindex(testpoint, stop_index);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, ori_pointcloud_list.size()),
    [&] (tbb::blocked_range<size_t> r)
    {
        
        std::vector<std::pair<int, int>> filter_idnex;

        for(size_t i = r.begin(); i != r.end(); i++){
            // if(r.begin() != stop_index ) break;

            SphTools sph_tools;

            auto t0 = std::chrono::high_resolution_clock::now();

            

            Eigen::Matrix4f curr_pose = pose_matrix[i];//获取当前帧的位姿
            local_pose.x = curr_pose(0,3);local_pose.y = curr_pose(1,3);local_pose.z = curr_pose(2,3);//记录局部位姿
            std::vector<pcl::PointCloud<PointT>::Ptr> ori_pointcloud_list_;//用于储存ori_pointcloud_list的变换后的点云

            for(auto pointcloud_ptr : ori_pointcloud_list){//将原始点云全部变换到局部坐标系下
                
                pcl::PointCloud<PointT>::Ptr temp_pointcloud(new pcl::PointCloud<PointT>);
                pcl::transformPointCloud(*pointcloud_ptr, *temp_pointcloud, curr_pose.inverse());
                // *transafter_pointcloud += *temp_pointcloud;
                ori_pointcloud_list_.emplace_back(temp_pointcloud);
            }


            //记录用于位姿变换的位姿
            Eigen::Matrix4f relative_pose = curr_pose.inverse();

            bool do_once = false;//用于只记录当前的点临近栅格并保存点云一次，只针对单帧情况

            float curr_frame_max_deep = 0;

            auto curr_pointcloud_sphgrid = curr_frame_pointcloud_sphgrid(ori_pointcloud_list_[i],i,curr_frame_max_deep);//获取当前帧的球坐标系下的栅格点云以及最大深度

            std::cout<<"curr_frame_max_deep "<<curr_frame_max_deep<<std::endl;

            // 从 curr_pointcloud_sphgrid 中构建cv::mat
            // cv::Mat depth_image = cv::Mat::zeros(phi_size, theta_size, CV_32FC1);
            // for (int i = 0; i < phi_size; i++){
            //     for (int j = 0; j < theta_size; j++){
            //         int index = i*theta_size + j;
            //         if(curr_pointcloud_sphgrid[index].occupied == false) continue;
            //         depth_image.at<float>(i, j) = curr_pointcloud_sphgrid[index].bar_depth;
            //     }
            // }
            // 将depth_image存储为/home/mt_eb1/LYX/filter_noise_point/Table/depth.png
            // cv::Mat depth_image_show;
            // cv::normalize(depth_image, depth_image_show, 0, 255, cv::NORM_MINMAX);
            // // inverse the color
            // cv::Mat depth_image_show_inv = cv::Mat::zeros(phi_size, theta_size, CV_32FC1);
            // std::cout<<"min_phi "<<min_phi<<" max_phi "<<max_phi<<std::endl;    
            // for (int i = 0; i < phi_size; i++){
            //     if(i >= min_phi && i <=max_phi)
            //     for (int j = 0; j < theta_size; j++){
            //         depth_image_show_inv.at<float>(i, j) = 255 - depth_image_show.at<float>(i, j);
            //     }
            //     else{
            //         for (int j = 0; j < theta_size; j++){
            //             depth_image_show_inv.at<float>(i, j) = depth_image_show.at<float>(i, j);
            //         }
            //     }
            // }

            // std::string temp_save_i = std::to_string(i);
            
            // //若能找到"/home/mt_eb1/LYX/DataFromServe/HongKongData/MT20241129-100544/curr_frame_depth/"+temp_save_i+".png"路径下的文件，则不保存
            
            // if(fs::exists(img_save_path+img_save_file+temp_save_i+".png")){

            // }else{
            //     cv::imwrite(img_save_path+img_save_file+temp_save_i+".png", depth_image_show_inv);
            // }


            auto t2 = std::chrono::high_resolution_clock::now();


            //遍历点云
            for(int pointcloud_of_map_ptr_index = 0 ; pointcloud_of_map_ptr_index < ori_pointcloud_list_.size();pointcloud_of_map_ptr_index++){

                auto t3 = std::chrono::high_resolution_clock::now();

                //获取当前map的点云若为当前帧，跳过
                if(pointcloud_of_map_ptr_index == i){
                    // for(int point_index = 0 ;point_index < ori_pointcloud_list_[pointcloud_of_map_ptr_index]->points.size();point_index++){
                    //     const PointT& point_in_selected_pointcloud = ori_pointcloud_list_[pointcloud_of_map_ptr_index]->points[point_index];
                    //     auto temp = point_in_selected_pointcloud;temp.r = 0;temp .g = 255;temp.b = 0;
                    //     obvious_point(TestPointSphIndex, temp, stop_index);
                    // }

                    continue;
                } 

                //获取当前map的点
                for(int point_index = 0 ;point_index < ori_pointcloud_list_[pointcloud_of_map_ptr_index]->points.size();point_index++){

                    auto t4 = std::chrono::high_resolution_clock::now();



                    //获取Map中欧的点
                    const PointT& point_in_selected_pointcloud = ori_pointcloud_list_[pointcloud_of_map_ptr_index]->points[point_index];
                    // obvious_point(TestPointSphIndex, point_in_selected_pointcloud, stop_index);
                    float map_point_depth = get_point_in_sph_r(point_in_selected_pointcloud);
                    //map点的深度大于当前帧最深点的深度，跳过。相当于不在当前帧的视角范围内的点不做处理
                    if(map_point_depth > curr_frame_max_deep ) continue;
                    //获取Map点在球坐标系下的theta和phi
                    float theta = get_point_in_sph_theta(point_in_selected_pointcloud);
                    float phi = get_point_in_sph_phi(point_in_selected_pointcloud);
                    int index = index_in_vector_of_sphgrid(theta, phi);



                    //若球坐标系像素不包含当前帧的点，则跳过
                    if(curr_pointcloud_sphgrid[index].occupied == false) continue;

                    //获取与Map在同一个像素中的最浅的当前帧点
                    PointT curr_frame_pointcloud_sphgrid_mindepth_point;
                    curr_frame_pointcloud_sphgrid_mindepth_point.x = curr_pointcloud_sphgrid[index].x;
                    curr_frame_pointcloud_sphgrid_mindepth_point.y = curr_pointcloud_sphgrid[index].y;
                    curr_frame_pointcloud_sphgrid_mindepth_point.z = curr_pointcloud_sphgrid[index].z;

                    //获取当前帧点在球坐标系下的深度，theta，phi
                    float depth_curr_frame_pointcloud_sphgrid_bar_point = get_point_in_sph_r(curr_frame_pointcloud_sphgrid_mindepth_point);
                    float curr_frame_pointcloud_sphgrid_bar_point_theta = get_point_in_sph_theta(curr_frame_pointcloud_sphgrid_mindepth_point);
                    float curr_frame_pointcloud_sphgrid_bar_point_phi = get_point_in_sph_phi(curr_frame_pointcloud_sphgrid_mindepth_point);

                    //获取Map点在球坐标系下的theta和phi的索引
                    int point_in_selected_pointcloud_theta_index = get_point_in_sph_theta_index(point_in_selected_pointcloud);
                    int point_in_selected_pointcloud_phi_index = get_point_in_sph_phi_index(point_in_selected_pointcloud);
                    //若Map点超过当前帧的视角范围，则跳过
                    if( (point_in_selected_pointcloud_phi_index >= max_phi) || (point_in_selected_pointcloud_phi_index <= min_phi) )continue;



                    auto t5 = std::chrono::high_resolution_clock::now();

                    //map点要浅于当前帧最浅点的深度时，被认定为可能的拉丝点
                    if( map_point_depth < (curr_pointcloud_sphgrid[index].min_depth -0.1))//若curr_pointcloud_sphgrid[index].occupied == false, 则该min_depth为999999(初始值)
                    {

                        std::vector<int> searched_pixel_index;//临近像素的下标
                        
                        //获取临近像素
                        int search_pixel_range = 1;//像素范围
                        for(int i = -search_pixel_range; i <= search_pixel_range; i++){
                            for(int j = -search_pixel_range; j <= search_pixel_range; j++){
                                int theta_index = point_in_selected_pointcloud_theta_index + j;
                                int phi_index = point_in_selected_pointcloud_phi_index + i;
                                
                                if(theta_index < 0 || theta_index >= theta_size || phi_index < min_phi || phi_index >= max_phi) continue;//与下面的if保持一致，所有像素都在非视角边缘内
                                searched_pixel_index.emplace_back(phi_index*theta_size + theta_index);
      
                            }
                        }

                        bool point_in_range = false;


                        
                        
                        bool is_delete = true;//最为关键的变量，判断该点是否应该删除
                        int pass_time = 0;//记录临近像素中没有被占用的像素的数量
                        // int size_small = 0;//记录临近像素中point_nums小于2的像素的数量

                        if(searched_pixel_index.empty()) continue;//若临近像素为空，则跳过



                        for(auto & pixel_index : searched_pixel_index){
                            if(curr_pointcloud_sphgrid[pixel_index].occupied == false){
                                pass_time++;//计数 非占据体素
                                continue;
                            }
                            // if(curr_pointcloud_sphgrid[pixel_index].size < 2){
                            //     size_small++;//计数 临近体素中点数小于2的体素
                            // }
                            float sphgrid_depth = curr_pointcloud_sphgrid[pixel_index].min_depth;//取当前curr_pointcloud_sphgrid中最浅点的深度
                            // float diff_angle = sph_tools.ComputeAngle(point_in_selected_pointcloud, curr_pointcloud_sphgrid[pixel_index].mindepth_point);//计算map点和像素中最浅点的角度差
                            // diff_angle = diff_angle*180/M_PI;

                            if( map_point_depth  > (sphgrid_depth-0.1)){//map_point_depth(中心体素深度) 若有一个像素浅于中心体素，则不删除
                                // pass_time++;

                                is_delete = false;

                                break;
                            }
                            else{
                                float diff_angle = sph_tools.ComputeAngle(point_in_selected_pointcloud, curr_pointcloud_sphgrid[pixel_index].mindepth_point);//计算map点和像素中最浅点的角度差
                                diff_angle = diff_angle*180/M_PI;  
                                if(diff_angle < 0.3){
                                    is_delete = true;
                                }else{
                                    is_delete = false;                                
                                    }          
                            }

                            



                        }


                        if(pass_time == searched_pixel_index.size()) is_delete = false; //若临近像素中都没有当前帧的点，则不删除

                        
                        //若该点应该删除，则将其filter次数加1
                        if(is_delete){
                            // filter_idnex.emplace_back(std::make_pair(pointcloud_of_map_ptr_index, point_index));
                            mtx.lock();
                            ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index]++;
                            mtx.unlock();
                            //test
                            // if(my_point_index == index){
                            //     my_point_filter_map_sph->points.push_back(point_in_selected_pointcloud);
                            // }
                            // //test
                        }



                    }


                }
                auto t5 = std::chrono::high_resolution_clock::now();
            }


            auto t6 = std::chrono::high_resolution_clock::now();
            std::string p_n = std::to_string(i);



            std::cout<<"begin  "<<i<<" end "<<i<<" spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t6 - t2).count()<<" ms"<<std::endl;
            // if((i+1) % 10 == 0){
            //     save_pointcloud();
            // }
            // count_lock.lock();
            // process_num++;
            // count_lock.unlock();

            // if((process_num+1)%10 == 0){
            //     std::cout<<"save "<<std::endl;
            //     save_pointcloud();
            // }
        }

        // mtx.lock();
        // for(auto & index : filter_idnex){
        //     // ori_pointcloudIndex_pointIndex_status[index.first][index.second]++;
        //     *ori_pointcloudIndex_pointIndex_status_automic[index.first][index.second]++;
        // }
        // mtx.unlock();

        // save_pointcloud();
    }
    
        );
    // 单线程和多线程。注释掉TBB，改变开始的for循环
    save_pointcloud();
    // Eigen::Matrix4f save_curr = pose_matrix[stop_index];
    // PointT ori_point;
    // //请将save_curr位移部分赋值给ori_point的xyz
    // ori_point.x = save_curr(0,3);ori_point.y = save_curr(1,3);ori_point.z = save_curr(2,3);ori_point.g =255;
    // auto temp_cloud = ori_pointcloud_list[stop_index];
    // temp_cloud->points.push_back(ori_point);
    // temp_cloud->width = temp_cloud->points.size();
    // temp_cloud->height = 1;
    // pcl::io::savePLYFileASCII(base_path+"/out/"+std::to_string(stop_index)+"frame.ply", *ori_pointcloud_list[stop_index]);
}


void PointCloudCart2Sph::get_depth_image_show(){ 
    // pcl::PointCloud<PointT>::Ptr all_points(new pcl::PointCloud<PointT>);
    // for(auto & pointcloud : ori_pointcloud_list){
    //     *all_points += *pointcloud;
    // }
    // pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/ori_allpoints.ply", *all_points);
    Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
    // pcl::transformPointCloud(*all_points, *all_points, curr_pose);
    int pose_index = 0;
    for(auto pose_ma : pose_matrix){
        pcl::PointCloud<PointT>::Ptr all_points_(new pcl::PointCloud<PointT>);
        Eigen::Matrix4f relative_pose = pose_ma.inverse()*last_pose;
        last_pose = pose_ma;
        for(auto& pointcloud_ptr : ori_pointcloud_list){
            // pcl::PointCloud<PointT>::Ptr temp_pointcloud (new pcl::PointCloud<PointT>);
            pcl::transformPointCloud(*pointcloud_ptr, *pointcloud_ptr, relative_pose);
            *all_points_ += *pointcloud_ptr;
        }
        std::vector<PointCloudCart2Sph::Sphgrid> PointcloudSphgrid;
        for(int i = 0; i < phi_size; i++){
            for(int j = 0; j < theta_size; j++){
                PointcloudSphgrid.emplace_back(PointCloudCart2Sph::Sphgrid(0));
            }
            
        }   
        for(auto & point : all_points_->points){
            float theta = get_point_in_sph_theta(point);
            float phi = get_point_in_sph_phi(point);
            int index = index_in_vector_of_sphgrid(theta, phi);
            float depth = get_point_in_sph_r(point);
            PointcloudSphgrid[index].occupied = true;
            PointcloudSphgrid[index].bar_depth = depth;
        }
        for(auto & point : all_points_->points){
            float theta = get_point_in_sph_theta(point);
            float phi = get_point_in_sph_phi(point);
            int index = index_in_vector_of_sphgrid(theta, phi);
            float depth = get_point_in_sph_r(point);
            if(PointcloudSphgrid[index].occupied == false) continue;
            if(depth < PointcloudSphgrid[index].bar_depth){
                PointcloudSphgrid[index].bar_depth = depth;
            }
        }
        cv::Mat depth_image = cv::Mat(phi_size, theta_size, CV_8UC1, cv::Scalar(0));
        for(int i = 0; i < phi_size; i++){
            for(int j = 0; j < theta_size; j++){
                auto depth = PointcloudSphgrid[i*theta_size+j].bar_depth;
                if(depth>30){
                    depth_image.at<uint8_t>(i, j) = 255;
                }else if (depth < 0.001)
                {
                    depth_image.at<uint8_t>(i, j) = 255;
                }else{
                    depth_image.at<uint8_t>(i, j) = depth/30*255;
                }
                
            }
        }

        cv::String path = "/home/mt_eb1/LYX/filter_noise_point/Table/"+std::to_string(pose_index)+".png";
        cv::imwrite(path, depth_image);
        pose_index++;
        // for(auto& pointcloud_ptr : ori_pointcloud_list){
        //     pcl::transformPointCloud(*pointcloud_ptr, *pointcloud_ptr, pose_ma.inverse());
        //     // *all_points += *pointcloud_ptr;
        // }

    }
}

void PointCloudCart2Sph::save_pointcloud(){
    pcl::PointCloud<PointT>::Ptr filter_point(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr true_pointcloud(new pcl::PointCloud<PointT>);
    // pcl::PointCloud<PointT>::Ptr all_points_(new pcl::PointCloud<PointT>);
    // for(int pointcloud_index = 0 ; pointcloud_index < ori_pointcloud_list.size();pointcloud_index++){
    //     pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/"+std::to_string(pointcloud_index)+".ply", *ori_pointcloud_list[pointcloud_index]);
    // }
    // mtx.lock();
    // mtx.unlock();
    auto copy_ori_pointcloudIndex_pointIndex_status = ori_pointcloudIndex_pointIndex_status;
    for(auto &vector :copy_ori_pointcloudIndex_pointIndex_status){
        std::sort(vector.begin(), vector.end(),std::greater<int>());
    }
    int count = 0;
    int count_time = 0;
    for(auto &vector :copy_ori_pointcloudIndex_pointIndex_status){
        if(vector[1] >1){
            count+=vector[1];
            count_time++;
        }
        
    }
    count=count/(2*count_time);
    //test
    // count = 0;
    //test
    std::cout<<"count "<<count<<std::endl;
    for(int pointcloud_of_map_ptr_index = 0 ; pointcloud_of_map_ptr_index < ori_pointcloud_list.size();pointcloud_of_map_ptr_index++ ){
        // *all_points_ += *ori_pointcloud_list[pointcloud_of_map_ptr_index];
        for(int point_index = 0 ;point_index < ori_pointcloud_list[pointcloud_of_map_ptr_index]->points.size();point_index++){
            if(ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index] > count){
                filter_point->points.push_back(ori_pointcloud_list[pointcloud_of_map_ptr_index]->points[point_index]);
                // std::cout<<"ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index] "<<ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index]<<std::endl;
            }else{
                true_pointcloud->points.push_back(ori_pointcloud_list[pointcloud_of_map_ptr_index]->points[point_index]);
            }
        }
    }
    for(auto & point : filter_point->points){
        point.r = 255;
        point.g = 0;
        point.b = 0;
    }
    // pcl::transformPointCloud(*filter_point, *filter_point, recover_pose);
    // pcl::transformPointCloud(*true_pointcloud, *true_pointcloud, recover_pose);
    Eigen::Matrix4f save_curr = pose_matrix[50];
    PointT ori_point;
    //请将save_curr位移部分赋值给ori_point的xyz
    ori_point.x = save_curr(0,3);ori_point.y = save_curr(1,3);ori_point.z = save_curr(2,3);ori_point.g =255;
    // all_points_->points.push_back(ori_point);true_pointcloud->points.push_back(ori_point);filter_point->points.push_back(ori_point);
    // all_points_->width = all_points_->points.size();true_pointcloud->width = true_pointcloud->points.size();filter_point->width = filter_point->points.size();
    // all_points_->height = 1;true_pointcloud->height = 1;filter_point->height = 1;
    // pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/all_points.ply", *all_points_);
    // pcl::transformPointCloud(*filter_point, *filter_point, recover_pose);
    //如果/home/mt_eb1/LYX/DataFromServe/HongKongData/MT20241129-100544路径下不存在out文件夹，则创建
    if(!fs::exists(pointcloud_save_path+pointcloud_save_file)){
        fs::create_directory(pointcloud_save_path+pointcloud_save_file);
        std::cout<<"Directory created successfully "<< std::endl;
    }else{
        std::cout << "Directory already exists" << std::endl;
    }
    std::cout<<"filter_point->points.size() "<<filter_point->points.size()<<std::endl;
    filter_point->width = filter_point->points.size();
    filter_point->height = 1;
    pcl::io::savePLYFileASCII(pointcloud_save_path+pointcloud_save_file+"filtered_point.ply", *filter_point);
    filter_point->clear();
    // pcl::transformPointCloud(*true_pointcloud, *true_pointcloud, recover_pose);
    std::cout<<"true_pointcloud->points.size() "<<true_pointcloud->points.size()<<std::endl;
    true_pointcloud->width = true_pointcloud->points.size();
    true_pointcloud->height = 1;
    pcl::io::savePLYFileASCII(pointcloud_save_path+pointcloud_save_file+"true_pointcloud.ply", *true_pointcloud);
    PointInSphGrid->width = PointInSphGrid->points.size();
    PointInSphGrid->height = 1;
    pcl::io::savePLYFileASCII(pointcloud_save_path+pointcloud_save_file+"PointInSphGrid.ply", *PointInSphGrid);
    true_pointcloud->clear();
}
// float PointCloudCart2Sph::get_point_in_sph_theta(const pointXYZRGBHM& point){
//     const float& x = point.xyz.x();
//     const float& y = point.xyz.y();
//     float theta = atan2(y, x);
//     theta = M_PI - theta;
//     return theta;
// }
// float PointCloudCart2Sph::get_point_in_sph_phi(const pointXYZRGBHM& point){
//     const float& x = point.xyz.x();
//     const float& y = point.xyz.y();
//     const float& z = point.xyz.z();
//     float r = sqrt(x*x + y*y + z*z);
//     float phi = acos(z/r);
//     float phi_theta = phi/M_PI*180;
//     // if(phi < 0 )
//     // for(int index=0;index<30;index++)
//     //     std::cout<<"phi "<<phi_theta<<std::endl;
//     return phi;
// }
// void PointCloudCart2Sph:: compute_Sphgrid_depth(std::vector<PointCloudCart2Sph::Sphgrid> &sphgrid){
//     for(auto &grid : sphgrid){
//         if(grid.occupied == false) continue;
//         float min_r = 0;
//         for(auto &point : grid.points){
//             float r = sqrt(point->xyz.x()*point->xyz.x() + point->xyz.y()*point->xyz.y() + point->xyz.z()*point->xyz.z());
//             min_r += r;
//         }
//         grid.depth = min_r/grid.points.size();
        
//     }
// }
// cv::Mat PointCloudCart2Sph::get_depth_image_show() {
//     Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
//     for(int pose_index = 0 ; pose_index < ori_pointcloud_list.size();pose_index++){
//         auto t0 = std::chrono::high_resolution_clock::now();
//         std::cout<<"pose_index "<<pose_index<<std::endl;
//         pose_index = 50;
//         Eigen::Matrix4f& curr_pose = pose_matrix[pose_index];
//         Eigen::Matrix4f relative_pose = curr_pose*last_pose.inverse();
//         last_pose = curr_pose;
//         for(auto& pointcloud_ptr : ori_pointcloud_list){
//             pcl::transformPointCloud(*pointcloud_ptr, *pointcloud_ptr, relative_pose);
//         }
//         auto t1 = std::chrono::high_resolution_clock::now();

//         auto curr_pointcloud_sphgrid = curr_frame_pointcloud_sphgrid(ori_pointcloud_list[pose_index]);

//         auto t2 = std::chrono::high_resolution_clock::now();

//         for(int pointcloud_of_map_ptr_index = 0 ; pointcloud_of_map_ptr_index < ori_pointcloud_list.size();pointcloud_of_map_ptr_index++){

//             auto t3 = std::chrono::high_resolution_clock::now();

//             if(pointcloud_of_map_ptr_index == pose_index) continue;
//             for(int point_index = 0 ;point_index < ori_pointcloud_list[pointcloud_of_map_ptr_index]->points.size();point_index++){

//                 auto t4 = std::chrono::high_resolution_clock::now();

//                 if(ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index] == 0) continue;

//                 PointT point_in_selected_pointcloud = ori_pointcloud_list[pointcloud_of_map_ptr_index]->points[point_index];

//                 float theta = get_point_in_sph_theta(point_in_selected_pointcloud);
//                 float phi = get_point_in_sph_phi(point_in_selected_pointcloud);
//                 int index = index_in_vector_of_sphgrid(theta, phi);

//                 // if(curr_pointcloud_sphgrid[index].occupied == false) continue;

//                 float map_point_depth = get_point_in_sph_r(point_in_selected_pointcloud);
//                 if(map_point_depth < curr_pointcloud_sphgrid[index].depth){
//                     curr_pointcloud_sphgrid[index].depth = map_point_depth;
//                 }
//                 auto t5 = std::chrono::high_resolution_clock::now();
//                 // if((curr_pointcloud_sphgrid[index].depth - map_point_depth)>0 && (curr_pointcloud_sphgrid[index].depth - map_point_depth)<0.03 || (map_point_depth - curr_pointcloud_sphgrid[index].depth)>0 && (map_point_depth - curr_pointcloud_sphgrid[index].depth)<0.03){
//                 //     ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index]=0;
//                 // }

//                 // if((curr_pointcloud_sphgrid[index].depth - map_point_depth)>0.05 ){
//                 //     ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index]=0;
//                 // }

//             }
//             auto t5 = std::chrono::high_resolution_clock::now();
//         }
//         auto t6 = std::chrono::high_resolution_clock::now();
        
//         std::cout<<"transformMap spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t1 - t0).count()<<" ms"<<std::endl;
//         std::cout<<"get curr_frame_pointcloud_sphgrid spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t2 - t1).count()<<" ms"<<std::endl;
//         std::cout<<"Map is deleted virul points by one pointcloud spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t6 - t2).count()<<" ms"<<std::endl;
//         recover_pose = curr_pose;
//     }

// }
// std::vector<PointCloudCart2Sph::Sphgrid> PointCloudCart2Sph::local_pointcloud_sphgrid(){
//     std::vector<PointCloudCart2Sph::Sphgrid> LocalPointcloudSphgrid;
//     for(int i=0;i<phi_size;i++){
//         for(int j=0;j<theta_size;j++){
//             LocalPointcloudSphgrid.push_back(PointCloudCart2Sph::Sphgrid(i, j));
//         }
//     }
//     // int test_num = 0;
//     for(auto &point : pointcloud){

//         float theta = get_point_in_sph_theta(point);
//         float phi = get_point_in_sph_phi(point);
//         int index = index_in_vector_of_sphgrid(theta, phi);

//         LocalPointcloudSphgrid[index].points.push_back(&point);

//         if(LocalPointcloudSphgrid[index].occupied == false){
//             LocalPointcloudSphgrid[index].theta_index = theta;
//             LocalPointcloudSphgrid[index].phi_index = phi;
//             LocalPointcloudSphgrid[index].occupied = true;
//         }
//     }
//     return LocalPointcloudSphgrid;
// }

// std::vector<PointCloudCart2Sph::Sphgrid> PointCloudCart2Sph::curr_frame_pointcloud_sphgrid(){
//     std::vector<PointCloudCart2Sph::Sphgrid> CurrFramePointcloudSphgrid;
//     for(int i=0;i<phi_size;i++){
//         for(int j=0;j<theta_size;j++){
//             CurrFramePointcloudSphgrid.push_back(PointCloudCart2Sph::Sphgrid(i, j));
//         }
//     }
//     for(auto &point : curr_frame_pointcloud){
//         float theta = get_point_in_sph_theta(point);
//         float phi = get_point_in_sph_phi(point);
//         int index = index_in_vector_of_sphgrid(theta, phi);
//         CurrFramePointcloudSphgrid[index].points.push_back(&point);
//         if(CurrFramePointcloudSphgrid[index].occupied == false){
//             CurrFramePointcloudSphgrid[index].theta_index = theta;
//             CurrFramePointcloudSphgrid[index].phi_index = phi;
//             CurrFramePointcloudSphgrid[index].occupied = true;
//         }
//     }
//     compute_Sphgrid_depth(CurrFramePointcloudSphgrid);//计算每个sphgrid的平均深度

//     return CurrFramePointcloudSphgrid;
// }
// void PointCloudCart2Sph::get_curr_frame_pointcloud(pcl::PointCloud<PointT>::Ptr curr_frame,const Eigen::Matrix4f &curr_pose){
//     curr_frame_pointcloud.clear();
//     pcl::transformPointCloud(*curr_frame, *curr_frame, curr_pose);
//     for(auto & point : curr_frame->points){
//         curr_frame_pointcloud.emplace_back(pointXYZRGBHM(point));
//     }
//     // std::cout<<"curr_frame_pointcloud size "<<curr_frame_pointcloud.size()<<std::endl;
// }
// void PointCloudCart2Sph::run(int picture_num){
//     std::vector<PointCloudCart2Sph::Sphgrid> sphgrid = local_pointcloud_sphgrid();
//     // std::cout<<"sphgrid size "<<sphgrid.size()<<std::endl;
//     compute_Sphgrid_depth(sphgrid);
//     std::cout<<"compute_Sphgrid_depth done"<<std::endl;
//     cv::Mat depth_image_show = get_depth_image_show(sphgrid);
//     cv::String path = "/home/mt_eb1/LYX/filter_noise_point/Table/"+std::to_string(picture_num)+"test.png";
//     cv::imwrite(path, depth_image_show);
//     std::cout<<picture_num<<" done "<<std::endl;
//     // cv::waitKey(0);
// }

// void PointCloudCart2Sph::delete_virual_pointcloud(){
//     std::vector<PointCloudCart2Sph::Sphgrid> map_sphgrid = local_pointcloud_sphgrid();
//     std::vector<PointCloudCart2Sph::Sphgrid> curr_frame_sphgrid = curr_frame_pointcloud_sphgrid();
//     // std::cout<<"map_sphgrid size "<<map_sphgrid.size()<<std::endl;
//     // std::cout<<"curr_frame_sphgrid size "<<curr_frame_sphgrid.size()<<std::endl;

//     for(int i=0;i<curr_frame_sphgrid.size();i++){
//         if(curr_frame_sphgrid[i].occupied == true){
//             int curr_frame_sphgrid_poins_size_in_sphgrid = curr_frame_sphgrid[i].points.size();
//             float sum_depth = 0;
//             for(auto &point : curr_frame_sphgrid[i].points){
//                 float r = sqrt(point->xyz.x()*point->xyz.x() + point->xyz.y()*point->xyz.y() + point->xyz.z()*point->xyz.z());
//                 sum_depth += r;
//             }
//             float bar_depth = sum_depth/curr_frame_sphgrid_poins_size_in_sphgrid;

//             for(auto &point : map_sphgrid[i].points){
//                 float r = sqrt(point->xyz.x()*point->xyz.x() + point->xyz.y()*point->xyz.y() + point->xyz.z()*point->xyz.z());
//                 if(r < bar_depth){
//                     point->miss +=1;
//                 }else{
//                     point->hit +=1;
//                 }
//             }
//         }
//     }
// }

// void PointCloudCart2Sph::high_delete_virual_pointcloud(){
//     std::vector<PointCloudCart2Sph::Sphgrid> curr_frame_sphgrid = curr_frame_pointcloud_sphgrid();
//     // int test_num = 0;
//     std::vector<pointXYZRGBHM> save_pointcloud;
//     for(auto &point : pointcloud){

//         float theta = get_point_in_sph_theta(point);
//         float phi = get_point_in_sph_phi(point);
//         int index = index_in_vector_of_sphgrid(theta, phi);

//         if(curr_frame_sphgrid[index].occupied == true){
//             float r = sqrt(point.xyz.x()*point.xyz.x() + point.xyz.y()*point.xyz.y() + point.xyz.z()*point.xyz.z());
//             float curr_frame_sphgrid_depth = curr_frame_sphgrid[index].depth;
//             if((curr_frame_sphgrid_depth - r)>0.01 && (curr_frame_sphgrid_depth - r)<0.06 || (r - curr_frame_sphgrid_depth)>0.01 && (r - curr_frame_sphgrid_depth)<0.06){
//                 filter_pointcloud.emplace_back(point);
//             }else{
//                 save_pointcloud.emplace_back(point);
//             }
//         }
//     }
//     pointcloud = save_pointcloud;


// }

// std::vector<pcl::PointCloud<PointT>::Ptr> PointCloudCart2Sph::get_result_pointcloud(){
//     pcl::PointCloud<PointT>::Ptr result_pointcloud(new pcl::PointCloud<PointT>);
//     pcl::PointCloud<PointT>::Ptr save_pointcloud(new pcl::PointCloud<PointT>);

//     // for(auto& point : pointcloud){
//     //     if(point.hit > 0.5*point.miss){
//     //         PointT point_xyz;
//     //         point_xyz.x = point.xyz.x();
//     //         point_xyz.y = point.xyz.y();
//     //         point_xyz.z = point.xyz.z();
//     //         result_pointcloud->points.push_back(point_xyz);
//     //     }
//     // }

//     //test
//     std::cout<<"pointcloud size "<<pointcloud.size()<<std::endl;    
//     int select_point = 0;
//     int sum_miss = 0;
//     int sum_hit = 0;
//     for(auto& point : pointcloud){
//         int miss_num = point.miss;
//         int hit_num = point.hit;
//         if(miss_num !=0 && hit_num !=0){
//             select_point++;
//             sum_miss += miss_num;
//             sum_hit += hit_num;
//         }
//         if(miss_num > 0.7*hit_num){
//             PointT point_xyz;
//             point_xyz.x = point.xyz.x();
//             point_xyz.y = point.xyz.y();
//             point_xyz.z = point.xyz.z();
//             point_xyz.r = point.rgb(0);
//             point_xyz.g = point.rgb(1);
//             point_xyz.b = point.rgb(2);
//             result_pointcloud->points.push_back(point_xyz);
//         }else{
//             PointT point_xyz;
//             point_xyz.x = point.xyz.x();
//             point_xyz.y = point.xyz.y();
//             point_xyz.z = point.xyz.z();
//             point_xyz.r = point.rgb(0);
//             point_xyz.g = point.rgb(1);
//             point_xyz.b = point.rgb(2);
//             save_pointcloud->points.push_back(point_xyz);
//         }
//     }
//     float bar_miss = float(sum_miss)/float(select_point);
//     float bar_hit = float(sum_hit)/float(select_point);
//     std::cout<<"bar_miss "<<bar_miss<<std::endl;
//     std::cout<<"bar_hit "<<bar_hit<<std::endl;
//     std::cout<<"miss_num > hit_num "<<result_pointcloud->points.size()<<std::endl; 
//     std::vector<pcl::PointCloud<PointT>::Ptr> pointcloud_list;
//     pointcloud_list.push_back(result_pointcloud);
//     pointcloud_list.push_back(save_pointcloud);
//     return pointcloud_list;
// }


int PointCloudCart2Sph:: get_testpoint_sphindex(PointT& test_point,int& pose_num){
    PointInSphGrid->push_back(test_point);
    auto curr_pose = pose_matrix[pose_num];
    Eigen::Matrix<float, 4, 1> test_point_eigen(test_point.x, test_point.y, test_point.z, 1);
    test_point_eigen = curr_pose.inverse()*test_point_eigen;
    test_point.x = test_point_eigen(0);test_point.y = test_point_eigen(1);test_point.z = test_point_eigen(2);test_point.g = 255;
    float theta = get_point_in_sph_theta(test_point);
    float phi = get_point_in_sph_phi(test_point);
    int index = index_in_vector_of_sphgrid(theta, phi);
    return index;
}

void PointCloudCart2Sph:: obvious_point(int test_point_sphindex, PointT local_MapPoint, int& pose_num){
    auto curr_pose = pose_matrix[pose_num];

    float local_MapPoint_theta = get_point_in_sph_theta(local_MapPoint);
    float local_MapPoint_phi = get_point_in_sph_phi(local_MapPoint);
    int local_MapPoint_index = index_in_vector_of_sphgrid(local_MapPoint_theta, local_MapPoint_phi);


    if(local_MapPoint_index == test_point_sphindex){
        PointT world_point(local_MapPoint);
        Eigen::Matrix<float, 4, 1> test_point_eigen(local_MapPoint.x, local_MapPoint.y, local_MapPoint.z, 1);
        test_point_eigen = curr_pose*test_point_eigen;
        world_point.x = test_point_eigen(0);world_point.y = test_point_eigen(1);world_point.z = test_point_eigen(2);
        PointInSphGrid->points.push_back(world_point);
    }
    
}
