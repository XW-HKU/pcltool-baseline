#include "PointCloudCart2Sph.h"
#include <pcl/features/normal_3d.h> //For computeCovarianceMatrix

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
    float phi_theta = phi/M_PI*180;
    // if(phi < 0 )

    return phi;
}
float PointCloudCart2Sph::get_point_in_sph_r(const PointT& point){
    
    return sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
}

std::vector<PointCloudCart2Sph::Sphgrid> PointCloudCart2Sph:: curr_frame_pointcloud_sphgrid (pcl::PointCloud<PointT>::Ptr& curr_frame){
    std::vector<PointCloudCart2Sph::Sphgrid> CurrFramePointcloudSphgrid;
    int temp = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < phi_size; i++){
        for(int j = 0; j < theta_size; j++){
            CurrFramePointcloudSphgrid.emplace_back(PointCloudCart2Sph::Sphgrid(0));
        }
        
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    for(auto & point : curr_frame->points){
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
        }
        CurrFramePointcloudSphgrid[index].bar_depth += depth;
        //test
        CurrFramePointcloudSphgrid[index].x += point.x;
        CurrFramePointcloudSphgrid[index].y += point.y;
        CurrFramePointcloudSphgrid[index].z += point.z;
        //test
        CurrFramePointcloudSphgrid[index].size += 1;
        // if(CurrFramePointcloudSphgrid[index].occupied == false){
        //     CurrFramePointcloudSphgrid[index].occupied = true;
        // }
    }
    auto t2 = std::chrono::high_resolution_clock::now();

    for(auto & grid : CurrFramePointcloudSphgrid){
        if(grid.occupied == false) continue;
        grid.bar_depth = grid.bar_depth/grid.size;

        //test
        grid.x = grid.x/grid.size;
        grid.y = grid.y/grid.size;
        grid.z = grid.z/grid.size;
        //test

        // std::cout<<"grid.depth "<<grid.depth<<std::endl;
    }

    auto t3 = std::chrono::high_resolution_clock::now();
    std::cout<<"init CurrFramePointcloudSphgrid spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t1 - t0).count()<<" ms"<<std::endl;
    std::cout<<"fill CurrFramePointcloudSphgrid spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t2 - t1).count()<<" ms"<<std::endl;
    // std::cout<<"compute CurrFramePointcloudSphgrid depth spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t3 - t2).count()<<" ms"<<std::endl;
    return CurrFramePointcloudSphgrid;    

}

void PointCloudCart2Sph::delete_virual_pointcloud(){

    Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
    int stop_index = 210;
    for(int pose_index = 0 ; pose_index < ori_pointcloud_list.size();pose_index++){
        auto t0 = std::chrono::high_resolution_clock::now();
        std::cout<<"pose_index "<<pose_index<<std::endl;
        // int stop_index = 50;
        Eigen::Matrix4f& curr_pose = pose_matrix[pose_index];
        Eigen::Matrix4f relative_pose = curr_pose.inverse()*last_pose;
        last_pose = curr_pose;recover_pose = curr_pose;
        for(auto& pointcloud_ptr : ori_pointcloud_list){
            pcl::transformPointCloud(*pointcloud_ptr, *pointcloud_ptr, relative_pose);
        }
        //test
        PointT my_point;PointT bar_point;pcl::PointCloud<PointT>::Ptr my_point_occupied_map_sph(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr my_point_filter_map_sph(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr my_point_curr_frame_in_same_map_sph(new pcl::PointCloud<PointT>);
        // my_point.x = 0.517215;消防柜边边
        // my_point.y = -0.671407;
        // my_point.z = 0.024059;
        // my_point.x = 0.317446;消防柜
        // my_point.y = -0.839254;
        // my_point.z = 0.039464;
        // my_point.b = 255;
        // my_point.x = 0.213499;//消防柜
        // my_point.y = -0.810174;
        // my_point.z = 0.013973;
        // my_point.b = 255;
        // my_point.x = -0.195496;//水管
        // my_point.y = -1.048251;
        // my_point.z = 0.395797;
        // my_point.b = 255;

        // my_point.x = -0.718042;//电梯侧面
        // my_point.y = 1.673669;
        // my_point.z = 0.012777;
        // my_point.b = 255;

        // my_point.x = -0.624646;//电梯顶部
        // my_point.y = 2.345728;
        // my_point.z = 0.591637;
        // my_point.b = 255;       

        // my_point.x = -0.927345;//直角墙壁
        // my_point.y = 0.053948;
        // my_point.z = 0.690682;
        // my_point.b = 255; 

        // my_point.x = 1.307393;//电梯侧顶部
        // my_point.y = 2.018894;
        // my_point.z = 0.786813;
        // my_point.b = 255; 

        // my_point.x = 1.075589;//门的一侧
        // my_point.y = -0.611792;
        // my_point.z = 0.064304;
        // my_point.b = 255; 

        // my_point.x = 1.881700;//门的左顶侧
        // my_point.y = -0.310064;
        // my_point.z = 0.442763;
        // my_point.b = 255; 

        // my_point.x = -3.091873;//天花板点
        // my_point.y = 0.334648;
        // my_point.z = 0.513744;
        // my_point.b = 255; 

        // my_point.x = -0.774121;//电梯平滑的奇怪点
        // my_point.y = 1.748739;
        // my_point.z = 0.202999;
        // my_point.b = 255; 

        // 11.28 房间的新数据 第200帧
        // my_point.x = -1.850120;
        // my_point.y = -0.583996;
        // my_point.z =  1.145391;
        // my_point.b = 255; 

        // my_point.x = 3.035350;// // 11.28 房间的新数据 第200帧 电梯表面
        // my_point.y = 1.210225;
        // my_point.z = 2.317321;
        // my_point.b = 255; 

        // my_point.x = 3.025894;// // 11.28 房间的新数据 第200帧 墙侧
        // my_point.y = 1.221539;
        // my_point.z =  2.448452;
        // my_point.b = 255; 

        // my_point.x = 2.254071;// // 11.28 房间的新数据 第200帧 拉丝点
        // my_point.y = -0.630593;
        // my_point.z =  2.700840;
        // my_point.b = 255; 

        my_point.x = -0.753579;// // 11.28 房间的新数据 电梯侧面的误杀
        my_point.y = -0.575765;
        my_point.z =  2.101778;
        my_point.b = 255; 

        my_point.x = -0.827977;// // 11.28 房间的新数据 电梯侧面的误杀
        my_point.y = -0.578418;
        my_point.z =  1.942765;
        my_point.b = 255; 
        //因为在世界坐标系下
        bool save_once = false;
        Eigen::Matrix<float, 4, 1> my_point_eigen(my_point.x, my_point.y, my_point.z, 1);
        my_point_eigen = relative_pose*my_point_eigen;
        my_point.x = my_point_eigen(0);my_point.y = my_point_eigen(1);my_point.z = my_point_eigen(2);
        //乘以相对位姿转到当前帧坐标系下

        PointT ori_point;ori_point.x = 0;ori_point.y = 0;ori_point.z = 0;ori_point.g = 255;
        float my_point_theta = get_point_in_sph_theta(my_point);
        float my_point_phi = get_point_in_sph_phi(my_point);
        int my_point_index = index_in_vector_of_sphgrid(my_point_theta, my_point_phi);
        std::cout<<"my_point_index init "<<my_point.x<<" "<<my_point.y<<" "<<my_point.z<<" "<<my_point_index<<std::endl;
        auto curr_pointcloud_sphgrid = curr_frame_pointcloud_sphgrid(ori_pointcloud_list[pose_index]);
        auto t1 = std::chrono::high_resolution_clock::now();
        // //test
        // bool obversed_pixel = false;pcl::PointCloud<PointT>::Ptr point_in_pixel(new pcl::PointCloud<PointT>);
        // //test
        
        // //test
        // for(auto& point : *ori_pointcloud_list[pose_index]){
        //     int point_index = index_in_vector_of_sphgrid(get_point_in_sph_theta(point), get_point_in_sph_phi(point));
            
        //     if(point_index == my_point_index) {
        //         auto temp = point;
        //         temp.r=0;
        //         temp.g=255;
        //         temp.b=0;
        //         my_point_curr_frame_in_same_map_sph->points.push_back(temp);
        //     }
        // }
        // if(abs(curr_pointcloud_sphgrid[my_point_index].x) > 0.0001 && abs(curr_pointcloud_sphgrid[my_point_index].y) > 0.0001 && abs(curr_pointcloud_sphgrid[my_point_index].z) > 0.0001){
        //     my_point.x = curr_pointcloud_sphgrid[my_point_index].x; my_point.y = curr_pointcloud_sphgrid[my_point_index].y; my_point.z = curr_pointcloud_sphgrid[my_point_index].z;
        // }else{
        //     my_point.r=0;my_point.g=0;my_point.b=255;//若当前帧有点坐落于该区域则为当前帧的均值点，否则为选定的点
        // }
        // my_point_curr_frame_in_same_map_sph->push_back(my_point);
        // //test


        //test

        // std::cout<<" curr_pointcloud_sphgrid "<<curr_pointcloud_sphgrid.size()<<std::endl;
        // curr_pointcloud_sphgrid[my_point_index].depth =sqrt(my_point.x*my_point.x + my_point.y*my_point.y + my_point.z*my_point.z);
        auto t2 = std::chrono::high_resolution_clock::now();
        //test

        //test
        // for(int pointcloud_of_map_ptr_index = 0 ; pointcloud_of_map_ptr_index < ori_pointcloud_list.size();pointcloud_of_map_ptr_index++){
        //     if(pointcloud_of_map_ptr_index == pose_index) continue;
        //     for(int point_index = 0 ;point_index < ori_pointcloud_list[pointcloud_of_map_ptr_index]->points.size();point_index++){
        //         PointT point_in_selected_pointcloud = ori_pointcloud_list[pointcloud_of_map_ptr_index]->points[point_index];

        //         float theta = get_point_in_sph_theta(point_in_selected_pointcloud);
        //         float phi = get_point_in_sph_phi(point_in_selected_pointcloud);
        //         int index = index_in_vector_of_sphgrid(theta, phi);
        //         if(index == my_point_index) my_point_occupied_map_sph->points.push_back(point_in_selected_pointcloud);                
        //     }
        // }
        // //test


        for(int pointcloud_of_map_ptr_index = 0 ; pointcloud_of_map_ptr_index < ori_pointcloud_list.size();pointcloud_of_map_ptr_index++){

            auto t3 = std::chrono::high_resolution_clock::now();

            if(pointcloud_of_map_ptr_index == pose_index) continue;
            //获取当前map的点云
            for(int point_index = 0 ;point_index < ori_pointcloud_list[pointcloud_of_map_ptr_index]->points.size();point_index++){

                auto t4 = std::chrono::high_resolution_clock::now();

                // if(ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index] == 0) continue;
                const PointT& point_in_selected_pointcloud = ori_pointcloud_list[pointcloud_of_map_ptr_index]->points[point_index];

                float theta = get_point_in_sph_theta(point_in_selected_pointcloud);
                float phi = get_point_in_sph_phi(point_in_selected_pointcloud);
                int index = index_in_vector_of_sphgrid(theta, phi);
                //////////////////////////////
                // if(index == my_point_index) my_point_occupied_map_sph->points.push_back(point_in_selected_pointcloud);
                if(curr_pointcloud_sphgrid[index].occupied == false) continue;
                PointT curr_frame_pointcloud_sphgrid_bar_point;
                curr_frame_pointcloud_sphgrid_bar_point.x = curr_pointcloud_sphgrid[index].x;
                curr_frame_pointcloud_sphgrid_bar_point.y = curr_pointcloud_sphgrid[index].y;
                curr_frame_pointcloud_sphgrid_bar_point.z = curr_pointcloud_sphgrid[index].z;
                float depth_curr_frame_pointcloud_sphgrid_bar_point = get_point_in_sph_r(curr_frame_pointcloud_sphgrid_bar_point);
                // std::cout<<"curr_pointcloud_sphgrid[index].size "<<curr_pointcloud_sphgrid[index].size<<std::endl;  
                //test
                // if(curr_pointcloud_sphgrid[index].size < 1) continue;
                // std::cout<<"curr_pointcloud_sphgrid[index].size "<<curr_pointcloud_sphgrid[index].size<<std::endl;
                //test
                float map_point_depth = get_point_in_sph_r(point_in_selected_pointcloud);
                auto t5 = std::chrono::high_resolution_clock::now();
                // if((curr_pointcloud_sphgrid[index].depth - map_point_depth)>0 && (curr_pointcloud_sphgrid[index].depth - map_point_depth)<0.03 || (map_point_depth - curr_pointcloud_sphgrid[index].depth)>0 && (map_point_depth - curr_pointcloud_sphgrid[index].depth)<0.03){
                //     ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index]=0;
                // }
                float depth_diff = curr_pointcloud_sphgrid[index].max_depth - curr_pointcloud_sphgrid[index].min_depth;
                float map_point_depth_diff = map_point_depth - curr_pointcloud_sphgrid[index].min_depth;
                float map_point_depth_diff_bar = curr_pointcloud_sphgrid[index].bar_depth - map_point_depth;
                if(map_point_depth_diff < 0 && map_point_depth_diff_bar > 0.05 && map_point_depth_diff_bar < 0.5){
                    // //test
                    // if(index == my_point_index){
                    
                    // my_point_filter_map_sph->points.push_back(point_in_selected_pointcloud);
                    // } 
                    // //test

                    //new funcion
                    std::vector<int> searched_pixel_index;
                    // float search_radius = 0.18*(log((0.1*depth_curr_frame_pointcloud_sphgrid_bar_point*depth_curr_frame_pointcloud_sphgrid_bar_point+1/exp(1))*exp(1)));
                    int search_pixel_range = 9;
                    int point_in_selected_pointcloud_theta_index = get_point_in_sph_theta_index(point_in_selected_pointcloud);
                    int point_in_selected_pointcloud_phi_index = get_point_in_sph_phi_index(point_in_selected_pointcloud);
                    for(int i = -search_pixel_range; i < search_pixel_range+1; i++){
                        for(int j = -search_pixel_range; j < search_pixel_range+1; j++){
                            int theta_index = point_in_selected_pointcloud_theta_index + i;
                            int phi_index = point_in_selected_pointcloud_phi_index + j;
                            
                            if(theta_index < 0 || theta_index >= theta_size || phi_index < 0 || phi_index >= phi_size) continue;
                            int pixel_index = phi_index*theta_size + theta_index;
                            searched_pixel_index.emplace_back(pixel_index);
                        }
                    }
                    // std::cout<<"searched_pixel_index.size() "<<searched_pixel_index.size()<<std::endl;
                    //new funcion
                    bool is_delete = true;
                    int pass_time = 0;
                    for(auto & pixel_index : searched_pixel_index){
                        if(curr_pointcloud_sphgrid[pixel_index].occupied == false){
                            pass_time++;
                            continue;
                        }
                        float depth_ = curr_pointcloud_sphgrid[pixel_index].bar_depth;
                        if(depth_ < map_point_depth){
                            is_delete = false;
                            break;
                        }
                    }
                    if(pass_time == searched_pixel_index.size()) is_delete = false;
                    if(is_delete){
                        ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index]+=1;
                        if(my_point_index == index){
                            my_point_filter_map_sph->points.push_back(point_in_selected_pointcloud);
                        }
                    }

                    // if(my_point_index == index && obversed_pixel == false){
                    //     std::cout<<"obversed_pixel my_point_index "<<my_point_index<<std::endl;
                    //     obversed_pixel = true;
                    //     std::cout<<"search_pixel_range "<<search_pixel_range<<std::endl; 
                    //     for(auto & pixel_index : searched_pixel_index){
                    //         if(curr_pointcloud_sphgrid[pixel_index].occupied == false) continue;
                    //         std::cout<<"pixel_index "<<pixel_index<<std::endl;
                    //         PointT temp;
                    //         temp.x = curr_pointcloud_sphgrid[pixel_index].x;
                    //         temp.y = curr_pointcloud_sphgrid[pixel_index].y;
                    //         temp.z = curr_pointcloud_sphgrid[pixel_index].z;
                    //         Eigen::Matrix<float, 4, 1> temp_point_Eigen(temp.x, temp.y, temp.z, 1);
                    //         auto temp_point = recover_pose*temp_point_Eigen;
                    //         std::cout<<"temp.x "<<temp_point[0]<<" temp.y "<<temp_point[1]<<" temp.z "<<temp_point[2]<<" curr_pointcloud_sphgrid[pixel_index] "<<curr_pointcloud_sphgrid[pixel_index].bar_depth<<std::endl;
                    //         point_in_pixel->points.push_back(temp);
                    //     }
                    //     // PointT temp;
                    //     // temp.x = my_point.x;
                    //     // temp.y = my_point.y;
                    //     // temp.z = my_point.z;
                    //     // point_in_pixel->points.push_back(temp);
                    //     pcl::transformPointCloud(*point_in_pixel, *point_in_pixel, recover_pose);
                    //     pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/my_point/point_in_pixel.ply", *point_in_pixel);                   
                    // }


                }
                //恢复sphgrid_index_of_MapPoint_AND_MapPoint_coord中非最近的点

            }
            auto t5 = std::chrono::high_resolution_clock::now();
        }
        auto t6 = std::chrono::high_resolution_clock::now();
        
        std::cout<<"transformMap spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t1 - t0).count()<<" ms"<<std::endl;
        std::cout<<"get curr_frame_pointcloud_sphgrid spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t2 - t1).count()<<" ms"<<std::endl;
        std::cout<<"Map is deleted virul points by one pointcloud spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t6 - t2).count()<<" ms"<<std::endl;
        // if(pose_index == stop_index){
        //     break;
        // }


        //test
        // if(pose_index == stop_index){
        //     pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>);
        //     pcl::transformPointCloud(*my_point_curr_frame_in_same_map_sph, *temp, recover_pose);
        //     pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/my_point_curr_frame_in_same_map_sph_noRayPath.ply", *temp);
        //     temp->clear();
        //     pcl::transformPointCloud(*ori_pointcloud_list[pose_index], *temp, Eigen::Matrix4f::Identity());
        //     pcl::PointCloud<PointT>::Ptr pose_point(new pcl::PointCloud<PointT>);
        //     PointT poseposepoint;
        //     poseposepoint.x = recover_pose(0,3);
        //     poseposepoint.y = recover_pose(1,3);
        //     poseposepoint.z = recover_pose(2,3);
        //     poseposepoint.r = 0;
        //     poseposepoint.g = 255;
        //     poseposepoint.b = 0;
        //     pose_point->points.push_back(poseposepoint);

        //     int dPointxNum = 1000;
        //     float dx = (my_point.x-ori_point.x)/dPointxNum;float dy = (my_point.y-ori_point.y)/dPointxNum;float dz = (my_point.z-ori_point.z)/dPointxNum;
        //     PointT start_point = ori_point;
        //     start_point.r =120;start_point.g = 120;start_point.b = 120;
        //     my_point_curr_frame_in_same_map_sph->push_back(start_point);
        //     std::cout<<"my_point.x "<<my_point.x<<" my_point.y "<<my_point.y<<" my_point.z "<<my_point.z<<std::endl;
        //     std::cout<<"ori_point.x "<<ori_point.x<<" ori_point.y "<<ori_point.y<<" ori_point.z "<<ori_point.z<<std::endl;
        //     for(int i = 0; i < dPointxNum; i++){
        //         start_point.x += dx;start_point.y += dy;start_point.z += dz;
        //         auto temp = start_point;
        //         temp.r = 0;
        //         temp.g = 0;
        //         temp.b = 255;
        //         my_point_curr_frame_in_same_map_sph->points.push_back(temp);
        //     }

        //     pcl::transformPointCloud(*temp, *temp, recover_pose);
        //     pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/first_frame.ply", *temp);
        //     pcl::transformPointCloud(*pose_point, *pose_point, recover_pose);
        //     pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/pose_point.ply", *pose_point);

        //     for(auto & point : my_point_occupied_map_sph->points){
        //         point.r = 255;
        //         point.g = 255;
        //         point.b = 255;
        //     }
        //     pcl::transformPointCloud(*my_point_occupied_map_sph, *my_point_occupied_map_sph, recover_pose);
        //     pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/my_point/my_point_occupied_map_sph.ply", *my_point_occupied_map_sph);

        //     // my_point_filter_map_sph->push_back(my_point);
        //     std::cout<<"my_point_filter_map_sph size "<<my_point_filter_map_sph->points.size()<<std::endl;
        //     my_point_filter_map_sph->width = my_point_filter_map_sph->points.size();
        //     my_point_filter_map_sph->height = 1;
        //     for(auto & point : my_point_filter_map_sph->points){
        //         point.r = 255;
        //         point.g = 0;
        //         point.b = 0;
        //     }
        //     pcl::transformPointCloud(*my_point_filter_map_sph, *my_point_filter_map_sph, recover_pose);
        //     pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/my_point/my_point_filter_map_sph.ply", *my_point_filter_map_sph);
        //     my_point_curr_frame_in_same_map_sph->width = my_point_curr_frame_in_same_map_sph->points.size();
        //     std::cout<<"my_point_curr_frame_in_same_map_sph size "<<my_point_curr_frame_in_same_map_sph->points.size()<<std::endl;
        //     my_point_curr_frame_in_same_map_sph->height = 1;
        //     // for(auto & point : *my_point_curr_frame_in_same_map_sph){
        //     //     point.r = 0;
        //     //     point.g = 255;
        //     //     point.b = 0;
        //     //     std::cout<<"point.x "<<point.x<<" point.y "<<point.y<<" point.z "<<point.z<<std::endl;
        //     // }
        //     pcl::transformPointCloud(*my_point_curr_frame_in_same_map_sph, *my_point_curr_frame_in_same_map_sph, recover_pose);
        //     pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/my_point/my_point_curr_frame_in_same_map_sph.ply", *my_point_curr_frame_in_same_map_sph);
        //     break;
        // }
        //test

        // if(pose_index == stop_index){
        //     break;
        // }   
    }

    save_pointcloud();
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
    pcl::PointCloud<PointT>::Ptr all_points_(new pcl::PointCloud<PointT>);
    // for(int pointcloud_index = 0 ; pointcloud_index < ori_pointcloud_list.size();pointcloud_index++){
    //     pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/"+std::to_string(pointcloud_index)+".ply", *ori_pointcloud_list[pointcloud_index]);
    // }
    for(int pointcloud_of_map_ptr_index = 0 ; pointcloud_of_map_ptr_index < ori_pointcloud_list.size();pointcloud_of_map_ptr_index++ ){
        *all_points_ += *ori_pointcloud_list[pointcloud_of_map_ptr_index];
        for(int point_index = 0 ;point_index < ori_pointcloud_list[pointcloud_of_map_ptr_index]->points.size();point_index++){
            if(ori_pointcloudIndex_pointIndex_status[pointcloud_of_map_ptr_index][point_index] > 2){
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
    PointT ori_point;
    ori_point.x = 0;ori_point.y = 0;ori_point.z = 0;ori_point.g = 255;
    all_points_->points.push_back(ori_point);true_pointcloud->points.push_back(ori_point);filter_point->points.push_back(ori_point);
    all_points_->width = all_points_->points.size();true_pointcloud->width = true_pointcloud->points.size();filter_point->width = filter_point->points.size();
    all_points_->height = 1;true_pointcloud->height = 1;filter_point->height = 1;
    // pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/all_points.ply", *all_points_);
    pcl::transformPointCloud(*filter_point, *filter_point, recover_pose);
    pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/filter_point>0.10.ply", *filter_point);
    pcl::transformPointCloud(*true_pointcloud, *true_pointcloud, recover_pose);
    pcl::io::savePLYFileASCII("/home/mt_eb1/LYX/filter_noise_point/Table/true_pointcloud.ply", *true_pointcloud);
     
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

