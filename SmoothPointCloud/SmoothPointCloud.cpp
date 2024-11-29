#include"SmoothPointCloud.h"

void SmoothPointCloud :: smooth_pointcloud(std::vector<pcl::PointCloud<PointT>::Ptr> &pointclouds_list){
    std::vector<pcl::PointCloud<PointT>::Ptr> out;
    pcl::PointCloud<PointT>::Ptr all_points(new pcl::PointCloud<PointT>);
    pcl::KdTreeFLANN<PointT> kdtree;float nearest_K_Search = 0.03;float curr_point_weight = 0.1; auto others_weight = 1-curr_point_weight;
    auto t0 = std::chrono::high_resolution_clock::now();
    for(int pointcloud_index = 0 ;pointcloud_index < pointclouds_list.size();pointcloud_index++){
        *all_points += *pointclouds_list[pointcloud_index];
    }
    kdtree.setInputCloud(all_points);
    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout<<"kdtree spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t1 - t0).count()<<" ms"<<std::endl;
    for(const auto & pointcloud : pointclouds_list){
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        auto t2 = std::chrono::high_resolution_clock::now();
        for(auto & point : pointcloud->points){
            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;
            if(kdtree.radiusSearch(point, nearest_K_Search, pointIdxNKNSearch, pointNKNSquaredDistance) > 1){
                PointT temp_point;
                for(int i = 0; i < pointIdxNKNSearch.size(); i++){
                    temp_point.x += all_points->points[pointIdxNKNSearch[i]].x;
                    temp_point.y += all_points->points[pointIdxNKNSearch[i]].y;
                    temp_point.z += all_points->points[pointIdxNKNSearch[i]].z;

                }
                //均值点
                temp_point.x = temp_point.x/(pointIdxNKNSearch.size());
                temp_point.y = temp_point.y/(pointIdxNKNSearch.size());
                temp_point.z = temp_point.z/(pointIdxNKNSearch.size());


                //加权平均
                temp_point.x = curr_point_weight*point.x + others_weight*temp_point.x;
                temp_point.y = curr_point_weight*point.y + others_weight*temp_point.y;
                temp_point.z = curr_point_weight*point.z + others_weight*temp_point.z;
                temp_point.r = point.r;temp_point.g = point.g;temp_point.b = point.b;
                cloud->points.push_back(temp_point);
            }else{
                cloud->points.push_back(point);
            }
        }
        out.push_back(cloud);
        auto t3 = std::chrono::high_resolution_clock::now();
        std::cout<<"smooth a pointcloud spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t3 - t2).count()<<" ms"<<std::endl;
    }
    pointclouds_list = out;


    // std::vector<pcl::PointCloud<PointT>::Ptr> out;
    // pcl::PointCloud<PointT>::Ptr all_points(new pcl::PointCloud<PointT>);
    // pcl::KdTreeFLANN<PointT> kdtree;int nearest_K_Search = 5;float curr_point_weight = 0.6;
    // auto t0 = std::chrono::high_resolution_clock::now();
    // for(int pointcloud_index = 0 ;pointcloud_index < pointclouds_list.size();pointcloud_index++){
    //     *all_points += *pointclouds_list[pointcloud_index];
    // }
    // kdtree.setInputCloud(all_points);
    // auto t1 = std::chrono::high_resolution_clock::now();
    // std::cout<<"kdtree spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t1 - t0).count()<<" ms"<<std::endl;
    // for(const auto & pointcloud : pointclouds_list){
    //     pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    //     auto t2 = std::chrono::high_resolution_clock::now();
    //     for(auto & point : pointcloud->points){
    //         std::vector<int> pointIdxNKNSearch(1);
    //         std::vector<float> pointNKNSquaredDistance(1);
    //         if(kdtree.nearestKSearch(point, nearest_K_Search, pointIdxNKNSearch, pointNKNSquaredDistance) > 1){
    //             PointT temp_point;
    //             for(int i = 1; i < nearest_K_Search; i++){
    //                 temp_point.x += all_points->points[pointIdxNKNSearch[i]].x;
    //                 temp_point.y += all_points->points[pointIdxNKNSearch[i]].y;
    //                 temp_point.z += all_points->points[pointIdxNKNSearch[i]].z;

    //             }
    //             //均值点
    //             temp_point.x = temp_point.x/(nearest_K_Search-1);
    //             temp_point.y = temp_point.y/(nearest_K_Search-1);
    //             temp_point.z = temp_point.z/(nearest_K_Search-1);


    //             //加权平均
    //             temp_point.x = curr_point_weight*point.x + (1-curr_point_weight)*temp_point.x;
    //             temp_point.y = curr_point_weight*point.y + (1-curr_point_weight)*temp_point.y;
    //             temp_point.z = curr_point_weight*point.z + (1-curr_point_weight)*temp_point.z;
    //             temp_point.r = point.r;temp_point.g = point.g;temp_point.b = point.b;
    //             cloud->points.push_back(temp_point);
    //         }else{
    //             cloud->points.push_back(point);
    //         }
    //     }
    //     out.push_back(cloud);
    //     auto t3 = std::chrono::high_resolution_clock::now();
    //     std::cout<<"smooth a pointcloud spend time "<<std::chrono::duration_cast<std::chrono::milliseconds> (t3 - t2).count()<<" ms"<<std::endl;
    // }
    // pointclouds_list = out;
}