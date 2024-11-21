#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv)
{
    // Check input arguments
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " input.ply voxel_size" << std::endl;
        return -1;
    }

    // Read parameters
    std::string input_filename = argv[1];
    float voxel_size = std::stof(argv[2]);

    // Create point cloud pointers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Load PLY point cloud file
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(input_filename, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", input_filename.c_str());
        return -1;
    }

    std::cout << "Original point cloud has " << cloud->points.size() << " points." << std::endl;

    // Create the voxel grid filter object
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    // Set the voxel size
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud_filtered);

    std::cout << "Filtered point cloud has " << cloud_filtered->points.size() << " points." << std::endl;

    // Generate output filename
    size_t dot_pos = input_filename.find_last_of('.');
    std::string base_name;
    std::string extension;
    if (dot_pos == std::string::npos)
    {
        base_name = input_filename;
        extension = "";
    }
    else
    {
        base_name = input_filename.substr(0, dot_pos);
        extension = input_filename.substr(dot_pos);
    }

    // Format voxel size, keeping three decimal places
    std::ostringstream ss;
    ss << "_voxel" << std::fixed << std::setprecision(3) << voxel_size;
    std::string voxel_suffix = ss.str();

    std::string output_filename = base_name + voxel_suffix + extension;

    // Save the downsampled point cloud
    pcl::io::savePLYFile(output_filename, *cloud_filtered);

    std::cout << "Downsampling completed, saved as " << output_filename << std::endl;

    return 0;
}
