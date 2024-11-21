#include "lx_tools.h"


// typedef unsigned int uint32;

bool LxTools::readSumFile(const std::string & readFile, 
                          std::vector<pcl::PointXYZRGBL> &pclPointsVec, 
                          std::vector<PointXYZRGBI> &pointsVec, 
                          std::vector<Eigen::Matrix4d> &imu_poses, 
                          uint32_t &numberOfIntensity)
{
    int32_t package_num;
    int dropNum = 0;
    char cryptStr1[MD5_SECRET_LEN_16 * 2 + 1];
    numberOfIntensity = 0;

    printf("read lx file: %s\n", readFile.c_str());
    std::ifstream secs_in(readFile.c_str(), std::ios::in | std::ios::binary);
    if (!secs_in.is_open())
        return -1;

    secs_in.read((char *)&package_num, sizeof(package_num));
    printf("data source version is: %d\n", package_num);

    if (package_num == 1)
    {
        section sc;
        while (!secs_in.eof())
        {
            memset((void *)&sc, 0, sizeof(section));
            secs_in.read((char *)&sc.pose, sizeof(sc.pose));
            secs_in.read((char *)&sc.ledia_imp_key, sizeof(sc.ledia_imp_key));
            secs_in.read((char *)&(sc.timestamp), sizeof(sc.timestamp));
            secs_in.read((char *)&sc.status_num, sizeof(sc.status_num));
            secs_in.read((char *)&sc.point_num, sizeof(sc.point_num));

            sc.psp = new PointXYZRGBI[sizeof(PointXYZRGBI) * sc.point_num];
            secs_in.read((char *)sc.psp, sizeof(PointXYZRGBI) * sc.point_num);

            secs_in.read((char *)&sc.cryptStr, sizeof(sc.cryptStr));

            uint32_t r = 0, g = 0, b = 0;
            commonMd5Secret32((char *)sc.psp, sizeof(PointXYZRGBI) * sc.point_num, cryptStr1);
            printf("data source crypt str is %s, read crypt is %s", sc.cryptStr, cryptStr1);
            if (strcmp(cryptStr1, sc.cryptStr) == 0)
            {
                for (int i = 0; i < sc.point_num; i++)
                {
                    pointsVec.push_back(sc.psp[i]);
                    uint32_t rgbi = *reinterpret_cast<uint32_t *>(&sc.psp[i].rgba);
                    r = rgbi & 0xff;
                    g = (rgbi >> 8) & 0xff;
                    b = (rgbi >> 16) & 0xff;

                    if (r == 255 && g == 255 && b == 255)
                        numberOfIntensity++;
                }
            }
            else
            {
                dropNum += sc.point_num;
            }
        }
    }
    else if (package_num == 2)
    {
        section2 sc2;
        int frame_i = 0;
        imu_poses.clear();
        while (!secs_in.eof())
        {
            memset((void *)&sc2, 0, sizeof(section2));
            secs_in.read((char *)&sc2.pose, sizeof(sc2.pose));
            secs_in.read((char *)&sc2.ledia_imp_key, sizeof(sc2.ledia_imp_key));
            // printf("ledia_imp_key is %d\n", sc2.ledia_imp_key);
            secs_in.read((char *)&(sc2.timestamp), sizeof(sc2.timestamp));
            secs_in.read((char *)&sc2.status_num, sizeof(sc2.status_num));
            secs_in.read((char *)&sc2.point_num, sizeof(sc2.point_num));

            
            if (sc2.point_num < 1) 
            {
                printf("Find No point in section %d!\n", frame_i++);
                continue;
            }
            // if (frame_i > 720) break;
            sc2.psp = new PointXYZRGBI[sizeof(PointXYZRGBI) * sc2.point_num];
            secs_in.read((char *)sc2.psp, sizeof(PointXYZRGBI) * sc2.point_num);
            secs_in.read((char *)&sc2.crcRlt, sizeof(sc2.crcRlt));

            Eigen::Matrix4d T(Eigen::Matrix4d::Identity());
            Eigen::Quaterniond q(sc2.pose[6], sc2.pose[3], sc2.pose[4], sc2.pose[5]);
            T.block<3,3>(0,0) = q.toRotationMatrix();
            T.block<3,1>(0,3) = Eigen::Vector3d(sc2.pose[0], sc2.pose[1], sc2.pose[2]);
            imu_poses.push_back(T);
            printf("pose %d is %f, %f, %f, with q %lf %lf %lf %lf, pt num %d\n", frame_i++, 
                    sc2.pose[0], sc2.pose[1], sc2.pose[2],
                    q.x(), q.y(), q.z(), q.w(), sc2.point_num);

            uint32_t r = 0, g = 0, b = 0;
            uint32_t crcRltRead = crc32((char *)(sc2.psp), sizeof(PointXYZRGBI) * sc2.point_num);
            // printf("data source crypt str is %s, read crypt is %s", sc.cryptStr, cryptStr1);
            if (sc2.crcRlt == crcRltRead)
            {
                pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_frame(new pcl::PointCloud<pcl::PointXYZRGBL>);
                for (int i = 0; i < sc2.point_num; i+=1) // TODO: 1/10 downsample only for DEBUG
                {
                    Eigen::Vector3d pt(sc2.psp[i].x, sc2.psp[i].y, sc2.psp[i].z);
                    if ((pt - T.block<3,1>(0,3)).norm() < 1) continue;
                    if ((pt - T.block<3,1>(0,3)).norm() > 8) continue;
                    pointsVec.push_back(sc2.psp[i]);
                    uint32_t rgbi = *reinterpret_cast<uint32_t *>(&sc2.psp[i].rgba);
                    r = rgbi & 0xff;
                    g = (rgbi >> 8) & 0xff;
                    b = (rgbi >> 16) & 0xff;

                    pcl::PointXYZRGBL pcl_point;
                    pcl_point.x = sc2.psp[i].x;
                    pcl_point.y = sc2.psp[i].y;
                    pcl_point.z = sc2.psp[i].z;
                    pcl_point.r = r;
                    pcl_point.g = g;
                    pcl_point.b = b;
                    pcl_point.label = sc2.ledia_imp_key;

                    // if (pcl_point.y < -1.0 )
                    //     continue;
                    // printf("label: %d\n", sc2.ledia_imp_key);
                    pclPointsVec.push_back(pcl_point);
                    cloud_frame->push_back(pcl_point);
                    
                    

                    if (r == 255 && g == 255 && b == 255)
                        numberOfIntensity++;
                }
                cloud_frame_list_.push_back(cloud_frame);
            }
            else
            {
                dropNum += sc2.point_num;
            }
        }
    }
    else
    {
        printf("not support this versions of package.");
        return -1;
    }

    if ((dropNum - 1) >= 1)
    {
        printf("drop point is %d", dropNum);
    }
    
    secs_in.close();
    return true;
}