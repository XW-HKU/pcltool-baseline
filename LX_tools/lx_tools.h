#ifndef LXFILTER_H
#define LXFILTER_H
//#ifndef CC_LX_FILTER_HEADER
//#define CC_LX_FILTER_HEADER

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <openssl/md5.h>
#include <fstream>
#include <sstream>
#include <iostream>

// typedef unsigned int uint32;

#ifndef MD5_SECRET_LEN_16
    #define MD5_SECRET_LEN_16     (16)
#endif

#ifndef MD5_BYTE_STRING_LEN
    #define MD5_BYTE_STRING_LEN   (4)
#endif

//! LX format types
static const char e_lx_type_names[][12]= {
        "LX_FLOAT32", "LX_FLOAT64",
        "LX_UCHAR", "LX_FLOAT"
};

//! LX format storage modes
static const char e_lx_storage_mode_names[][24]={"LX_BIG_ENDIAN","LX_LITTLE_ENDIAN"};

/* ply format mode type */
typedef enum e_lx_storage_mode_ {
    LX_BIG_ENDIAN,
    LX_LITTLE_ENDIAN,
    LX_ASCII,
    LX_DEFAULT      /* has to be the last in enum */
} e_lx_storage_mode; /* order matches ply_storage_mode_list */

/* ply data type */
typedef enum e_lx_type {
    LX_FLOAT32, LX_FLOAT64,
    LX_FLOAT, LX_DOUBLE,
} e_lx_type;

class PointXYZRGBI
{
public:
    float x;
    float y;
    float z;
    float rgba;
};

class section{
    public:
        // tx, ty, tz, qx, qy, qz, qw
        float pose[7];
        // the number of ledia keyframe
        int32_t ledia_imp_key;
        // timestamp
        double timestamp;
        // 0,1 store GPS、sensor healty status
        int32_t status_num;
        // contains points per section
        int32_t point_num;

        // points data    = new PointXYZRGBI[point_num]
        PointXYZRGBI* psp;
        // the points MD5 str
        char cryptStr[MD5_SECRET_LEN_16 * 2 + 1];
};

class section2 {
    public:
        // tx, ty, tz, qx, qy, qz, qw
        float pose[7];
        // the number of ledia keyframe
        int32_t ledia_imp_key;
        // timestamp
        double timestamp;
        // 0,1 store GPS、sensor healty status
        int32_t status_num;
        // contains points per section
        int32_t point_num;
        // points data    = new PointXYZRGBI[point_num]
        PointXYZRGBI* psp;
        // the points crc
        int32_t crcRlt;
};

//! Stanford LX file I/O filter
class LxTools
{
public:
    LxTools(){};

    void commonMd5Secret32(const char *src, size_t srcSize, char *result)
    {
        MD5_CTX ctx;
        static std::string md5String;
        if (md5String != "")
        {
            md5String = "";
        }
        unsigned char md[MD5_SECRET_LEN_16] = {0};
        char tmp[MD5_BYTE_STRING_LEN] = {0};

        MD5_Init(&ctx);
        MD5_Update(&ctx, src, srcSize);
        MD5_Final(md, &ctx);

        for (int i = 0; i < 16; ++i)
        {
            memset(tmp, 0x00, sizeof(tmp));
            snprintf(tmp, sizeof(tmp), "%02X", md[i]);
            md5String += tmp;
        }
        strcpy(result, md5String.c_str());
    }

    uint32_t crc32(const char *src, size_t length)
    {
        uint32_t crc = 0xFFFFFFFF;
        uint32_t crcTable[256] = {0};
        for (uint32_t i = 0; i < 256; i++)
        {
            uint32_t c = i;
            for (int j = 0; j < 8; j++)
            {
                if (c & 1)
                {
                    c = 0xEDB88320L ^ (c >> 1);
                }
                else
                {
                    c = c >> 1;
                }
            }
            crcTable[i] = c;
        }
        crc = 0xFFFFFFFF;
        for (size_t i = 0; i < length; i++)
        {
            crc = crcTable[(crc ^ src[i]) & 0xFF] ^ (crc >> 8);
        }
        return crc ^ 0xFFFFFFFF;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> cloud_frame_list_;

    bool readSumFile(const std::string & readFile, 
                     std::vector<pcl::PointXYZRGBL> &pclPointsVec,
                     std::vector<PointXYZRGBI> &pointsVec, 
                     std::vector<Eigen::Matrix4d> & imu_poses, 
                     unsigned &numberOfIntensity_);

    //static accessors
    // static void SetDefaultOutputFormat(e_lx_storage_mode format);

    //inherited from FileIOFilter
    // void loadFile(const std::tring& filename, ccHObject& container, LoadParameters& parameters);

//     bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
//     CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;

//     //! Custom loading method
//     CC_FILE_ERROR loadFile(const QString& filename, const QString& textureFilename, ccHObject& container, LoadParameters& parameters);

// private:
//     //! Internal method
//     CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename, e_lx_storage_mode storageType);
};

#endif // CC_LX_FILTER_HEADER
