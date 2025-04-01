#pragma once

#include <vector>
#include <string>
namespace vanjee
{
    namespace lidar
    {
        struct PointXYZI
        {
            float x;
            float y;
            float z;
            float intensity;
        };
        struct PointXYZHSV
        {
            float x;
            float y;
            float z;
            float h;
            float s;
            float v;
        };
        struct PointXYZIRT
        {
            float x;
            float y;
            float z;
            float intensity;
            uint16_t ring;
            double timestamp;
        };
        template <typename T_Point>
        class PointCloudT
        {

        public:
            typedef T_Point PointT;
            typedef std::vector<PointT> VectorT;
            uint32_t height = 0;   
            uint32_t width = 0;    
            bool is_dense = false; 
            double timestamp;      
            uint32_t seq = 0;      
            VectorT points;
        };
    } // namespace lidar
    
} // namespace vanjee
