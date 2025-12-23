/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For Vanjee LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the Vanjee, nor Wanji Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <pcl/io/io.h>
#include <pcl/point_types.h>

namespace vanjee {
namespace lidar {
typedef ::pcl::Vector3fMap Vector3fMap;

struct PointXYZI : public ::pcl::PointXYZI {
#ifdef ENABLE_GTEST
  uint32_t point_id;
  float hor_angle;
  float ver_angle;
  float distance;
  uint8_t flag;
#endif
  PointXYZI() {
#ifdef ENABLE_GTEST
    point_id = 0;
    hor_angle = 0;
    ver_angle = 0;
    distance = 0;
    flag = 0;
#endif
  }
};

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  double timestamp;
#ifdef ENABLE_GTEST
  uint32_t point_id;
  float hor_angle;
  float ver_angle;
  float distance;
  uint8_t flag;
#endif
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointXYZIRT() {
    x = 0;
    y = 0;
    z = 0;
    data[3] = 1;
    intensity = 0;
    ring = 0;
    timestamp = 0;
#ifdef ENABLE_GTEST
    point_id = 0;
    hor_angle = 0;
    ver_angle = 0;
    distance = 0;
    flag = 0;
#endif
  }
} EIGEN_ALIGN16;

struct PointXYZIRTT {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  double timestamp;
  uint8_t tag;
#ifdef ENABLE_GTEST
  uint32_t point_id;
  float hor_angle;
  float ver_angle;
  float distance;
  uint8_t flag;
#endif
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointXYZIRTT() {
    x = 0;
    y = 0;
    z = 0;
    data[3] = 1;
    intensity = 0;
    ring = 0;
    timestamp = 0;
    tag = 0;
#ifdef ENABLE_GTEST
    point_id = 0;
    hor_angle = 0;
    ver_angle = 0;
    distance = 0;
    flag = 0;
#endif
  }
} EIGEN_ALIGN16;

template <typename T_Point>
class PointCloudT : public ::pcl::PointCloud<T_Point> {
 public:
  typedef T_Point PointT;
  typedef typename ::pcl::PointCloud<T_Point>::VectorType VectorT;
  double timestamp = 0.0;
  uint32_t seq = 0;
};
}  // namespace lidar

}  // namespace vanjee

#ifdef ENABLE_GTEST
POINT_CLOUD_REGISTER_POINT_STRUCT(
  vanjee::lidar::PointXYZIRT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (double, timestamp, timestamp)
  (std::uint32_t, point_id, point_id)
  (float, hor_angle, hor_angle)
  (float, ver_angle, ver_angle)
  (float, distance, distance)
  (std::uint8_t, flag, flag)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(
  vanjee::lidar::PointXYZIRTT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (double, timestamp, timestamp)
  (std::uint8_t, tag, tag)
  (std::uint32_t, point_id, point_id)
  (float, hor_angle, hor_angle)
  (float, ver_angle, ver_angle)
  (float, distance, distance)
  (std::uint8_t,flag, flag)
)

#else

POINT_CLOUD_REGISTER_POINT_STRUCT(
  vanjee::lidar::PointXYZIRT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (double, timestamp, timestamp)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(
  vanjee::lidar::PointXYZIRTT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (double, timestamp, timestamp)
  (std::uint8_t, tag, tag)
)

#endif
