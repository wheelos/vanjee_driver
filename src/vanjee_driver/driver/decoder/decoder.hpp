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

3. Neither the names of the Vanjee, nor Suteng Innovation Technology, nor the
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
#include <vanjee_driver/common/error_code.hpp>
#include <vanjee_driver/driver/decoder/basic_attr.hpp>
#include <vanjee_driver/driver/decoder/chan_angles.hpp>
#include <vanjee_driver/driver/decoder/imu_calibration_param.hpp>
#include <vanjee_driver/driver/decoder/member_checker.hpp>
#include <vanjee_driver/driver/decoder/section.hpp>
#include <vanjee_driver/driver/decoder/split_strategy.hpp>
#include <vanjee_driver/driver/decoder/trigon.hpp>
#include <vanjee_driver/driver/difop/difop_base.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/driver_param.hpp>
#include <vanjee_driver/msg/imu_packet.hpp>
#include <vanjee_driver/msg/scan_data_msg.hpp>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#ifdef ENABLE_TRANSFORM
#include <Eigen/Dense>
#endif

#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct {
  uint16_t start_angle;
  uint16_t end_angle;
} WJFOV;
typedef struct {
  uint8_t sync_mode;
  uint8_t sync_sts;
  WJTimestampUTC timestamp;
} WJTimeInfo;

#pragma pack(pop)

enum WJEchoMode { ECHO_SINGLE = 0, ECHO_DUAL };

struct WJDecoderConstParam {
  uint16_t MSOP_LEN;

  uint16_t LASER_NUM;
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;

  double DISTANCE_MIN;
  double DISTANCE_MAX;
  double DISTANCE_RES;
  double TEMPERATURE_RES;
};

typedef struct _HideAngleParams {
  float min_angle;
  float max_angle;
} HideAngleParams;

typedef struct _HideDistanceParams {
  float min_distance;
  float max_distance;
} HideDistanceParams;

typedef struct _HideRangeParams {
  uint16_t hide_point_channel_id;
  std::vector<HideAngleParams> hide_point_angle;
  std::vector<HideDistanceParams> hide_point_distance;
} HideRangeParams;

#define INIT_ONLY_ONCE()         \
  static bool init_flag = false; \
  if (init_flag)                 \
    return param;                \
  init_flag = true;

/// @brief Analyze the MSOP/DIFOP packet of the lidar to obtain a point cloud.
template <typename T_PointCloud>
class Decoder {
 public:
#ifdef ENABLE_TRANSFORM
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size) = 0;
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
    WJ_INFO << "Default processDifop." << WJ_REND;
  };
  virtual ~Decoder() = default;
  bool processMsopPkt(const uint8_t *pkt, size_t size);

  explicit Decoder(const WJDecoderConstParam &const_param, const WJDecoderParam &param);
  /// @brief Calculate the time interval for data blocks
  double getPacketDuration();
  double prevPktTs();
  void transformPoint(float &x, float &y, float &z);

  std::vector<std::string> split(const std::string &s, char delimiter);
  void hidePiontsParamsLoad();
  bool isValueInRange(uint16_t channel, float angle, float distance, const std::vector<HideRangeParams> &ranges);

  void regCallback(const std::function<void(const Error &)> &cb_excep, const std::function<void(uint16_t, double)> &cb_split_frame,
                   const std::function<void(void)> &cb_imu_pkt = nullptr, const std::function<void(double)> &cb_scan_data = nullptr,
                   const std::function<void(double)> &cb_device_ctrl_state = nullptr);

  void regGetDifoCtrlDataInterface(std::shared_ptr<std::map<uint16, GetDifoCtrlClass>> get_difo_ctrl_map_ptr);

  std::shared_ptr<T_PointCloud> point_cloud_;
  std::shared_ptr<ImuPacket> imu_packet_;
  std::shared_ptr<ScanData> scan_data_;
  std::shared_ptr<DeviceCtrl> device_ctrl_;

 public:
  double cloudTs();
  uint8_t checkBCC(const uint8_t *pkt, size_t start_index, size_t end_index);
  uint16_t checkCRC16(const uint8_t *pkt, size_t start_index, size_t end_index);

  WJDecoderConstParam const_param_;
  WJDecoderParam param_;
  Imu_Calibration_Param imu_calibration_param_;
  std::function<void(uint16_t, double)> cb_split_frame_;
  std::function<void(void)> cb_imu_pkt_;
  std::function<void(double)> cb_scan_data_;
  std::function<void(const Error &)> cb_excep_;
  std::function<void(double)> cb_device_ctrl_state_;
  std::shared_ptr<std::map<uint16, GetDifoCtrlClass>> get_difo_ctrl_map_ptr_;

#ifdef ENABLE_TRANSFORM
  Eigen::Matrix4d trans_;
#endif

  Trigon trigon_;
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

  double packet_duration_;
  DistanceSection distance_section_;
  Projection projection;

  WJEchoMode echo_mode_;
  bool angles_ready_;
  double prev_pkt_ts_;
  double first_point_ts_;
  double last_point_ts_;
  uint16_t first_line_id_;

  bool hide_range_flag_;
  std::vector<HideRangeParams> hide_range_params_;
};

template <typename T_PointCloud>
void Decoder<T_PointCloud>::regGetDifoCtrlDataInterface(std::shared_ptr<std::map<uint16, GetDifoCtrlClass>> get_difo_ctrl_map_ptr) {
  get_difo_ctrl_map_ptr_ = get_difo_ctrl_map_ptr;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::regCallback(const std::function<void(const Error &)> &cb_excep,
                                               const std::function<void(uint16_t, double)> &cb_split_frame,
                                               const std::function<void(void)> &cb_imu_pkt, const std::function<void(double)> &cb_scan_data,
                                               const std::function<void(double)> &cb_device_ctrl_state) {
  cb_excep_ = cb_excep;
  cb_split_frame_ = cb_split_frame;
  cb_imu_pkt_ = cb_imu_pkt;
  cb_scan_data_ = cb_scan_data;
  cb_device_ctrl_state_ = cb_device_ctrl_state;
}

template <typename T_PointCloud>
inline Decoder<T_PointCloud>::Decoder(const WJDecoderConstParam &const_param, const WJDecoderParam &param)
    : const_param_(const_param),
      param_(param),
      packet_duration_(0),
      distance_section_(const_param.DISTANCE_MIN, const_param.DISTANCE_MAX, param.min_distance, param.max_distance),
      echo_mode_(ECHO_SINGLE),
      angles_ready_(false),
      prev_pkt_ts_(0.0),
      first_point_ts_(0.0),
      last_point_ts_(0.0),
      first_line_id_(1),
      hide_range_flag_(false) {
  hidePiontsParamsLoad();
#ifdef ENABLE_TRANSFORM
  Eigen::AngleAxisd current_rotation_x(DEGREE_TO_RADIAN(param_.transform_param.roll), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd current_rotation_y(DEGREE_TO_RADIAN(param_.transform_param.pitch), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd current_rotation_z(DEGREE_TO_RADIAN(param_.transform_param.yaw), Eigen::Vector3d::UnitZ());
  Eigen::Translation3d current_translation(param_.transform_param.x, param_.transform_param.y, param_.transform_param.z);
  trans_ = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
#endif
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::getPacketDuration() {
  return packet_duration_;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::prevPktTs() {
  return prev_pkt_ts_;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::cloudTs() {
  double ret_point_ts = param_.ts_first_point ? first_point_ts_ : last_point_ts_;
  return (ret_point_ts < 0 ? 0 : ret_point_ts);
}

/// @brief Perform coordinate transformation on points. It is based on the
/// third-party open-source library Eigen
template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::transformPoint(float &x, float &y, float &z) {
#ifdef ENABLE_TRANSFORM
  if (param_.transform_param.x != 0 || param_.transform_param.y != 0 || param_.transform_param.z != 0 || param_.transform_param.roll != 0 ||
      param_.transform_param.pitch != 0 || param_.transform_param.yaw != 0) {
    Eigen::Vector4d target_ori(x, y, z, 1);
    Eigen::Vector4d target_rotate = trans_ * target_ori;
    x = target_rotate(0);
    y = target_rotate(1);
    z = target_rotate(2);
  }
#endif
}

template <typename T_PointCloud>
std::vector<std::string> Decoder<T_PointCloud>::split(const std::string &s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);

  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::hidePiontsParamsLoad() {
  if (param_.hide_points_range != "") {
    std::string hide_points_range_str = param_.hide_points_range;
    hide_points_range_str.erase(std::remove(hide_points_range_str.begin(), hide_points_range_str.end(), ' '), hide_points_range_str.end());

    std::vector<std::string> vec_str = split(hide_points_range_str, ';');
    for (size_t i = 0; i < vec_str.size(); i++) {
      std::vector<std::string> group_str = split(vec_str[i], ',');
      if (group_str.size() != 3) {
        WJ_WARNING << "group " << (i + 1) << " : format warning!" << WJ_REND;
        continue;
      }
      std::vector<std::string> channel_group_str = split(group_str[0], '/');
      std::vector<std::string> angle_group_str = split(group_str[1], '/');
      std::vector<std::string> distance_group_str = split(group_str[2], '/');

      std::vector<std::string> param_channel_str;
      std::vector<std::string> param_angle_str;
      std::vector<std::string> param_distance_str;

      HideRangeParams hide_range_param;

      for (size_t j = 0; j < angle_group_str.size(); j++) {
        HideAngleParams hide_angle_params;
        param_angle_str = split(angle_group_str[j], '-');

        if (param_angle_str.size() == 1) {
          hide_angle_params.min_angle = std::stof(param_angle_str[0]);
          hide_angle_params.max_angle = std::stof(param_angle_str[0]);
        } else if (param_angle_str.size() == 2) {
          hide_angle_params.min_angle = std::stof(param_angle_str[0]);
          hide_angle_params.max_angle = std::stof(param_angle_str[1]);
        } else {
          WJ_WARNING << "group " << (i + 1) << " : angle format warning!" << WJ_REND;
          continue;
        }
        hide_range_param.hide_point_angle.push_back(hide_angle_params);
      }

      for (size_t j = 0; j < distance_group_str.size(); j++) {
        HideDistanceParams hide_distance_params;
        param_distance_str = split(distance_group_str[j], '-');

        if (param_distance_str.size() == 1) {
          hide_distance_params.min_distance = std::stof(param_distance_str[0]);
          hide_distance_params.max_distance = std::stof(param_distance_str[0]);
        } else if (param_distance_str.size() == 2) {
          hide_distance_params.min_distance = std::stof(param_distance_str[0]);
          hide_distance_params.max_distance = std::stof(param_distance_str[1]);
        } else {
          WJ_WARNING << "group " << (i + 1) << " : distance format warning!" << WJ_REND;
          continue;
        }
        hide_range_param.hide_point_distance.push_back(hide_distance_params);
      }

      std::vector<uint16_t> channel_id;
      for (size_t j = 0; j < channel_group_str.size(); j++) {
        param_channel_str = split(channel_group_str[j], '-');

        if (param_channel_str.size() == 1) {
          channel_id.push_back(std::stoi(param_channel_str[0]));
        } else if (param_distance_str.size() == 2) {
          for (int k = std::stoi(param_channel_str[0]); k <= std::stoi(param_channel_str[1]); k++) {
            channel_id.push_back(k);
          }
        } else {
          WJ_WARNING << "group " << (i + 1) << " : channel format warning!" << WJ_REND;
          continue;
        }
      }

      for (size_t channel_index = 0; channel_index < channel_id.size(); channel_index++) {
        HideRangeParams hide_range_param_temp;
        hide_range_param_temp = hide_range_param;
        hide_range_param_temp.hide_point_channel_id = channel_id[channel_index];
        hide_range_params_.push_back(hide_range_param_temp);
      }
    }
    if (hide_range_params_.size() > 0) {
      hide_range_flag_ = true;
    }
  }
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::isValueInRange(uint16_t channel, float angle, float distance, const std::vector<HideRangeParams> &ranges) {
  std::vector<int> channel_index;
  for (size_t i = 0; i < ranges.size(); i++) {
    if (channel == ranges[i].hide_point_channel_id) {
      channel_index.push_back(i);
    }
  }

  for (size_t i = 0; i < channel_index.size(); i++) {
    bool angle_flag = false;
    bool distance_flag = false;

    for (size_t j = 0; j < ranges[channel_index[i]].hide_point_angle.size(); j++) {
      if (angle >= ranges[channel_index[i]].hide_point_angle[j].min_angle && angle <= ranges[channel_index[i]].hide_point_angle[j].max_angle) {
        angle_flag = true;
        break;
      }
    }
    for (size_t j = 0; j < ranges[channel_index[i]].hide_point_distance.size(); j++) {
      if (distance >= ranges[channel_index[i]].hide_point_distance[j].min_distance &&
          distance <= ranges[channel_index[i]].hide_point_distance[j].max_distance) {
        distance_flag = true;
        break;
      }
    }
    if (angle_flag && distance_flag)
      return true;
  }
  return false;
}

template <typename T_PointCloud>
inline uint8_t Decoder<T_PointCloud>::checkBCC(const uint8_t *pkt, size_t start_index, size_t end_index) {
  uint8 bcc = 0x00;
  for (size_t i = start_index; i < end_index; i++) {
    bcc ^= pkt[i];
  }
  return bcc;
}

template <typename T_PointCloud>
inline uint16_t Decoder<T_PointCloud>::checkCRC16(const uint8_t *pkt, size_t start_index, size_t end_index) {
  static const uint16_t crctab[256] = {
      0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210,
      0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401,
      0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6,
      0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
      0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
      0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD,
      0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A,
      0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
      0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB,
      0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
      0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9,
      0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
      0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07,
      0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
      0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};
  uint16_t crc16 = 0x00;

  for (size_t i = start_index; i < end_index; i++) {
    crc16 = (uint16)(crctab[(crc16 >> 8) ^ pkt[i]] ^ (crc16 << 8));
  }

  return crc16;
}

/// @brief MSOP Packet Handler
/// @brief This is a purely virtual function, provided by the derived classes of
/// each lidar to provide its own implementation
template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::processMsopPkt(const uint8_t *pkt, size_t size) {
  constexpr static int CLOUD_POINT_MAX = 1000000;
  if (this->point_cloud_ && (this->point_cloud_->points.size() > CLOUD_POINT_MAX)) {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_CLOUDOVERFLOW)), 1);
  }
  if (param_.wait_for_difop && !angles_ready_) {
    DELAY_LIMIT_CALL(cb_excep_(Error(ERRCODE_NODIFOPRECV)), 1);
    return false;
  }
  // if (size != this->const_param_.MSOP_LEN)
  // {
  //     LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGMSOPLEN)), 1);
  //     return false;
  // }
  // /// @brief Packet的标志字节是否匹配。
  return decodeMsopPkt(pkt, size);
}

}  // namespace lidar
}  // namespace vanjee