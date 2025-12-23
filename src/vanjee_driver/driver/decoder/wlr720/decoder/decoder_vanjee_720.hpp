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

#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/decoder/decoder_packet_base/imu/imuParamGet.hpp>
#include <vanjee_driver/driver/decoder/imu_calibration_param.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/cmd_repository_720.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/protocol_imu_temp_get.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/protocol_imuaddpa_get.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/protocol_imulinepa_get.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/protocol_ld_eccentricity_param_get.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/protocol_ldangle_get.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/protocol_sn_get.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee720Channel {
  uint16_t distance;
  uint8_t intensity;
  uint8_t reflectivity;
} Vanjee720Channel;

typedef struct _Vanjee720Block {
  uint16_t azimuth;
  Vanjee720Channel channel[16];
} Vanjee720Block;

typedef struct _Vanjee720Block2 {
  uint16_t azimuth;
  Vanjee720Channel channel[19];
} Vanjee720Block2;

typedef struct _Vanjee720BlockFFEE {
  uint8_t header[2];
  uint16_t azimuth;
  Vanjee720Channel channel[19];
} Vanjee720BlockFFEE;

typedef struct _Vanjee720Difop {
  uint8_t mac_id[2];
  uint16_t circle_id;
  uint8_t datetime[6];
  uint8_t timestamp[4];
  int16_t imu_angle_voc_x;
  int16_t imu_angle_voc_y;
  int16_t imu_angle_voc_z;
  int16_t imu_linear_acce_x;
  int16_t imu_linear_acce_y;
  int16_t imu_linear_acce_z;
  uint8_t info[34];
} Vanjee720Difop;

typedef struct _Vanjee720MsopPkt16 {
  uint8_t head[2];
  uint8_t channel_num;
  uint8_t return_wave_num;
  uint8_t block_num;
  Vanjee720Block blocks[18];
  Vanjee720Difop difop;
} Vanjee720MsopPkt16;

typedef struct _Vanjee720MsopPkt19 {
  Vanjee720Block2 blocks[15];
  Vanjee720Difop difop;
} Vanjee720MsopPkt19;

typedef struct _Vanjee720MsopPkt19Double {
  uint8_t head[2];
  uint8_t channel_num;
  uint8_t return_wave_num;
  uint8_t block_num;
  Vanjee720Block2 blocks[12];
  Vanjee720Difop difop;
} Vanjee720MsopPkt19Double;

typedef struct _Vanjee720MsopPkt19FFEE {
  Vanjee720BlockFFEE blocks[15];
  Vanjee720Difop difop;
} Vanjee720MsopPkt19FFEE;

typedef struct _Vanjee720MsopPktErrorCode {
  uint8_t head[2];
  uint16_t frame_len;
  uint8_t check_type;
  uint16_t device_type;
  uint8_t error_code_version;
  uint32_t sec;
  uint32_t nsec;
  uint8_t device_state;
  uint8_t error_code_num;
  uint16_t error_code;
  uint8_t remain[10];
  uint8_t check[2];
  uint8_t tail[2];
} Vanjee720MsopPktErrorCode;

#pragma pack(pop)
template <typename T_PointCloud>
class DecoderVanjee720 : public DecoderMech<T_PointCloud> {
 private:
  int32_t optcent_2_lidar_arg_ = 17571;
  float optcent_2_lidar_l_ = 4.5545 * 1e-2;

  std::vector<std::vector<double>> all_points_luminous_moment_720c_;  // Cache 16 channels for one circle of point cloud time difference
  std::vector<std::vector<double>> all_points_luminous_moment_720f_;  // Cache 19 channels for one circle of point cloud time difference
  const double luminous_period_of_ld_ = 5.555e-5;                     // Time interval at adjacent horizontal angles
  const double luminous_period_of_adjacent_ld_ = 2.34e-6;             // Time interval between adjacent vertical angles within the group
  const double luminous_period_of_adjacent_ld_720d_ = 3.2e-6;         // Time interval between adjacent vertical angles within the group

  int32_t azimuth_trans_pre_ = -1.0;

  int32_t pre_frame_id_ = -1;
  int32_t pre_circle_id_ = -1;
  uint8_t publish_mode_ = 0;

  int32_t pre_pkt_hor_angle_ = -1;
  double pre_pkt_time_ = -1;
  int32_t resolution_num_offset_ = -1;
  int32_t pre_pkt_resolution = -1;
  bool angle_param_get_flag_ = false;
  uint16_t eccentricity_param_get_flag_ = 0;
  bool imu_acc_param_get_flag_ = false;
  bool imu_ang_param_get_flag_ = false;
  std::vector<int32_t> eccentricity_angles_;
  std::vector<int32_t> eccentricity_angles_real_;
  uint8_t lidar_type_ = 0;

  int16_t pre_error_code_frame_id_ = -1;
  bool timestamp_table_init_flag_ = false;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);
  static WJEchoMode getEchoMode(uint8_t mode);
  void initLdLuminousMoment19(void);
  bool initLdLuminousMoment16(void);
  bool lidarParamGet(uint32_t hor_angle, uint32_t resolution, uint8_t block_num, double time);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 100;

  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee720() = default;
  explicit DecoderVanjee720(const WJDecoderParam &param);

  bool decodeMsopPkt720F_FFEE(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt720C(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt720F_FFDD(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt720F_DoubleEcho(const uint8_t *pkt, size_t size);
  void decodeMsopPktErrorCode(const uint8_t *pkt, size_t size);

  void SendImuData(Vanjee720Difop difop, double temperature, double timestamp, double lidar_timestamp);

  uint16_t chanToline(uint16_t chan);

 public:
  double imu_temperature_;
};

template <typename T_PointCloud>
bool DecoderVanjee720<T_PointCloud>::lidarParamGet(uint32_t hor_angle, uint32_t resolution, uint8_t block_num, double time) {
  if (pre_pkt_hor_angle_ > 0 && (hor_angle + 36000 - pre_pkt_hor_angle_) % 36000 == resolution * block_num) {
    if (pre_pkt_time_ > 0 && fabs(time - pre_pkt_time_) > 0.001) {
      resolution_num_offset_ = (hor_angle / resolution) % 180;
      return true;
    }
  }
  pre_pkt_hor_angle_ = hor_angle;
  pre_pkt_time_ = time;
  return false;
}

template <typename T_PointCloud>
void DecoderVanjee720<T_PointCloud>::initLdLuminousMoment19() {
  double offset = 0;
  all_points_luminous_moment_720f_.resize(4);
  all_points_luminous_moment_720f_[0].resize(68400);
  all_points_luminous_moment_720f_[1].resize(34200);
  all_points_luminous_moment_720f_[2].resize(22800);
  all_points_luminous_moment_720f_[3].resize(17100);

  for (uint16_t col = 0; col < 3600; col++) {
    for (uint8_t row = 0; row < 19; row++) {
      if (row < 5)
        offset = row * luminous_period_of_adjacent_ld_;
      else if (row < 9)
        offset = (row - 5) * luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ / 4;
      else if (row < 14)
        offset = (row - 9) * luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * 2 / 4;
      else
        offset = (row - 14) * luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * 3 / 4;

      if (col < 900) {
        for (int i = 0; i < 4; i++) all_points_luminous_moment_720f_[i][col * 19 + row] = col * luminous_period_of_ld_ + offset;
      } else if (col >= 900 && col < 1200) {
        for (int i = 0; i < 3; i++) all_points_luminous_moment_720f_[i][col * 19 + row] = col * luminous_period_of_ld_ + offset;
      } else if (col >= 1200 && col < 1800) {
        for (int i = 0; i < 2; i++) all_points_luminous_moment_720f_[i][col * 19 + row] = col * luminous_period_of_ld_ + offset;
      } else {
        all_points_luminous_moment_720f_[0][col * 19 + row] = col * luminous_period_of_ld_ + offset;
      }
    }
  }
}

template <typename T_PointCloud>
bool DecoderVanjee720<T_PointCloud>::initLdLuminousMoment16() {
  if (!timestamp_table_init_flag_ && ((this->param_.wait_for_difop && lidar_type_ != 0) || (!this->param_.wait_for_difop))) {
    double offset = 0;
    all_points_luminous_moment_720c_.resize(4);
    all_points_luminous_moment_720c_[0].resize(57600);
    all_points_luminous_moment_720c_[1].resize(28800);
    all_points_luminous_moment_720c_[2].resize(19200);
    all_points_luminous_moment_720c_[3].resize(14400);
    for (uint16_t col = 0; col < 3600; col++) {
      for (uint8_t row = 0; row < 16; row++) {
        if (lidar_type_ == 1) {
          if (row < 4)
            offset = (row + 1) * luminous_period_of_adjacent_ld_;
          else if (row < 7)
            offset = (row + 1 - 4) * luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ / 4;
          else if (row < 12)
            offset = (row - 7) * luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * 2 / 4;
          else
            offset = (row + 1 - 12) * luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * 3 / 4;
        } else {
          offset = row * luminous_period_of_adjacent_ld_720d_ + 1.968e-6;
        }

        if (col < 900) {
          for (int i = 0; i < 4; i++) all_points_luminous_moment_720c_[i][col * 16 + row] = col * luminous_period_of_ld_ + offset;
        } else if (col >= 900 && col < 1200) {
          for (int i = 0; i < 3; i++) all_points_luminous_moment_720c_[i][col * 16 + row] = col * luminous_period_of_ld_ + offset;
        } else if (col >= 1200 && col < 1800) {
          for (int i = 0; i < 2; i++) all_points_luminous_moment_720c_[i][col * 16 + row] = col * luminous_period_of_ld_ + offset;
        } else {
          all_points_luminous_moment_720c_[0][col * 16 + row] = col * luminous_period_of_ld_ + offset;
        }
      }
    }

    timestamp_table_init_flag_ = true;
  }
  return timestamp_table_init_flag_;
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee720<T_PointCloud>::getConstParam(uint8_t mode) {
  // WJ_INFOL << "publish_mode ============mode=================" << mode << WJ_REND;
  uint16_t msop_len = 1253;
  uint16_t laser_num = 16;
  uint16_t block_num = 18;
  uint16_t chan_num = 16;
  float distance_min = 0.3f;
  float distance_max = 120.0f;
  float distance_resolution = 0.004f;
  float init_temperature = 80.0f;

  // if(lidar_type_id == 1 && mode == 1)
  // {
  //     msop_len = 1260;
  //     laser_num = 16;
  //     block_num = 15;
  //     chan_num = 19;
  // }
  // else if(lidar_type_id == 1 && mode == 1)
  // {
  //     msop_len = 1253;
  //     laser_num = 16;
  //     block_num = 18;
  //     chan_num= 16;
  // }
  // else if(lidar_type_id == 2 && mode == 1)
  // {
  //     msop_len = 1235;
  //     laser_num = 16;
  //     block_num = 15;
  //     chan_num = 16;
  // }
  // else if(lidar_type_id == 2 && mode == 2)
  // {
  //     msop_len = 1001;
  //     laser_num = 16;
  //     block_num = 12;
  //     chan_num = 19;
  // }

  static WJDecoderMechConstParam param = {
      msop_len  /// msop len
      ,
      laser_num  /// laser number
      ,
      block_num  /// blocks per packet
      ,
      chan_num  /// channels per block
      ,
      distance_min  /// distance min
      ,
      distance_max  /// distance max
      ,
      distance_resolution  /// distance resolution
      ,
      init_temperature  /// initial value of temperature
  };
  param.BLOCK_DURATION = 0.1 / 360;
  return param;
}

template <typename T_PointCloud>
inline WJEchoMode DecoderVanjee720<T_PointCloud>::getEchoMode(uint8_t mode) {
  switch (mode) {
    case 0x10:  ///
      return WJEchoMode::ECHO_DUAL;
    case 0x20:
    case 0x30:

    default:
      return WJEchoMode::ECHO_SINGLE;
  }
}

template <typename T_PointCloud>
inline DecoderVanjee720<T_PointCloud>::DecoderVanjee720(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  publish_mode_ = param.publish_mode;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);

  this->start_angle_ = this->param_.start_angle * 1000;
  this->end_angle_ = this->param_.end_angle * 1000;

  eccentricity_angles_.resize(14400);
  eccentricity_angles_real_.resize(14400);

  this->m_imu_params_get_ = std::make_shared<ImuParamGet>(90, param.transform_param);
  imu_temperature_ = -100.0;
  if (param.imu_enable == -1) {
    this->point_cloud_ready_ = true;
  } else if (param.imu_enable == 0) {
    imu_temperature_ = 40.0;
    this->point_cloud_ready_ = true;
  } else {
    WJ_INFO << "Waiting for IMU calibration..." << WJ_REND;
  }

  if (this->param_.config_from_file) {
    this->chan_angles_.loadFromFile(this->param_.angle_path_hor, 14400);
    for (int i = 0; i < 14400; i++) {
      eccentricity_angles_real_[i] = this->chan_angles_.eccentricityAdjust(i, 1);
    }
    this->chan_angles_.loadFromFile(this->param_.angle_path_ver);
    this->imu_calibration_param_.loadFromFile(param.imu_param_path);

    this->m_imu_params_get_->setImuTempCalibrationParams(this->imu_calibration_param_.x_axis_temp_k, this->imu_calibration_param_.x_axis_temp_b,
                                                         this->imu_calibration_param_.y_axis_temp_k, this->imu_calibration_param_.y_axis_temp_b,
                                                         this->imu_calibration_param_.z_axis_temp_k, this->imu_calibration_param_.z_axis_temp_b);

    this->m_imu_params_get_->setImuAcceCalibrationParams(this->imu_calibration_param_.x_axis_acc_k, this->imu_calibration_param_.x_axis_acc_b,
                                                         this->imu_calibration_param_.y_axis_acc_k, this->imu_calibration_param_.y_axis_acc_b,
                                                         this->imu_calibration_param_.z_axis_acc_k, this->imu_calibration_param_.z_axis_acc_b);
  }
  initLdLuminousMoment19();
}

template <typename T_PointCloud>
inline bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  switch (size) {
    case 1260: {
      ret = decodeMsopPkt720F_FFEE(pkt, size);
    } break;

    case 1253: {
      if (!initLdLuminousMoment16()) {
        return ret;
      }
      ret = decodeMsopPkt720C(pkt, size);
    } break;

    case 1235: {
      ret = decodeMsopPkt720F_FFDD(pkt, size);
    } break;

    case 1001: {
      ret = decodeMsopPkt720F_DoubleEcho(pkt, size);
    } break;

    case sizeof(Vanjee720MsopPktErrorCode): {
      decodeMsopPktErrorCode(pkt, size);
    } break;
  }
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee720<T_PointCloud>::decodeMsopPktErrorCode(const uint8_t *pkt, size_t size) {
  if (!this->param_.device_ctrl_state_enable && !this->param_.send_lidar_param_enable) {
    return;
  }
  std::vector<uint8_t> buf(pkt, pkt + size);
  const Vanjee720MsopPktErrorCode &packet = *(Vanjee720MsopPktErrorCode *)pkt;
  if (packet.check_type == 1) {
    if (CheckClass::Xor(buf, 2, size - 6) != (uint16)((packet.check[0] << 8) + packet.check[1])) {
      WJ_WARNING << "Error code pkt check err" << WJ_REND;
      return;
    }
  } else if (packet.check_type == 2) {
    if (CheckClass::Crc16(buf, 2, size - 6) != (uint16)((packet.check[0] << 8) + packet.check[1])) {
      WJ_WARNING << "Error code pkt check err" << WJ_REND;
      return;
    }
  }

  uint16_t frame_id = (uint16_t)(packet.device_state & 0x0f);
  uint16_t loss_packets_num = (frame_id + 16 - pre_error_code_frame_id_) % 16;
  if (loss_packets_num > 1 && pre_error_code_frame_id_ >= 0) {
    WJ_WARNING << "error code frame loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
    pre_error_code_frame_id_ = frame_id;
    return;
  }
  pre_error_code_frame_id_ = frame_id;

  double pkt_ts = 0.0;
  if (!this->param_.use_lidar_clock)
    pkt_ts = getTimeHost() * 1e-6;
  else {
    pkt_ts = packet.sec + (packet.nsec * 1e-9);
    pkt_ts = pkt_ts < 0 ? 0 : pkt_ts;
  }

  uint16_t device_state = (uint16_t)((packet.device_state >> 4) & 0x0f);
  if (device_state == 0) {
    return;
  }
  this->deviceStatePublish(0, packet.error_code, device_state, pkt_ts);
}

template <typename T_PointCloud>
bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt720F_FFEE(const uint8_t *pkt, size_t size) {
  const Vanjee720MsopPkt19FFEE &packet = *(Vanjee720MsopPkt19FFEE *)pkt;

  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  pkt_host_ts = getTimeHost() * 1e-6;
  WJTimestampYMD tm{packet.difop.datetime[5], packet.difop.datetime[4], packet.difop.datetime[3],
                    packet.difop.datetime[2], packet.difop.datetime[1], packet.difop.datetime[0]};
  double nsec = (packet.difop.timestamp[0] + (packet.difop.timestamp[1] << 8) + (packet.difop.timestamp[2] << 16) +
                 ((packet.difop.timestamp[3] & 0x0F) << 24)) *
                1e-8;
  pkt_lidar_ts = parseTimeYMD(&tm) * 1e-6 + nsec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else {
    pkt_ts = pkt_lidar_ts < 0 ? 0 : pkt_lidar_ts;
  }

  uint32_t rpm = packet.difop.info[12] | (packet.difop.info[13] << 8);
  int32_t resolution = 10;
  uint8_t resolution_index = 0;
  double last_point_to_first_point_time = 0;
  if (packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0) {
    resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  } else {
    resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  }

  if (resolution != pre_pkt_resolution) {
    resolution_num_offset_ = -1;
    pre_pkt_hor_angle_ = -1;
    pre_pkt_time_ = -1;
  }
  pre_pkt_resolution = resolution;
  if (resolution_num_offset_ < 0 && !lidarParamGet(packet.blocks[0].azimuth, resolution, 15, pkt_lidar_ts)) {
    return false;
  }

  if (resolution == 10) {
    resolution_index = 0;
    last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720f_[resolution_index][68399];
  } else if (resolution == 20) {
    resolution_index = 1;
    last_point_to_first_point_time = (1.0 / 10.0) - all_points_luminous_moment_720f_[resolution_index][34199];
  } else if (resolution == 30) {
    resolution_index = 2;
    last_point_to_first_point_time = (1.0 / 15.0) - all_points_luminous_moment_720f_[resolution_index][22799];
  } else if (resolution == 40) {
    resolution_index = 3;
    last_point_to_first_point_time = (1.0 / 20.0) - all_points_luminous_moment_720f_[resolution_index][17099];
  } else {
    return ret;
  }

  if (pkt_lidar_ts != this->prev_pkt_ts_) {
    SendImuData(packet.difop, imu_temperature_, pkt_ts, pkt_lidar_ts);
  }

  if (!this->param_.point_cloud_enable)
    return false;

  uint16_t frame_id = (packet.difop.info[30] << 8) | packet.difop.info[31];
  uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = frame_id;

  if (!this->imu_ready_ && !this->point_cloud_ready_) {
    this->prev_pkt_ts_ = pkt_lidar_ts;
    return ret;
  } else if (!this->point_cloud_ready_) {
    this->point_cloud_ready_ = true;
  }

  uint32_t loss_circles_num = (ntohs(packet.difop.circle_id) + 65536 - pre_circle_id_) % 65536;
  if (loss_circles_num > 1) {
    this->point_cloud_->points.clear();
  }

  for (uint16_t blk = 0; blk < 15; blk++) {
    const Vanjee720BlockFFEE &block = packet.blocks[blk];
    int32_t azimuth = block.azimuth % 36000;
    int32_t azimuth_10 = block.azimuth * 10;
    int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

    if ((this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0) ||
        (loss_circles_num == 1 && blk == 0 && (packet.blocks[0].azimuth % 36000) != 0 && this->point_cloud_->points.size() != 0)) {
      int32_t time_reload_num = 0;
      int32_t time_change_num = (azimuth / resolution - resolution_num_offset_);
      if (time_change_num >= 0) {
        time_reload_num = time_change_num / 180;
        if (time_change_num % 180 >= 0) {
          time_reload_num += 1;
        }
      }
      uint32_t point_gap_num = (time_reload_num * 180 + resolution_num_offset_) * 19;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720f_[resolution_index][point_gap_num] - last_point_to_first_point_time;
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size() - 1];

      this->cb_split_frame_(19, this->cloudTs());
      ret = true;
    }
    if ((block.header[0] == 255) && (block.header[1] == 238)) {
      double timestamp_point;
      uint32_t col_index_eccentricity = azimuth / 2.5;
      uint32_t cur_blk_first_point_id = azimuth / resolution * 19;
      for (uint16_t chan = 0; chan < 19; chan++) {
        float x, y, z, xy;

        uint32_t point_id = cur_blk_first_point_id + chan;
        if (this->param_.ts_first_point == true) {
          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id];
        } else {
          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id] -
                            all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size() - 1];
        }

        const Vanjee720Channel &channel = block.channel[chan];
        float distance = channel.distance * this->const_param_.DISTANCE_RES;

        int l_line = chanToline(chan + 1);
        int32_t angle_vert = this->chan_angles_.vertAdjust(l_line);
        int32_t angle_horiz = 0;
        uint32_t offset_angle = this->rpmOffsetAngle(rpm, all_points_luminous_moment_720f_[resolution_index][chan]);
        switch (chan) {
          case 0:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity] + offset_angle + 90000) %
                          360000;
            break;

          case 5:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10 + resolution * 2.5, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity + 1] + offset_angle + 90000) %
                          360000;
            break;

          case 9:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10 + resolution * 5, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity + 2] + offset_angle + 90000) %
                          360000;
            break;

          case 14:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10 + resolution * 7.5, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity + 3] + offset_angle + 90000) %
                          360000;
            break;

          default:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity] + offset_angle + 90000) %
                          360000;
            break;
        }

        if (this->distance_section_.in(distance)) {
          float temp_dis = distance - optcent_2_lidar_l_;
          distance = temp_dis > 0 ? temp_dis : 0;
        }

        if (this->distance_section_.in(distance)) {
          int32_t optcent_2_lidar_angle_hor = (angle_horiz - optcent_2_lidar_arg_ + 360000) % 360000;
          CenterCompensationParams optical_center_compensation_param;
          this->calcOpticalCenterCompensation(optical_center_compensation_param, optcent_2_lidar_angle_hor, optcent_2_lidar_l_, 0);

          xy = distance * COS(angle_vert);
          x = xy * SIN(angle_horiz) + optical_center_compensation_param.x;
          y = xy * COS(angle_horiz) + optical_center_compensation_param.y;
          z = distance * SIN(angle_vert);
          uint32_t angle_horiz_mask = (450000 - angle_horiz) % 360000;
#ifdef ENABLE_GTEST
          distance = std::pow(x * x + y * y + z * z, 0.5);
          angle_horiz_mask = this->coordTransAzimuth(x, y);
#endif
          if (this->start_angle_ < this->end_angle_) {
            if (angle_horiz_mask < this->start_angle_ || angle_horiz_mask > this->end_angle_) {
              distance = 0;
            }
          } else {
            if (angle_horiz_mask > this->end_angle_ && angle_horiz_mask < this->start_angle_) {
              distance = 0;
            }
          }

          if (this->hide_range_params_.size() > 0 && distance != 0 &&
              this->isValueInRange(l_line + this->first_line_id_, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
            distance = 0;
          }
        }

        if (this->distance_section_.in(distance)) {
          this->transformPoint(x, y, z);

          typename T_PointCloud::PointT point;
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, channel.reflectivity);
          setTimestamp(point, timestamp_point);
          setRing(point, l_line + this->first_line_id_);
          setTag(point, 0);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, angle_horiz / 1000.0);
          setVerAngle(point, angle_vert / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        } else {
          typename T_PointCloud::PointT point;
          if (!this->param_.dense_points) {
            setX(point, NAN);
            setY(point, NAN);
            setZ(point, NAN);
          } else {
            setX(point, 0);
            setY(point, 0);
            setZ(point, 0);
          }

          setIntensity(point, 0);
          setTimestamp(point, timestamp_point);
          setRing(point, l_line + this->first_line_id_);
          setTag(point, 0);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, angle_horiz / 1000.0);
          setVerAngle(point, angle_vert / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        }
      }
    }
    if (azimuth_trans == 0) {
      uint32_t point_gap_num = resolution_num_offset_ * 19;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720f_[resolution_index][point_gap_num];
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size() - 1];
      this->cb_split_frame_(19, this->cloudTs());
      ret = true;
    }
  }
  pre_circle_id_ = ntohs(packet.difop.circle_id);
  this->prev_pkt_ts_ = pkt_lidar_ts;
  return ret;
}

template <typename T_PointCloud>
bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt720C(const uint8_t *pkt, size_t size) {
  const Vanjee720MsopPkt16 &packet = *(Vanjee720MsopPkt16 *)pkt;

  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  pkt_host_ts = getTimeHost() * 1e-6;
  WJTimestampYMD tm{packet.difop.datetime[5], packet.difop.datetime[4], packet.difop.datetime[3],
                    packet.difop.datetime[2], packet.difop.datetime[1], packet.difop.datetime[0]};
  double nsec = (packet.difop.timestamp[0] + (packet.difop.timestamp[1] << 8) + (packet.difop.timestamp[2] << 16) +
                 ((packet.difop.timestamp[3] & 0x0F) << 24)) *
                1e-8;
  pkt_lidar_ts = parseTimeYMD(&tm) * 1e-6 + nsec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else {
    pkt_ts = pkt_lidar_ts < 0 ? 0 : pkt_lidar_ts;
  }

  uint32_t rpm = packet.difop.info[12] | (packet.difop.info[13] << 8);
  int32_t resolution = 10;
  uint8_t resolution_index = 0;
  double last_point_to_first_point_time = 0;
  if (packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0) {
    resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  } else {
    resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  }

  if (resolution != pre_pkt_resolution) {
    resolution_num_offset_ = -1;
    pre_pkt_hor_angle_ = -1;
    pre_pkt_time_ = -1;
  }
  pre_pkt_resolution = resolution;
  if (resolution_num_offset_ < 0 && !lidarParamGet(packet.blocks[0].azimuth, resolution, packet.block_num, pkt_lidar_ts)) {
    return false;
  }

  if (resolution == 10) {
    resolution_index = 0;
    last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720c_[resolution_index][57599];
  } else if (resolution == 20) {
    resolution_index = 1;
    last_point_to_first_point_time = (1.0 / 10.0) - all_points_luminous_moment_720c_[resolution_index][28799];
  } else if (resolution == 30) {
    resolution_index = 2;
    last_point_to_first_point_time = (1.0 / 15.0) - all_points_luminous_moment_720c_[resolution_index][19199];
  } else if (resolution == 40) {
    resolution_index = 3;
    last_point_to_first_point_time = (1.0 / 20.0) - all_points_luminous_moment_720c_[resolution_index][14399];
  } else {
    return ret;
  }

  if (pkt_lidar_ts != this->prev_pkt_ts_) {
    SendImuData(packet.difop, imu_temperature_, pkt_ts, pkt_lidar_ts);
  }

  if (!this->param_.point_cloud_enable)
    return false;

  uint16_t frame_id = (packet.difop.info[30] << 8) | packet.difop.info[31];
  uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = frame_id;

  if (!this->imu_ready_ && !this->point_cloud_ready_) {
    this->prev_pkt_ts_ = pkt_lidar_ts;
    return ret;
  } else if (!this->point_cloud_ready_) {
    this->point_cloud_ready_ = true;
  }

  uint32_t loss_circles_num = (ntohs(packet.difop.circle_id) + 65536 - pre_circle_id_) % 65536;
  if (loss_circles_num > 1) {
    this->point_cloud_->points.clear();
  }

  uint16_t block_num = packet.return_wave_num * packet.block_num;
  for (uint16_t blk = 0; blk < block_num; blk++) {
    if (packet.return_wave_num == 2 && publish_mode_ == 0 && (blk % 2 == 1)) {
      continue;
    } else if (packet.return_wave_num == 2 && publish_mode_ == 1 && (blk % 2 == 0)) {
      continue;
    }
    const Vanjee720Block &block = packet.blocks[blk];
    int32_t azimuth = block.azimuth % 36000;
    int32_t azimuth_10 = block.azimuth * 10;
    int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

    if ((this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0) ||
        (loss_circles_num == 1 && blk == 0 && (packet.blocks[0].azimuth % 36000) != 0 &&
         packet.blocks[0].azimuth < packet.blocks[block_num - 1].azimuth && this->point_cloud_->points.size() != 0)) {
      int32_t time_reload_num = 0;
      int32_t time_change_num = (azimuth / resolution - resolution_num_offset_);
      if (time_change_num >= 0) {
        time_reload_num = time_change_num / 180;
        if (time_change_num % 180 >= 0) {
          time_reload_num += 1;
        }
      }
      uint32_t point_gap_num = (time_reload_num * 180 + resolution_num_offset_) * 16;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720c_[resolution_index][point_gap_num] - last_point_to_first_point_time;
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_720c_[resolution_index][all_points_luminous_moment_720c_[resolution_index].size() - 1];

      this->cb_split_frame_(16, this->cloudTs());
      ret = true;
    }
    {
      double timestamp_point;
      uint32_t col_index_eccentricity = azimuth / 2.5;
      uint32_t cur_blk_first_point_id = azimuth / resolution * pkt[2];
      for (uint16_t chan = 0; chan < pkt[2]; chan++) {
        float x, y, z, xy;

        uint32_t point_id = cur_blk_first_point_id + chan;
        if (this->param_.ts_first_point == true) {
          timestamp_point = all_points_luminous_moment_720c_[resolution_index][point_id];
        } else {
          timestamp_point = all_points_luminous_moment_720c_[resolution_index][point_id] -
                            all_points_luminous_moment_720c_[resolution_index][all_points_luminous_moment_720c_[resolution_index].size() - 1];
        }

        const Vanjee720Channel &channel = block.channel[chan];
        float distance = channel.distance * this->const_param_.DISTANCE_RES;
        int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
        uint32_t offset_angle = this->rpmOffsetAngle(rpm, all_points_luminous_moment_720c_[resolution_index][chan]);
        int32_t angle_horiz = (this->chan_angles_.horizAdjust(chan, azimuth_10, RotateDirection::clockwise) +
                               eccentricity_angles_real_[col_index_eccentricity] + offset_angle + 90000) %
                              360000;

        if (this->distance_section_.in(distance)) {
          float temp_dis = distance - optcent_2_lidar_l_;
          distance = temp_dis > 0 ? temp_dis : 0;
        }

        if (this->distance_section_.in(distance)) {
          int32_t optcent_2_lidar_angle_hor = (angle_horiz - optcent_2_lidar_arg_ + 360000) % 360000;
          CenterCompensationParams optical_center_compensation_param;
          this->calcOpticalCenterCompensation(optical_center_compensation_param, optcent_2_lidar_angle_hor, optcent_2_lidar_l_, 0);

          xy = distance * COS(angle_vert);
          x = xy * SIN(angle_horiz) + optical_center_compensation_param.x;
          y = xy * COS(angle_horiz) + optical_center_compensation_param.y;
          z = distance * SIN(angle_vert);
          uint32_t angle_horiz_mask = (450000 - angle_horiz) % 360000;
#ifdef ENABLE_GTEST
          distance = std::pow(x * x + y * y + z * z, 0.5);
          angle_horiz_mask = this->coordTransAzimuth(x, y);
#endif
          if (this->start_angle_ < this->end_angle_) {
            if (angle_horiz_mask < this->start_angle_ || angle_horiz_mask > this->end_angle_) {
              distance = 0;
            }
          } else {
            if (angle_horiz_mask > this->end_angle_ && angle_horiz_mask < this->start_angle_) {
              distance = 0;
            }
          }

          if (this->hide_range_params_.size() > 0 && distance != 0 &&
              this->isValueInRange(chan + this->first_line_id_, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
            distance = 0;
          }
        }

        if (this->distance_section_.in(distance)) {
          this->transformPoint(x, y, z);

          typename T_PointCloud::PointT point;
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, channel.reflectivity);
          setTimestamp(point, timestamp_point);
          setRing(point, chan + this->first_line_id_);
          setTag(point, 0);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, angle_horiz / 1000.0);
          setVerAngle(point, angle_vert / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);

        } else {
          typename T_PointCloud::PointT point;
          if (!this->param_.dense_points) {
            setX(point, NAN);
            setY(point, NAN);
            setZ(point, NAN);
          } else {
            setX(point, 0);
            setY(point, 0);
            setZ(point, 0);
          }
          setIntensity(point, 0);
          setTimestamp(point, timestamp_point);
          setRing(point, chan + this->first_line_id_);
          setTag(point, 0);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, angle_horiz / 1000.0);
          setVerAngle(point, angle_vert / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        }
      }
    }
    if (azimuth_trans == 0 && (packet.return_wave_num == 1 || (packet.return_wave_num == 2 && publish_mode_ != 2) ||
                               (packet.return_wave_num == 2 && publish_mode_ == 2 && azimuth_trans_pre_ == 0))) {
      uint32_t point_gap_num = resolution_num_offset_ * 16;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720c_[resolution_index][point_gap_num];
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_720c_[resolution_index][all_points_luminous_moment_720c_[resolution_index].size() - 1];
      this->cb_split_frame_(16, this->cloudTs());
      ret = true;
    }
    azimuth_trans_pre_ = azimuth_trans;
  }
  pre_circle_id_ = ntohs(packet.difop.circle_id);
  this->prev_pkt_ts_ = pkt_lidar_ts;
  return ret;
}

template <typename T_PointCloud>
bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt720F_FFDD(const uint8_t *pkt, size_t size) {
  auto &packet = *(Vanjee720MsopPkt19 *)&pkt[5];

  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  pkt_host_ts = getTimeHost() * 1e-6;
  WJTimestampYMD tm{packet.difop.datetime[5], packet.difop.datetime[4], packet.difop.datetime[3],
                    packet.difop.datetime[2], packet.difop.datetime[1], packet.difop.datetime[0]};
  double nsec = (packet.difop.timestamp[0] + (packet.difop.timestamp[1] << 8) + (packet.difop.timestamp[2] << 16) +
                 ((packet.difop.timestamp[3] & 0x0F) << 24)) *
                1e-8;
  pkt_lidar_ts = parseTimeYMD(&tm) * 1e-6 + nsec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else {
    pkt_ts = pkt_lidar_ts < 0 ? 0 : pkt_lidar_ts;
  }

  uint32_t rpm = packet.difop.info[12] | (packet.difop.info[13] << 8);
  int32_t resolution = 10;
  uint8_t resolution_index = 0;
  double last_point_to_first_point_time = 0;
  if (packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0) {
    resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  } else {
    resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  }

  if (resolution != pre_pkt_resolution) {
    resolution_num_offset_ = -1;
    pre_pkt_hor_angle_ = -1;
    pre_pkt_time_ = -1;
  }
  pre_pkt_resolution = resolution;
  if (resolution_num_offset_ < 0 && !lidarParamGet(packet.blocks[0].azimuth, resolution, pkt[4], pkt_lidar_ts)) {
    return false;
  }

  if (resolution == 10) {
    resolution_index = 0;
    last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720f_[resolution_index][68399];
  } else if (resolution == 20) {
    resolution_index = 1;
    last_point_to_first_point_time = (1.0 / 10.0) - all_points_luminous_moment_720f_[resolution_index][34199];
  } else if (resolution == 30) {
    resolution_index = 2;
    last_point_to_first_point_time = (1.0 / 15.0) - all_points_luminous_moment_720f_[resolution_index][22799];
  } else if (resolution == 40) {
    resolution_index = 3;
    last_point_to_first_point_time = (1.0 / 20.0) - all_points_luminous_moment_720f_[resolution_index][17099];
  } else {
    return ret;
  }

  if (pkt_lidar_ts != this->prev_pkt_ts_) {
    SendImuData(packet.difop, imu_temperature_, pkt_ts, pkt_lidar_ts);
  }

  if (!this->param_.point_cloud_enable)
    return false;

  uint16_t frame_id = (packet.difop.info[30] << 8) | packet.difop.info[31];
  uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = frame_id;

  if (!this->imu_ready_ && !this->point_cloud_ready_) {
    this->prev_pkt_ts_ = pkt_lidar_ts;
    return ret;
  } else if (!this->point_cloud_ready_) {
    this->point_cloud_ready_ = true;
  }

  uint32_t loss_circles_num = (ntohs(packet.difop.circle_id) + 65536 - pre_circle_id_) % 65536;
  if (loss_circles_num > 1) {
    this->point_cloud_->points.clear();
  }

  uint16_t block_num = pkt[3] * pkt[4];
  for (uint16_t blk = 0; blk < block_num; blk++) {
    const Vanjee720Block2 &block = packet.blocks[blk];
    int32_t azimuth = block.azimuth % 36000;
    int32_t azimuth_10 = block.azimuth * 10;
    int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

    if ((this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0) ||
        (loss_circles_num == 1 && blk == 0 && (packet.blocks[0].azimuth % 36000) != 0 && this->point_cloud_->points.size() != 0)) {
      int32_t time_reload_num = 0;
      int32_t time_change_num = (azimuth / resolution - resolution_num_offset_);
      if (time_change_num >= 0) {
        time_reload_num = time_change_num / 180;
        if (time_change_num % 180 >= 0) {
          time_reload_num += 1;
        }
      }
      uint32_t point_gap_num = (time_reload_num * 180 + resolution_num_offset_) * 19;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720f_[resolution_index][point_gap_num] - last_point_to_first_point_time;
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size() - 1];

      this->cb_split_frame_(19, this->cloudTs());
      ret = true;
    }

    {
      double timestamp_point;
      uint32_t col_index_eccentricity = azimuth / 2.5;
      uint32_t cur_blk_first_point_id = azimuth / resolution * pkt[2];
      for (uint16_t chan = 0; chan < pkt[2]; chan++) {
        float x, y, z, xy;

        uint32_t point_id = cur_blk_first_point_id + chan;
        if (this->param_.ts_first_point == true) {
          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id];
        } else {
          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id] -
                            all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size() - 1];
        }

        const Vanjee720Channel &channel = block.channel[chan];
        float distance = channel.distance * this->const_param_.DISTANCE_RES;

        int l_line = chanToline(chan + 1);
        int32_t angle_vert = this->chan_angles_.vertAdjust(l_line);
        int32_t angle_horiz = 0;
        uint32_t offset_angle = this->rpmOffsetAngle(rpm, all_points_luminous_moment_720f_[resolution_index][chan]);
        switch (chan) {
          case 0:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity] + offset_angle + 90000) %
                          360000;
            break;

          case 5:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10 + resolution * 2.5, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity + 1] + offset_angle + 90000) %
                          360000;
            break;

          case 9:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10 + resolution * 5, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity + 2] + offset_angle + 90000) %
                          360000;
            break;

          case 14:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10 + resolution * 7.5, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity + 3] + offset_angle + 90000) %
                          360000;
            break;

          default:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity] + offset_angle + 90000) %
                          360000;
            break;
        }

        if (this->distance_section_.in(distance)) {
          float temp_dis = distance - optcent_2_lidar_l_;
          distance = temp_dis > 0 ? temp_dis : 0;
        }

        if (this->distance_section_.in(distance)) {
          int32_t optcent_2_lidar_angle_hor = (angle_horiz - optcent_2_lidar_arg_ + 360000) % 360000;
          CenterCompensationParams optical_center_compensation_param;
          this->calcOpticalCenterCompensation(optical_center_compensation_param, optcent_2_lidar_angle_hor, optcent_2_lidar_l_, 0);

          xy = distance * COS(angle_vert);
          x = xy * SIN(angle_horiz) + optical_center_compensation_param.x;
          y = xy * COS(angle_horiz) + optical_center_compensation_param.y;
          z = distance * SIN(angle_vert);
          uint32_t angle_horiz_mask = (450000 - angle_horiz) % 360000;
#ifdef ENABLE_GTEST
          distance = std::pow(x * x + y * y + z * z, 0.5);
          angle_horiz_mask = this->coordTransAzimuth(x, y);
#endif
          if (this->start_angle_ < this->end_angle_) {
            if (angle_horiz_mask < this->start_angle_ || angle_horiz_mask > this->end_angle_) {
              distance = 0;
            }
          } else {
            if (angle_horiz_mask > this->end_angle_ && angle_horiz_mask < this->start_angle_) {
              distance = 0;
            }
          }

          if (this->hide_range_params_.size() > 0 && distance != 0 &&
              this->isValueInRange(l_line + this->first_line_id_, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
            distance = 0;
          }
        }

        if (this->distance_section_.in(distance)) {
          this->transformPoint(x, y, z);

          typename T_PointCloud::PointT point;
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, channel.reflectivity);
          setTimestamp(point, timestamp_point);
          setRing(point, l_line + this->first_line_id_);
          setTag(point, 0);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, angle_horiz / 1000.0);
          setVerAngle(point, angle_vert / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        } else {
          typename T_PointCloud::PointT point;
          if (!this->param_.dense_points) {
            setX(point, NAN);
            setY(point, NAN);
            setZ(point, NAN);
          } else {
            setX(point, 0);
            setY(point, 0);
            setZ(point, 0);
          }
          setIntensity(point, 0);
          setTimestamp(point, timestamp_point);
          setRing(point, l_line + this->first_line_id_);
          setTag(point, 0);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, angle_horiz / 1000.0);
          setVerAngle(point, angle_vert / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        }
      }
    }
    if (azimuth_trans == 0) {
      uint32_t point_gap_num = resolution_num_offset_ * 19;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720f_[resolution_index][point_gap_num];
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size() - 1];
      this->cb_split_frame_(19, this->cloudTs());
      ret = true;
    }
  }
  pre_circle_id_ = ntohs(packet.difop.circle_id);
  this->prev_pkt_ts_ = pkt_lidar_ts;
  return ret;
}

template <typename T_PointCloud>
bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt720F_DoubleEcho(const uint8_t *pkt, size_t size) {
  auto &packet = *(Vanjee720MsopPkt19Double *)pkt;

  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  pkt_host_ts = getTimeHost() * 1e-6;
  WJTimestampYMD tm{packet.difop.datetime[5], packet.difop.datetime[4], packet.difop.datetime[3],
                    packet.difop.datetime[2], packet.difop.datetime[1], packet.difop.datetime[0]};
  double nsec = (packet.difop.timestamp[0] + (packet.difop.timestamp[1] << 8) + (packet.difop.timestamp[2] << 16) +
                 ((packet.difop.timestamp[3] & 0x0F) << 24)) *
                1e-8;
  pkt_lidar_ts = parseTimeYMD(&tm) * 1e-6 + nsec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else {
    pkt_ts = pkt_lidar_ts < 0 ? 0 : pkt_lidar_ts;
  }

  uint32_t rpm = packet.difop.info[12] | (packet.difop.info[13] << 8);
  int32_t resolution = 10;
  uint8_t resolution_index = 0;
  double last_point_to_first_point_time = 0;
  if (packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0) {
    resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  } else {
    resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  }

  if (resolution != pre_pkt_resolution) {
    resolution_num_offset_ = -1;
    pre_pkt_hor_angle_ = -1;
    pre_pkt_time_ = -1;
  }
  pre_pkt_resolution = resolution;
  if (resolution_num_offset_ < 0 && !lidarParamGet(packet.blocks[0].azimuth, resolution, packet.block_num, pkt_lidar_ts)) {
    return false;
  }

  if (resolution == 10) {
    resolution_index = 0;
    last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720f_[resolution_index][68399];
  } else if (resolution == 20) {
    resolution_index = 1;
    last_point_to_first_point_time = (1.0 / 10.0) - all_points_luminous_moment_720f_[resolution_index][34199];
  } else if (resolution == 30) {
    resolution_index = 2;
    last_point_to_first_point_time = (1.0 / 15.0) - all_points_luminous_moment_720f_[resolution_index][22799];
  } else if (resolution == 40) {
    resolution_index = 3;
    last_point_to_first_point_time = (1.0 / 20.0) - all_points_luminous_moment_720f_[resolution_index][17099];
  } else {
    return ret;
  }

  if (pkt_lidar_ts != this->prev_pkt_ts_) {
    SendImuData(packet.difop, imu_temperature_, pkt_ts, pkt_lidar_ts);
  }

  if (!this->param_.point_cloud_enable)
    return false;

  uint16_t frame_id = (packet.difop.info[30] << 8) | packet.difop.info[31];
  uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = frame_id;

  if (!this->imu_ready_ && !this->point_cloud_ready_) {
    this->prev_pkt_ts_ = pkt_lidar_ts;
    return ret;
  } else if (!this->point_cloud_ready_) {
    this->point_cloud_ready_ = true;
  }

  uint32_t loss_circles_num = (ntohs(packet.difop.circle_id) + 65536 - pre_circle_id_) % 65536;
  if (loss_circles_num > 1) {
    this->point_cloud_->points.clear();
  }

  uint16_t block_num = packet.return_wave_num * packet.block_num;
  for (uint16_t blk = 0; blk < block_num; blk++) {
    if (packet.return_wave_num == 2 && publish_mode_ == 0 && (blk % 2 == 1)) {
      continue;
    } else if (packet.return_wave_num == 2 && publish_mode_ == 1 && (blk % 2 == 0)) {
      continue;
    }
    const Vanjee720Block2 &block = packet.blocks[blk];
    int32_t azimuth = block.azimuth % 36000;
    int32_t azimuth_10 = block.azimuth * 10;
    int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

    if ((this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0) ||
        (loss_circles_num == 1 && blk == 0 && (packet.blocks[0].azimuth % 36000) != 0 &&
         packet.blocks[0].azimuth < packet.blocks[block_num - 1].azimuth && this->point_cloud_->points.size() != 0)) {
      int32_t time_reload_num = 0;
      int32_t time_change_num = (azimuth / resolution - resolution_num_offset_);
      if (time_change_num >= 0) {
        time_reload_num = time_change_num / 180;
        if (time_change_num % 180 >= 0) {
          time_reload_num += 1;
        }
      }
      uint32_t point_gap_num = (time_reload_num * 180 + resolution_num_offset_) * 19;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720f_[resolution_index][point_gap_num] - last_point_to_first_point_time;
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size() - 1];

      this->cb_split_frame_(19, this->cloudTs());
      ret = true;
    }

    {
      double timestamp_point;
      uint32_t col_index_eccentricity = azimuth / 2.5;
      uint32_t cur_blk_first_point_id = azimuth / resolution * pkt[2];
      for (uint16_t chan = 0; chan < pkt[2]; chan++) {
        float x, y, z, xy;

        uint32_t point_id = cur_blk_first_point_id + chan;
        if (this->param_.ts_first_point == true) {
          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id];
        } else {
          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id] -
                            all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size() - 1];
        }

        const Vanjee720Channel &channel = block.channel[chan];
        float distance = channel.distance * this->const_param_.DISTANCE_RES;

        int l_line = chanToline(chan + 1);
        int32_t angle_vert = this->chan_angles_.vertAdjust(l_line);
        int32_t angle_horiz = 0;
        uint32_t offset_angle = this->rpmOffsetAngle(rpm, all_points_luminous_moment_720f_[resolution_index][chan]);
        switch (chan) {
          case 0:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity] + offset_angle + 90000) %
                          360000;
            break;

          case 5:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10 + resolution * 2.5, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity + 1] + offset_angle + 90000) %
                          360000;
            break;

          case 9:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10 + resolution * 5, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity + 2] + offset_angle + 90000) %
                          360000;
            break;

          case 14:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10 + resolution * 7.5, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity + 3] + offset_angle + 90000) %
                          360000;
            break;

          default:
            angle_horiz = (this->chan_angles_.horizAdjust(l_line, azimuth_10, RotateDirection::clockwise) +
                           eccentricity_angles_real_[col_index_eccentricity] + offset_angle + 90000) %
                          360000;
            break;
        }

        if (this->distance_section_.in(distance)) {
          float temp_dis = distance - optcent_2_lidar_l_;
          distance = temp_dis > 0 ? temp_dis : 0;
        }

        if (this->distance_section_.in(distance)) {
          int32_t optcent_2_lidar_angle_hor = (angle_horiz - optcent_2_lidar_arg_ + 360000) % 360000;
          CenterCompensationParams optical_center_compensation_param;
          this->calcOpticalCenterCompensation(optical_center_compensation_param, optcent_2_lidar_angle_hor, optcent_2_lidar_l_, 0);

          xy = distance * COS(angle_vert);
          x = xy * SIN(angle_horiz) + optical_center_compensation_param.x;
          y = xy * COS(angle_horiz) + optical_center_compensation_param.y;
          z = distance * SIN(angle_vert);
          uint32_t angle_horiz_mask = (450000 - angle_horiz) % 360000;
#ifdef ENABLE_GTEST
          distance = std::pow(x * x + y * y + z * z, 0.5);
          angle_horiz_mask = this->coordTransAzimuth(x, y);
#endif
          if (this->start_angle_ < this->end_angle_) {
            if (angle_horiz_mask < this->start_angle_ || angle_horiz_mask > this->end_angle_) {
              distance = 0;
            }
          } else {
            if (angle_horiz_mask > this->end_angle_ && angle_horiz_mask < this->start_angle_) {
              distance = 0;
            }
          }

          if (this->hide_range_params_.size() > 0 && distance != 0 &&
              this->isValueInRange(l_line + this->first_line_id_, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
            distance = 0;
          }
        }

        if (this->distance_section_.in(distance)) {
          this->transformPoint(x, y, z);

          typename T_PointCloud::PointT point;
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, channel.reflectivity);
          setTimestamp(point, timestamp_point);
          setRing(point, l_line + this->first_line_id_);
          setTag(point, 0);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, angle_horiz / 1000.0);
          setVerAngle(point, angle_vert / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        } else {
          typename T_PointCloud::PointT point;
          if (!this->param_.dense_points) {
            setX(point, NAN);
            setY(point, NAN);
            setZ(point, NAN);
          } else {
            setX(point, 0);
            setY(point, 0);
            setZ(point, 0);
          }

          setIntensity(point, 0);
          setTimestamp(point, timestamp_point);
          setRing(point, l_line + this->first_line_id_);
          setTag(point, 0);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, angle_horiz / 1000.0);
          setVerAngle(point, angle_vert / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        }
      }
    }
    if (azimuth_trans == 0 && (packet.return_wave_num == 1 || (packet.return_wave_num == 2 && publish_mode_ != 2) ||
                               (packet.return_wave_num == 2 && publish_mode_ == 2 && azimuth_trans_pre_ == 0))) {
      uint32_t point_gap_num = resolution_num_offset_ * 19;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720f_[resolution_index][point_gap_num];
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size() - 1];
      this->cb_split_frame_(19, this->cloudTs());
      ret = true;
    }
    azimuth_trans_pre_ = azimuth_trans;
  }
  pre_circle_id_ = ntohs(packet.difop.circle_id);
  this->prev_pkt_ts_ = pkt_lidar_ts;
  return ret;
}

template <typename T_PointCloud>
inline uint16_t DecoderVanjee720<T_PointCloud>::chanToline(uint16_t chan) {
  uint16_t line = 1;
  switch (chan) {
    case 1:
      line = 8;
      break;

    case 2:
      line = 1;
      break;

    case 3:
      line = 2;
      break;

    case 4:
      line = 3;
      break;

    case 5:
      line = 4;
      break;

    case 6:
      line = 8;
      break;

    case 7:
      line = 5;
      break;

    case 8:
      line = 6;
      break;

    case 9:
      line = 7;
      break;

    case 10:
      line = 8;
      break;

    case 11:
      line = 9;
      break;

    case 12:
      line = 10;
      break;

    case 13:
      line = 11;
      break;

    case 14:
      line = 12;
      break;

    case 15:
      line = 8;
      break;

    case 16:
      line = 13;
      break;

    case 17:
      line = 14;
      break;

    case 18:
      line = 15;
      break;

    case 19:
      line = 16;
      break;
  }

  return line - 1;
}

template <typename T_PointCloud>
void DecoderVanjee720<T_PointCloud>::SendImuData(Vanjee720Difop difop, double temperature, double timestamp, double lidar_timestamp) {
  if (this->param_.imu_enable == -1)
    return;

  this->imu_ready_ = this->m_imu_params_get_->imuGet(difop.imu_angle_voc_x, difop.imu_angle_voc_y, difop.imu_angle_voc_z, difop.imu_linear_acce_x,
                                                     difop.imu_linear_acce_y, difop.imu_linear_acce_z, lidar_timestamp,
                                                     this->param_.imu_orientation_enable, temperature);
  if (this->imu_ready_) {
    this->imu_packet_->timestamp = timestamp;
    this->imu_packet_->angular_voc[0] = this->m_imu_params_get_->imu_result_stu_.x_angle;
    this->imu_packet_->angular_voc[1] = this->m_imu_params_get_->imu_result_stu_.y_angle;
    this->imu_packet_->angular_voc[2] = this->m_imu_params_get_->imu_result_stu_.z_angle;

    this->imu_packet_->linear_acce[0] = this->m_imu_params_get_->imu_result_stu_.x_acc;
    this->imu_packet_->linear_acce[1] = this->m_imu_params_get_->imu_result_stu_.y_acc;
    this->imu_packet_->linear_acce[2] = this->m_imu_params_get_->imu_result_stu_.z_acc;

    this->imu_packet_->orientation[0] = this->m_imu_params_get_->imu_result_stu_.q0;
    this->imu_packet_->orientation[1] = this->m_imu_params_get_->imu_result_stu_.q1;
    this->imu_packet_->orientation[2] = this->m_imu_params_get_->imu_result_stu_.q2;
    this->imu_packet_->orientation[3] = this->m_imu_params_get_->imu_result_stu_.q3;

    this->cb_imu_pkt_();
  }
}

template <typename T_PointCloud>
void DecoderVanjee720<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract720> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository720::CreateInstance()->sp_ld_angle_get_)) {
    p = std::make_shared<Protocol_LDAngleGet720>();
  } else if (*sp_cmd == *(CmdRepository720::CreateInstance()->sp_imu_line_param_get_)) {
    p = std::make_shared<Protocol_ImuLineGet720>();
  } else if (*sp_cmd == *(CmdRepository720::CreateInstance()->sp_imu_add_param_get_)) {
    p = std::make_shared<Protocol_ImuAddGet720>();
  } else if (*sp_cmd == *(CmdRepository720::CreateInstance()->sp_temperature_param_get_)) {
    p = std::make_shared<Protocol_ImuTempGet>();
  } else if (*sp_cmd == *(CmdRepository720::CreateInstance()->sp_ld_eccentricity_param_get_)) {
    p = std::make_shared<Protocol_LDEccentricityParamGet720>();
  } else if (*sp_cmd == *(CmdRepository720::CreateInstance()->sp_sn_param_get_)) {
    p = std::make_shared<Protocol_SnGet720>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_LDAngle720)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }
    if (!angle_param_get_flag_) {
      std::shared_ptr<Params_LDAngle720> param = std::dynamic_pointer_cast<Params_LDAngle720>(params);

      std::vector<double> vert_angles;
      std::vector<double> horiz_angles;
      if (param->lines_num_ != 16) {
        return;
      }
      for (int line_id = 0; line_id < param->lines_num_; line_id++) {
        vert_angles.push_back((double)(param->ver_angle_[line_id] / 1000.0));
        horiz_angles.push_back((double)(0));
      }

      this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver, param->lines_num_, vert_angles, horiz_angles);

      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        WJ_INFOL << "Get LiDAR<LD_VER> angle data..." << WJ_REND;
        if (this->param_.send_packet_enable) {
          (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setSendInterval(5000);
        } else {
          (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
        }
        angle_param_get_flag_ = true;
      }

      if (angle_param_get_flag_ && eccentricity_param_get_flag_ == 0x7fff && imu_acc_param_get_flag_ && imu_ang_param_get_flag_) {
        Decoder<T_PointCloud>::angles_ready_ = true;
      }
    }
  } else if (typeid(*params) == typeid(Params_IMULine720)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    if (!imu_acc_param_get_flag_) {
      std::shared_ptr<Params_IMULine720> param = std::dynamic_pointer_cast<Params_IMULine720>(params);

      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        WJ_INFOL << "Get LiDAR<IMU> angular_vel data..." << WJ_REND;
        if (this->param_.send_packet_enable) {
          (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setSendInterval(5000);
        } else {
          (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
        }
      }

      this->imu_calibration_param_.loadFromLiDAR(this->param_.imu_param_path, this->imu_calibration_param_.TEMP_PARAM, param->x_k_, param->x_b_,
                                                 param->y_k_, param->y_b_, param->z_k_, param->z_b_);

      WJ_INFO << param->x_k_ << "," << param->x_b_ << "," << param->y_k_ << "," << param->y_b_ << "," << param->z_k_ << "," << param->z_b_ << WJ_REND;

      this->m_imu_params_get_->setImuTempCalibrationParams(param->x_k_, param->x_b_, param->y_k_, param->y_b_, param->z_k_, param->z_b_);
      imu_acc_param_get_flag_ = true;
    }

    if (angle_param_get_flag_ && eccentricity_param_get_flag_ == 0x7fff && imu_acc_param_get_flag_ && imu_ang_param_get_flag_) {
      Decoder<T_PointCloud>::angles_ready_ = true;
    }

  } else if (typeid(*params) == typeid(Params_IMUAdd720)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    if (!imu_ang_param_get_flag_) {
      std::shared_ptr<Params_IMUAdd720> param = std::dynamic_pointer_cast<Params_IMUAdd720>(params);

      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        WJ_INFOL << "Get LiDAR<IMU> linear_acc data..." << WJ_REND;
        if (this->param_.send_packet_enable) {
          (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setSendInterval(5000);
        } else {
          (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
        }
      }

      this->imu_calibration_param_.loadFromLiDAR(this->param_.imu_param_path, this->imu_calibration_param_.ACC_PARAM, param->x_k_, param->x_b_,
                                                 param->y_k_, param->y_b_, param->z_k_, param->z_b_);

      WJ_INFO << param->x_k_ << "," << param->x_b_ << "," << param->y_k_ << "," << param->y_b_ << "," << param->z_k_ << "," << param->z_b_ << WJ_REND;

      this->m_imu_params_get_->setImuAcceCalibrationParams(param->x_k_, param->x_b_, param->y_k_, param->y_b_, param->z_k_, param->z_b_);

      imu_ang_param_get_flag_ = true;
    }

    if (angle_param_get_flag_ && eccentricity_param_get_flag_ == 0x7fff && imu_acc_param_get_flag_ && imu_ang_param_get_flag_) {
      Decoder<T_PointCloud>::angles_ready_ = true;
    }

  } else if (typeid(*params) == typeid(Params_LiDARRunStatus)) {
    std::shared_ptr<Params_LiDARRunStatus> param = std::dynamic_pointer_cast<Params_LiDARRunStatus>(params);

    // WJ_INFOL << "imu_temp:" << param->imu_temp_ << WJ_REND;
    imu_temperature_ = param->imu_temp_ / 100.0f;
  } else if (typeid(*params) == typeid(Params_LDEccentricityParamGet720)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }
    if (eccentricity_param_get_flag_ != 0x7fff) {
      std::shared_ptr<Params_LDEccentricityParamGet720> param = std::dynamic_pointer_cast<Params_LDEccentricityParamGet720>(params);
      for (int num_of_angle = 0; num_of_angle < param->frame_len_; num_of_angle++) {
        eccentricity_angles_[(param->frame_id_ - 1) * param->frame_len_ + num_of_angle] = (int32_t)(param->eccentricity_angle_[num_of_angle] * 50);
        eccentricity_angles_real_[(param->frame_id_ - 1) * param->frame_len_ + num_of_angle] =
            (int32_t)(param->eccentricity_angle_[num_of_angle] * 50);
      }
      eccentricity_param_get_flag_ |= (0x01 << (param->frame_id_ - 1));

      if (this->param_.send_packet_enable) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[param->frame_id_].setSendInterval(5000);
      } else {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[param->frame_id_].setStopFlag(true);
      }

      if (eccentricity_param_get_flag_ == 0x7fff && Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        this->chan_angles_.loadFromLiDAR(this->param_.angle_path_hor, param->frame_len_ * param->frame_id_, eccentricity_angles_);
        WJ_INFOL << "Get LiDAR<LD_HOR> angle data..." << WJ_REND;
      }
    }

    if (angle_param_get_flag_ && eccentricity_param_get_flag_ == 0x7fff && imu_acc_param_get_flag_ && imu_ang_param_get_flag_) {
      Decoder<T_PointCloud>::angles_ready_ = true;
    }
  } else if (typeid(*params) == typeid(Params_Sn720)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_Sn720> param = std::dynamic_pointer_cast<Params_Sn720>(params);
    if (lidar_type_ == 0) {
      std::string sn;
      for (uint32_t i = 0; i < param->sn_.size(); i++) {
        if (param->sn_[i] == '\0')
          break;
        sn += param->sn_[i];
      }

      if (sn.find('c') != std::string::npos || sn.find('C') != std::string::npos) {
        lidar_type_ = 1;
      } else {
        lidar_type_ = 2;
      }

      if (this->param_.send_packet_enable) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setSendInterval(5000);
      } else {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
    }
  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar

}  // namespace vanjee
