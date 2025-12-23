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
#include <vanjee_driver/driver/decoder/decoder_packet_base/decoder_packet_base.hpp>
#include <vanjee_driver/driver/decoder/decoder_packet_base/imu/imuParamGet.hpp>
#include <vanjee_driver/driver/decoder/imu_calibration_param.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/cmd_repository_722f.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_error_code_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_firmware_version_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_imu_packet_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_ldvalue_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_sn_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_temperature_param_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_work_mode_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722f/protocol/frames/protocol_work_mode_set.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee722fChannel {
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t tag;
} Vanjee722fChannel;

typedef struct _Vanjee722fDifop {
  uint8_t mac_id[2];
  uint16_t circle_id;
  uint32_t sec;
  uint32_t nsec;
  uint8_t remain[2];
  int16_t imu_linear_acce_x;
  int16_t imu_linear_acce_y;
  int16_t imu_linear_acce_z;
  int16_t imu_angle_voc_x;
  int16_t imu_angle_voc_y;
  int16_t imu_angle_voc_z;
  uint8_t remain1[11];
  uint8_t return_wave_mode;
  uint16_t rpm;
  uint8_t device_id[3];
  uint8_t version;
  uint8_t remain2[12];
  uint16_t pkt_id;
  uint8_t tail[2];
} Vanjee722fDifop;

typedef struct _Vanjee722fBlockChannel16 {
  uint16_t azimuth;
  Vanjee722fChannel channel[16];
} Vanjee722fBlockChannel16;

typedef struct _Vanjee722fMsopPktChannel16 {
  uint8_t head[2];
  uint8_t channel_num;
  uint8_t return_wave_num;
  uint8_t block_num;
  Vanjee722fBlockChannel16 blocks[10];
  Vanjee722fDifop difop;
} Vanjee722fMsopPktChannel16;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee722F : public DecoderMech<T_PointCloud> {
 private:
  int32_t optcent_2_lidar_arg_ = 21086;
  float optcent_2_lidar_l_ = 2.112 * 1e-2;
  float optcent_2_lidar_z_ = 6.39 * 1e-3;

  uint32_t frame_num_per_circle_ = 120;
  std::vector<std::vector<double>> all_points_luminous_moment_722_16_;
  const double luminous_period_of_ld_16_ = 8.33e-5;
  const double luminous_period_of_adjacent_ld_ = 5e-6;
  std::vector<uint32_t> all_points_luminous_moment_size_;
  int32_t pkt_id_in_circle_trans_pre_ = -1;

  int32_t pre_pkt_id_ = -1;
  int32_t pre_circle_id_ = -1;
  uint8_t publish_mode_ = 0;

  bool angle_param_get_flag_ = false;

  uint8_t protocol_version_ = 0;
  int32_t pre_imu_frame_id_ = -1;
  double pre_imu_timestamp_ = 0.0;
  uint32_t valid_point_num_ = 0;

  int32_t pre_azimuth_ = 0;
  double pre_blk_timestamp_ = 0;

  std::map<uint16, std::string> get_lidar_param_;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);
  void initLdLuminousMoment(void);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 100;
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  virtual bool decodeMsopPktChannel16(const uint8_t *pkt, size_t size);
  virtual bool decodeMsopPktChannel16V3(const uint8_t *pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee722F() = default;
  explicit DecoderVanjee722F(const WJDecoderParam &param);

  void SendImuData(Vanjee722fDifop difop, double temperature, double timestamp, double lidar_timestamp);

 public:
  double imu_temperature_;
};

template <typename T_PointCloud>
void DecoderVanjee722F<T_PointCloud>::initLdLuminousMoment() {
  all_points_luminous_moment_722_16_.resize(2);
  all_points_luminous_moment_722_16_[0].resize(76800);  // 38400
  all_points_luminous_moment_722_16_[1].resize(38400);  // 19200
  all_points_luminous_moment_size_.resize(2);
  all_points_luminous_moment_size_[0] = 38400;
  all_points_luminous_moment_size_[1] = 19200;
  for (uint16_t col = 0; col < 4800; col++) {
    for (uint8_t row = 0; row < 16; row++) {
      if (col < 2400) {
        for (int i = 0; i < 2; i++) {
          all_points_luminous_moment_722_16_[i][col * 16 + row] = col * luminous_period_of_ld_16_ + row * luminous_period_of_adjacent_ld_;
        }
      }
      all_points_luminous_moment_722_16_[0][col * 16 + row] = col * luminous_period_of_ld_16_ + row * luminous_period_of_adjacent_ld_;
    }
  }
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee722F<T_PointCloud>::getConstParam(uint8_t mode) {
  // WJ_INFOL << "publish_mode ============mode=================" << mode << WJ_REND;
  uint16_t msop_len = 725;
  uint16_t laser_num = 16;
  uint16_t block_num = 10;
  uint16_t chan_num = 16;
  float distance_min = 0.05f;
  float distance_max = 70.0f;
  float distance_resolution = 0.004f;
  float init_temperature = 80.0f;

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
inline DecoderVanjee722F<T_PointCloud>::DecoderVanjee722F(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  publish_mode_ = param.publish_mode;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);
  this->m_imu_params_get_ = std::make_shared<ImuParamGet>(0, param.transform_param);
  imu_temperature_ = -100.0;
  if (param.imu_enable == -1) {
    this->point_cloud_ready_ = true;
  } else if (param.imu_enable == 0) {
    imu_temperature_ = 40.0;
    this->point_cloud_ready_ = true;
  } else {
    WJ_INFO << "Waiting for IMU calibration..." << WJ_REND;
  }
  this->start_angle_ = this->param_.start_angle * 1000;
  this->end_angle_ = this->param_.end_angle * 1000;

  if (this->param_.config_from_file) {
    this->chan_angles_.loadFromFile(this->param_.angle_path_ver);
  }
  initLdLuminousMoment();
}

template <typename T_PointCloud>
inline bool DecoderVanjee722F<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  uint16_t l_pktheader = pkt[0] << 8 | pkt[1];
  switch (l_pktheader) {
    case 0xFFDD: {
      if (size == sizeof(Vanjee722fMsopPktChannel16)) {
        uint32_t version_index = offsetof(Vanjee722fMsopPktChannel16, difop) + offsetof(Vanjee722fDifop, version);
        if (pkt[version_index] < 3) {
          ret = decodeMsopPktChannel16(pkt, size);
        } else {
          ret = decodeMsopPktChannel16V3(pkt, size);
        }
      }
    } break;
    default:
      break;
  }

  return ret;
}

template <typename T_PointCloud>
bool DecoderVanjee722F<T_PointCloud>::decodeMsopPktChannel16(const uint8_t *pkt, size_t size) {
  auto &packet = *(Vanjee722fMsopPktChannel16 *)pkt;
  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  pkt_host_ts = getTimeHost() * 1e-6;
  pkt_lidar_ts = packet.difop.sec + packet.difop.nsec * 1e-9;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else {
    pkt_ts = pkt_lidar_ts < 0 ? 0 : pkt_lidar_ts;
  }

  protocol_version_ = packet.difop.version;

  int32_t resolution = 30;
  uint8_t resolution_index = 1;
  if (packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0) {
    resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  } else {
    resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  }
  if (resolution == 0) {
    this->prev_pkt_ts_ = pkt_ts;
    return ret;
  } else if (resolution <= 22) {
    resolution = 15;
    resolution_index = 0;
    frame_num_per_circle_ = 240 * packet.return_wave_num;
  } else {
    resolution = 30;
    resolution_index = 1;
    frame_num_per_circle_ = 120 * packet.return_wave_num;
  }

  if (!this->param_.point_cloud_enable) {
    this->prev_pkt_ts_ = pkt_ts;
    return false;
  }

  uint16_t pkt_id = ntohs(packet.difop.pkt_id);
  uint32_t loss_packets_num = (pkt_id + frame_num_per_circle_ - pre_pkt_id_) % frame_num_per_circle_;
  if (loss_packets_num > 1 && pre_pkt_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_pkt_id_ = pkt_id;

  if (!this->imu_ready_ && !this->point_cloud_ready_) {
    this->prev_pkt_ts_ = pkt_ts;
    return ret;
  } else if (!this->point_cloud_ready_) {
    this->point_cloud_ready_ = true;
  }

  uint32_t loss_circles_num = (ntohs(packet.difop.circle_id) + 65536 - pre_circle_id_) % 65536;
  if (loss_circles_num > 1) {
    this->point_cloud_->points.clear();
  }
  uint16_t block_num = packet.block_num * packet.return_wave_num;

  uint32_t pkt_id_in_circle = pkt_id;
  uint32_t pkt_id_in_circle_trans = (pkt_id + 1) % frame_num_per_circle_;
  if ((this->split_strategy_->newBlock(pkt_id_in_circle_trans) && pkt_id_in_circle_trans != 0) ||
      (loss_circles_num == 1 && pkt_id_in_circle != 0 && this->point_cloud_->points.size() != 0)) {
    uint32_t point_gap_num = (pkt_id_in_circle * block_num) * packet.channel_num + 1;
    this->last_point_ts_ = pkt_ts - all_points_luminous_moment_722_16_[resolution_index][point_gap_num];
    this->first_point_ts_ =
        this->last_point_ts_ - all_points_luminous_moment_722_16_[resolution_index][all_points_luminous_moment_size_[resolution_index] - 1];

    if (valid_point_num_ > 0) {
      valid_point_num_ = 0;
      this->cb_split_frame_(packet.channel_num, this->cloudTs());
    } else {
      this->point_cloud_->points.clear();
    }
    ret = true;
  }

  for (uint16_t blk = 0; blk < block_num; blk++) {
    if (packet.return_wave_num == 2 && publish_mode_ == 0 && (blk % 2 == 1)) {
      continue;
    } else if (packet.return_wave_num == 2 && publish_mode_ == 1 && (blk % 2 == 0)) {
      continue;
    }

    const Vanjee722fBlockChannel16 &block = packet.blocks[blk];
    int32_t azimuth = block.azimuth % 36000;

    {
      double timestamp_point;
      uint32_t cur_blk_first_point_id = (pkt_id_in_circle * block_num + blk) * packet.channel_num;
      int32_t azimuth_10 = azimuth * 10;
      for (uint16_t chan = 0; chan < packet.channel_num; chan++) {
        float x, y, z, xy;

        uint32_t point_id = cur_blk_first_point_id + chan;
        if (this->param_.ts_first_point == true) {
          timestamp_point = all_points_luminous_moment_722_16_[resolution_index][point_id];
        } else {
          timestamp_point = all_points_luminous_moment_722_16_[resolution_index][point_id] -
                            all_points_luminous_moment_722_16_[resolution_index][all_points_luminous_moment_size_[resolution_index] - 1];
        }

        const Vanjee722fChannel &channel = block.channel[chan];
        if (channel.distance > 0) {
          valid_point_num_++;
        }

        float distance = channel.distance * this->const_param_.DISTANCE_RES;
        int32_t angle_vert = (this->chan_angles_.vertAdjust(chan) + 360000) % 360000;
        int32_t angle_horiz = (this->chan_angles_.horizAdjust(chan, azimuth_10) + 360000) % 360000;

        if (this->distance_section_.in(distance)) {
          int32_t optcent_2_lidar_angle_hor = (azimuth_10 + optcent_2_lidar_arg_ + 360000) % 360000;
          CenterCompensationParams optical_center_compensation_param;
          this->calcOpticalCenterCompensation(optical_center_compensation_param, optcent_2_lidar_angle_hor, optcent_2_lidar_l_, optcent_2_lidar_z_);

          xy = distance * COS(angle_vert);
          x = xy * SIN(angle_horiz) + optical_center_compensation_param.x;
          y = xy * COS(angle_horiz) + optical_center_compensation_param.y;
          z = distance * SIN(angle_vert) + optical_center_compensation_param.z;
          uint32_t angle_horiz_mask = angle_horiz;
#ifdef ENABLE_GTEST
          distance = std::pow(x * x + y * y + z * z, 0.5);
          angle_horiz_mask = this->coordTransAzimuth(x, y, 1);
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

          if (this->hide_range_flag_ && distance != 0 &&
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
          setTag(point, channel.tag);
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
          setTag(point, channel.tag);
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
  }
  if (pkt_id_in_circle_trans == 0) {
    uint32_t point_gap_num = block_num * packet.channel_num - 1;
    this->last_point_ts_ = pkt_ts + all_points_luminous_moment_722_16_[resolution_index][point_gap_num];
    this->first_point_ts_ =
        this->last_point_ts_ - all_points_luminous_moment_722_16_[resolution_index][all_points_luminous_moment_size_[resolution_index] - 1];

    if (valid_point_num_ > 0) {
      valid_point_num_ = 0;
      this->cb_split_frame_(packet.channel_num, this->cloudTs());
    } else {
      this->point_cloud_->points.clear();
    }
    ret = true;
  }
  pkt_id_in_circle_trans_pre_ = pkt_id_in_circle_trans;

  pre_circle_id_ = ntohs(packet.difop.circle_id);
  this->prev_pkt_ts_ = pkt_lidar_ts;
  return ret;
}

template <typename T_PointCloud>
bool DecoderVanjee722F<T_PointCloud>::decodeMsopPktChannel16V3(const uint8_t *pkt, size_t size) {
  const double time_duration_16 = 8.1e-5;
  auto &packet = *(Vanjee722fMsopPktChannel16 *)pkt;
  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  pkt_host_ts = getTimeHost() * 1e-6;
  pkt_lidar_ts = packet.difop.sec + packet.difop.nsec * 1e-9;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else {
    pkt_ts = pkt_lidar_ts < 0 ? 0 : pkt_lidar_ts;
  }

  protocol_version_ = packet.difop.version;

  if (!this->param_.point_cloud_enable) {
    this->prev_pkt_ts_ = pkt_ts;
    return false;
  }

  uint16_t pkt_id = ntohs(packet.difop.pkt_id);
  uint32_t loss_packets_num = 0;
  if (pkt_id < pre_pkt_id_) {
    if (pkt_id != 0) {
      loss_packets_num = pkt_id;
    }
  } else {
    loss_packets_num = pkt_id - pre_pkt_id_;
  }
  if (loss_packets_num > 1 && pre_pkt_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_pkt_id_ = pkt_id;

  if (!this->imu_ready_ && !this->point_cloud_ready_) {
    this->prev_pkt_ts_ = pkt_ts;
    return ret;
  } else if (!this->point_cloud_ready_) {
    this->point_cloud_ready_ = true;
  }

  uint32_t loss_circle_num = (ntohs(packet.difop.circle_id) + 65536 - pre_circle_id_) % 65536;
  if (loss_circle_num > 1) {
    this->point_cloud_->points.clear();
  }

  uint16_t block_num = packet.block_num * packet.return_wave_num;
  int blk_stay_num = 0;
  for (uint16_t blk = 0; blk < block_num; blk++) {
    if (packet.return_wave_num == 2 && publish_mode_ == 0 && (blk % 2 == 1)) {
      continue;
    } else if (packet.return_wave_num == 2 && publish_mode_ == 1 && (blk % 2 == 0)) {
      continue;
    }
    const Vanjee722fBlockChannel16 &block = packet.blocks[blk];
    int32_t azimuth = block.azimuth % 36000;
    if (this->split_strategy_->newBlock(azimuth) && (ntohs(packet.difop.circle_id) != pre_circle_id_)) {
      if (valid_point_num_ > 0 && this->first_point_ts_ > 0) {
        valid_point_num_ = 0;
        double timestamp = 0;
        if (this->param_.ts_first_point == true) {
          for (auto &pt : this->point_cloud_->points) {
            getTimestamp(pt, timestamp);
            setTimestamp(pt, timestamp - this->first_point_ts_);
          }
        } else {
          for (auto &pt : this->point_cloud_->points) {
            getTimestamp(pt, timestamp);
            setTimestamp(pt, timestamp - this->last_point_ts_);
          }
        }

        this->cb_split_frame_(packet.channel_num, this->cloudTs());
      } else {
        this->point_cloud_->points.clear();
      }
      this->first_point_ts_ = pkt_ts + time_duration_16 * blk;
      ret = true;
    }

    int32_t azimuth_10 = azimuth * 10;
    double blk_timestamp = 0;
    if ((azimuth + 36000 - pre_azimuth_) % 36000 > 10) {
      blk_timestamp = pkt_ts + time_duration_16 * (blk - blk_stay_num);
    } else {
      blk_stay_num++;
      blk_timestamp = pre_blk_timestamp_;
    }
    for (uint16_t chan = 0; chan < packet.channel_num; chan++) {
      float x, y, z, xy;

      double point_timestamp =
          blk_timestamp + luminous_period_of_adjacent_ld_ * ((blk_timestamp == pre_blk_timestamp_) ? (packet.channel_num - 1) : chan);

      const Vanjee722fChannel &channel = block.channel[chan];
      if (channel.distance > 0) {
        valid_point_num_++;
      }

      float distance = channel.distance * this->const_param_.DISTANCE_RES;
      int32_t angle_vert = (this->chan_angles_.vertAdjust(chan) + 360000) % 360000;
      int32_t angle_horiz = (this->chan_angles_.horizAdjust(chan, azimuth_10) + 360000) % 360000;

      if (this->distance_section_.in(distance)) {
        int32_t optcent_2_lidar_angle_hor = (azimuth_10 + optcent_2_lidar_arg_ + 360000) % 360000;
        CenterCompensationParams optical_center_compensation_param;
        this->calcOpticalCenterCompensation(optical_center_compensation_param, optcent_2_lidar_angle_hor, optcent_2_lidar_l_, optcent_2_lidar_z_);

        xy = distance * COS(angle_vert);
        x = xy * SIN(angle_horiz) + optical_center_compensation_param.x;
        y = xy * COS(angle_horiz) + optical_center_compensation_param.y;
        z = distance * SIN(angle_vert) + optical_center_compensation_param.z;
        uint32_t angle_horiz_mask = angle_horiz;
#ifdef ENABLE_GTEST
        distance = std::pow(x * x + y * y + z * z, 0.5);
        angle_horiz_mask = this->coordTransAzimuth(x, y, 1);
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

        if (this->hide_range_flag_ && distance != 0 &&
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
        setTimestamp(point, point_timestamp);
        setRing(point, chan + this->first_line_id_);
        setTag(point, channel.tag);
#ifdef ENABLE_GTEST
        setPointId(point, (pkt_id * block_num + blk) * packet.channel_num + chan);
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
        setTimestamp(point, point_timestamp);
        setRing(point, chan + this->first_line_id_);
        setTag(point, channel.tag);
#ifdef ENABLE_GTEST
        setPointId(point, (pkt_id * block_num + blk) * packet.channel_num + chan);
        setHorAngle(point, angle_horiz / 1000.0);
        setVerAngle(point, angle_vert / 1000.0);
        setDistance(point, distance);
#endif
        this->point_cloud_->points.emplace_back(point);
      }
    }

    this->last_point_ts_ = pkt_ts + time_duration_16 * (blk + 1) - luminous_period_of_adjacent_ld_;
    pre_azimuth_ = azimuth;
    pre_blk_timestamp_ = blk_timestamp;
  }

  pre_circle_id_ = ntohs(packet.difop.circle_id);
  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee722F<T_PointCloud>::SendImuData(Vanjee722fDifop difop, double temperature, double timestamp, double lidar_timestamp) {
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
void DecoderVanjee722F<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract722F> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_ld_value_get_)) {
    p = std::make_shared<Protocol_LDValueGet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_get_work_mode_)) {
    p = std::make_shared<Protocol_WorkModeGet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_set_work_mode_)) {
    p = std::make_shared<Protocol_WorkModeSet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_get_imu_packet_)) {
    p = std::make_shared<Protocol_ImuPacketGet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_get_error_code_)) {
    p = std::make_shared<Protocol_ErrorCodeGet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_temperature_param_get_)) {
    p = std::make_shared<Protocol_TemperatureParamGet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_firmware_version_get_)) {
    p = std::make_shared<Protocol_FirmwareVersionGet722F>();
  } else if (*sp_cmd == *(CmdRepository722F::CreateInstance()->sp_sn_get_)) {
    p = std::make_shared<Protocol_SnGet722F>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_LDValue722F)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    if (!angle_param_get_flag_) {
      std::shared_ptr<Params_LDValue722F> param = std::dynamic_pointer_cast<Params_LDValue722F>(params);
      std::vector<double> vert_angles;
      std::vector<double> offset_angles;
      for (int num_of_lines = 0; num_of_lines < param->num_of_lines_; num_of_lines++) {
        vert_angles.push_back((double)(param->ver_angle_[num_of_lines] / 1000.0));
        offset_angles.push_back((double)(param->offset_angle_[num_of_lines] / 1000.0));
      }

      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver, param->num_of_lines_, vert_angles, offset_angles);
        WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
        if (this->param_.send_packet_enable) {
          (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setSendInterval(5000);
        } else {
          (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
        }
      }

      angle_param_get_flag_ = true;
    }

    if (angle_param_get_flag_) {
      Decoder<T_PointCloud>::angles_ready_ = true;
    }
  } else if (typeid(*params) == typeid(Params_WorkModeGet722F)) {
    std::shared_ptr<Params_WorkModeGet722F> param = std::dynamic_pointer_cast<Params_WorkModeGet722F>(params);
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    uint16_t work_mode = 0xffff;
    if (param->work_mode_ == 0) {
      work_mode = 1;
    } else if (param->work_mode_ == 1) {
      work_mode = 0;
    } else {
      work_mode = param->work_mode_;
    }
    if (get_lidar_param_.count((uint16_t)LidarParam::work_mode) > 0) {
      get_lidar_param_[(uint16_t)LidarParam::work_mode] = std::to_string(work_mode);
    } else {
      get_lidar_param_.emplace((uint16_t)LidarParam::work_mode, std::to_string(work_mode));
    }

    if (this->param_.device_ctrl_state_enable) {
      this->device_ctrl_->cmd_id = 1;
      this->device_ctrl_->cmd_param = work_mode;
      this->device_ctrl_->cmd_state = 1;
      this->cb_device_ctrl_state_(this->prev_pkt_ts_);
    }

    if (this->param_.send_lidar_param_enable) {
      LidarParameterInterface lidar_param;
      lidar_param.cmd_id = (uint16_t)LidarParam::work_mode;
      lidar_param.cmd_type = 0;
      lidar_param.repeat_interval = 0;
      this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
      this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);
    }
  } else if (typeid(*params) == typeid(Params_WorkModeSet722F)) {
    std::shared_ptr<Params_WorkModeSet722F> param = std::dynamic_pointer_cast<Params_WorkModeSet722F>(params);
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository722F::CreateInstance()->sp_get_work_mode_->GetCmdKey()].setStopFlag(false);
  } else if (typeid(*params) == typeid(Params_ImuPacketGet722F)) {
    std::shared_ptr<Params_ImuPacketGet722F> param = std::dynamic_pointer_cast<Params_ImuPacketGet722F>(params);
    if (protocol_version_ < 2 || this->param_.imu_enable == -1)
      return;
    double pkt_ts = 0.0;
    double imu_timestamp = param->imu_sec_ + (param->imu_nsec_ * 1e-9);
    if (this->param_.use_lidar_clock)
      pkt_ts = imu_timestamp;
    else
      pkt_ts = getTimeHost() * 1e-6;

    uint32_t loss_packets_num = (param->frame_id_ + 65536 - pre_imu_frame_id_) % 65536;
    if (loss_packets_num > 1 && pre_imu_frame_id_ >= 0)
      WJ_WARNING << "loss " << (loss_packets_num - 1) << " imu packets" << WJ_REND;
    pre_imu_frame_id_ = param->frame_id_;

    if (pre_imu_timestamp_ > 0 && imu_timestamp > pre_imu_timestamp_) {
      this->imu_ready_ = this->m_imu_params_get_->imuGet(
          param->imu_angle_voc_x_, param->imu_angle_voc_y_, param->imu_angle_voc_z_, param->imu_linear_acce_x_, param->imu_linear_acce_y_,
          param->imu_linear_acce_z_, imu_timestamp, this->param_.imu_orientation_enable, -100.0, false, false, false, true, false);
      if (this->imu_ready_) {
        this->imu_packet_->timestamp = pkt_ts;
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
    pre_imu_timestamp_ = imu_timestamp;

  } else if (typeid(*params) == typeid(Params_ErrorCodeGet722F)) {
    std::shared_ptr<Params_ErrorCodeGet722F> param = std::dynamic_pointer_cast<Params_ErrorCodeGet722F>(params);
    uint16_t device_state = (param->cmd_param_ & 0xffff) != 0 ? 1 : 0;
    if (device_state != 0) {
      this->deviceStatePublish(param->cmd_id_, param->cmd_param_, device_state, this->prev_pkt_ts_);
    }
  } else if (typeid(*params) == typeid(Params_TemperatureParamGet722F)) {
    std::shared_ptr<Params_TemperatureParamGet722F> param = std::dynamic_pointer_cast<Params_TemperatureParamGet722F>(params);

    // double temperature = (double)param->lidar_temp_ * 1e-2;
    // WJ_INFOL << "Temperature: " << temperature << " °C" << WJ_REND;

    if (get_lidar_param_.count((uint16_t)LidarParam::temperature > 0)) {
      get_lidar_param_[(uint16_t)LidarParam::temperature] = std::to_string(param->lidar_temp_ * 1e-2);
    } else {
      get_lidar_param_.emplace((uint16_t)LidarParam::temperature, std::to_string(param->lidar_temp_ * 1e-2));
    }

    if (this->param_.device_ctrl_state_enable) {
      this->device_ctrl_->cmd_id = 2;
      this->device_ctrl_->cmd_param = (uint16_t)param->lidar_temp_;
      this->device_ctrl_->cmd_state = 1;
      this->cb_device_ctrl_state_(this->prev_pkt_ts_);
    }

    if (this->param_.send_lidar_param_enable) {
      LidarParameterInterface lidar_param;
      lidar_param.cmd_id = (uint16_t)LidarParam::temperature;
      lidar_param.cmd_type = 0;

      if ((*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].getSendInterval() <= 100) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
        lidar_param.repeat_interval = 0;
      } else {
        lidar_param.repeat_interval = (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].getSendInterval();
      }

      this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
      this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);
    }

  } else if (typeid(*params) == typeid(Params_FirmwareVersion722F)) {
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    std::shared_ptr<Params_FirmwareVersion722F> param = std::dynamic_pointer_cast<Params_FirmwareVersion722F>(params);

    if (get_lidar_param_.count((uint16_t)LidarParam::firmware_version) > 0) {
      get_lidar_param_[(uint16_t)LidarParam::firmware_version] = param->firmware_version_;
    } else {
      get_lidar_param_.emplace((uint16_t)LidarParam::firmware_version, param->firmware_version_);
    }

    if (this->param_.send_lidar_param_enable) {
      LidarParameterInterface lidar_param;
      lidar_param.cmd_id = (uint16_t)LidarParam::firmware_version;
      lidar_param.cmd_type = 0;
      lidar_param.repeat_interval = 0;
      this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
      this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);
    }

    WJ_INFOL << "Get lidar firmware version succ ( " << param->firmware_version_ << " )" << WJ_REND;

  } else if (typeid(*params) == typeid(Params_Sn722F)) {
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    std::shared_ptr<Params_Sn722F> param = std::dynamic_pointer_cast<Params_Sn722F>(params);

    if (get_lidar_param_.count((uint16_t)LidarParam::sn) > 0) {
      get_lidar_param_[(uint16_t)LidarParam::sn] = param->sn_;
    } else {
      get_lidar_param_.emplace((uint16_t)LidarParam::sn, param->sn_);
    }

    if (this->param_.send_lidar_param_enable) {
      LidarParameterInterface lidar_param;
      lidar_param.cmd_id = (uint16_t)LidarParam::sn;
      lidar_param.cmd_type = 0;
      lidar_param.repeat_interval = 0;
      this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
      this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);
    }

    WJ_INFOL << "Get lidar sn succ ( " << param->sn_ << " )" << WJ_REND;

  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar

}  // namespace vanjee
