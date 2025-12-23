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
#ifdef FILTER_DLL_ENABLE
#include <vanjee_driver/driver/decoder/wlr760/algorithm/filterate_760.h>
#endif
#include <vanjee_driver/driver/decoder/wlr760/protocol/frames/cmd_repository_760.hpp>
#include <vanjee_driver/driver/decoder/wlr760/protocol/frames/protocol_heartbeat.hpp>
#include <vanjee_driver/driver/decoder/wlr760/protocol/frames/protocol_protocol_version_set.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)

typedef struct _Vanjee760ChannelData {
  uint16_t distance;
  uint8_t reflectivity;
} Vanjee760ChannelData;

typedef struct _Vanjee760Block {
  uint8_t data_header;
  uint16_t azimuth;
  uint8_t rotate_mirror_id;
  int16_t ver_offset_angle;
  int16_t hor_offset_angle;
  Vanjee760ChannelData channel[192];
} Vanjee760Block;

typedef struct _Vanjee760DifopPkt {
  uint8_t param_identify_code[4];
  uint16_t circle_id;
  uint16_t pkt_id;
  uint8_t device_type;
  uint8_t return_wave_num;
  uint16_t rpm;
  uint32_t second;
  uint32_t microsecond;
  uint16_t frame_len;
  uint8_t remain[4];
  uint16_t tail;
} Vanjee760DifopPkt;

typedef struct _Vanjee760MsopPkt {
  uint16_t header;
  Vanjee760Block blocks[2];
  Vanjee760DifopPkt difop;
} Vanjee760MsopPkt;

typedef struct _Vanjee760ChannelDataPulseWidth {
  uint16_t distance;
  uint8_t reflectivity;
  uint16_t low_thres_pulse_width;
  uint16_t high_thres_pulse_width;
} Vanjee760ChannelDataPulseWidth;

typedef struct _Vanjee760BlockPulseWidth {
  uint16_t azimuth;
  uint8_t rotate_mirror_id;
  int16_t ver_offset_angle;
  int16_t hor_offset_angle;
  Vanjee760ChannelDataPulseWidth channel[192];
} Vanjee760BlockPulseWidth;

typedef struct _Vanjee760MsopPktPulseWidth {
  uint16_t header;
  Vanjee760BlockPulseWidth block;
  Vanjee760DifopPkt difop;
} Vanjee760MsopPktPulseWidth;

typedef struct _Vanjee760DataUnit {
  uint16_t distance;
  uint8_t reflectivity;
} Vanjee760DataUnit;

typedef struct _Vanjee760DataBlock {
  uint8_t start_flag;
  uint16_t azimuth_offset;
  uint16_t elevation_offset;
  Vanjee760DataUnit channel[192];
} Vanjee760DataBlock;

typedef struct _Vanjee760DataBlockV1_1 {
  uint8_t start_flag;
  uint16_t azimuth_offset;
  uint16_t elevation_offset;
  Vanjee760DataUnit channel[192];
  uint8_t confidence[72];
} Vanjee760DataBlockV1_1;

typedef struct _Vanjee760DataUnitV1_132 {
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t tag;
} Vanjee760DataUnitV1_132;

typedef struct _Vanjee760DataBlockV1_132 {
  uint8_t start_flag;
  uint16_t azimuth_offset;
  uint16_t elevation_offset;
  Vanjee760DataUnitV1_132 channel[192];
} Vanjee760DataBlockV1_132;

typedef struct _ProtocolVersion760 {
  uint8_t main_version_id_;
  uint8_t sub_version_id_;
  uint16_t getVersionId() {
    return (main_version_id_ << 8) + sub_version_id_;
  }
} ProtocolVersion760;

typedef struct _PointCloudPacketBaseHeader760 {
  uint8_t header_[2];
  uint16_t len_;
  uint8_t lidar_type_[2];
  ProtocolVersion760 protocol_version_;
  uint16_t getLidarType() {
    return (lidar_type_[0] << 8) + lidar_type_[1];
  }
} PointCloudPacketBaseHeader760;

typedef struct _DataBlockInfo760 {
  uint8_t data_block_packet_type_;
  uint16_t data_block_num_in_packet_;
  uint16_t valid_data_block_num_in_packet_;
  uint16_t row_channel_num_in_data_block_;
  uint16_t col_channel_num_in_data_block_;
  uint8_t distance_resolution_;
} DataBlockInfo760;

typedef struct _TimestampInfo760 {
  uint8_t clock_source_;
  uint32_t second_;
  uint32_t nanosecond_;
} TimestampInfo760;

typedef struct _PointCloudPacketHeader760V1_132 {
  PointCloudPacketBaseHeader760 base_header_;
  uint16_t circle_id_;
  uint16_t total_pkt_num_;
  uint16_t pkt_id_;
  uint16_t operate_frequency_;
  uint8_t echo_num_;
  uint16_t col_channel_num_;
  uint16_t row_channel_num_;
  uint8_t reserved_field_1_[8];
  DataBlockInfo760 data_block_info_;
  TimestampInfo760 timestamp_info_;
  uint8_t reserved_field_2_[16];
  uint16_t info_flag_;
  Vanjee760DataBlockV1_132 block_;
} PointCloudPacketHeader760V1_132;

typedef struct _PointCloudPacketHeader760V1_1 {
  PointCloudPacketBaseHeader760 base_header_;
  uint16_t circle_id_;
  uint16_t total_pkt_num_;
  uint16_t pkt_id_;
  uint16_t operate_frequency_;
  uint8_t echo_num_;
  uint16_t col_channel_num_;
  uint16_t row_channel_num_;
  uint8_t reserved_field_1_[8];
  DataBlockInfo760 data_block_info_;
  TimestampInfo760 timestamp_info_;
  uint8_t reserved_field_2_[16];
  uint16_t info_flag_;
  Vanjee760DataBlockV1_1 block_;
} PointCloudPacketHeader760V1_1;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee760 : public DecoderMech<T_PointCloud> {
 private:
  std::vector<double> all_points_luminous_moment_760_120degree_;
  std::vector<double> all_points_luminous_moment_760_140degree_;
  const double luminous_period_of_ld_ = 8.33333e-5;  // 0.0000925;

  int32_t pre_frame_id_ = -1;
  int32_t pre_circle_id_ = -1;
  uint8_t publish_mode_ = 0;
  uint16_t pre_operate_frequency_ = 0;
  uint32_t one_point_info_tmp_index_ = 0;
  uint16_t hor_resolution_ = 15;
  int32_t max_hor_angle_ = 14040;

  uint32_t update_row_num_ = 0;

  std::vector<int32_t> ver_angle_type1_ = std::vector<int32_t>{
      12471,  12333,  12195,  12058,  11920,  11783,  11646,  11509,  11373,  11237,  11100,  10965,  10829,  10694,  10558,  10423,  10288,  10154,
      10019,  9885,   9751,   9617,   9483,   9350,   9217,   9083,   8950,   8818,   8685,   8553,   8420,   8288,   8156,   8024,   7893,   7761,
      7630,   7499,   7368,   7237,   7106,   6975,   6845,   6715,   6584,   6454,   6324,   6195,   6065,   5935,   5806,   5677,   5547,   5418,
      5289,   5160,   5032,   4903,   4774,   4646,   4518,   4389,   4261,   4133,   4005,   3877,   3749,   3621,   3494,   3366,   3238,   3111,
      2983,   2856,   2729,   2601,   2474,   2347,   2220,   2093,   1966,   1839,   1712,   1585,   1458,   1331,   1204,   1077,   951,    824,
      697,    570,    444,    317,    190,    63,     -63,    -190,   -317,   -444,   -570,   -697,   -824,   -951,   -1077,  -1204,  -1331,  -1458,
      -1585,  -1712,  -1839,  -1966,  -2093,  -2220,  -2347,  -2474,  -2601,  -2729,  -2856,  -2983,  -3111,  -3238,  -3366,  -3494,  -3621,  -3749,
      -3877,  -4005,  -4133,  -4261,  -4389,  -4518,  -4646,  -4774,  -4903,  -5032,  -5160,  -5289,  -5418,  -5547,  -5677,  -5806,  -5935,  -6065,
      -6195,  -6324,  -6454,  -6584,  -6715,  -6845,  -6975,  -7106,  -7237,  -7368,  -7499,  -7630,  -7761,  -7893,  -8024,  -8156,  -8288,  -8420,
      -8553,  -8685,  -8818,  -8950,  -9083,  -9217,  -9350,  -9483,  -9617,  -9751,  -9885,  -10019, -10154, -10288, -10423, -10558, -10694, -10829,
      -10965, -11100, -11237, -11373, -11509, -11646, -11783, -11920, -12058, -12195, -12331, -12471};

  std::vector<int32_t> ver_angle_type2_ = std::vector<int32_t>{
      12500, 12237, 11974,  11711,  11447,  11184,  10921,  10658,  10395,  10132,  9868,   9605,  9342,  9079,  8816,  8553,  8289,  8026,
      7763,  7500,  7237,   6974,   6711,   6447,   6250,   6184,   6118,   5987,   5921,   5855,  5724,  5658,  5592,  5461,  5395,  5329,
      5197,  5132,  5066,   4934,   4868,   4803,   4671,   4605,   4539,   4408,   4342,   4276,  4145,  4079,  4013,  3882,  3816,  3750,
      3618,  3553,  3487,   3355,   3289,   3224,   3092,   3026,   2961,   2829,   2763,   2697,  2566,  2500,  2434,  2303,  2237,  2171,
      2039,  1974,  1908,   1776,   1711,   1645,   1513,   1447,   1382,   1250,   1184,   1118,  987,   921,   855,   724,   658,   592,
      461,   395,   329,    197,    132,    66,     -66,    -132,   -197,   -329,   -395,   -461,  -592,  -658,  -724,  -855,  -921,  -987,
      -1118, -1184, -1250,  -1382,  -1447,  -1513,  -1645,  -1711,  -1776,  -1908,  -1974,  -2039, -2171, -2237, -2303, -2434, -2500, -2566,
      -2697, -2763, -2829,  -2961,  -3026,  -3092,  -3224,  -3289,  -3355,  -3487,  -3553,  -3618, -3750, -3816, -3882, -4013, -4079, -4145,
      -4276, -4342, -4408,  -4539,  -4605,  -4671,  -4803,  -4868,  -4934,  -5066,  -5132,  -5197, -5329, -5395, -5461, -5592, -5658, -5724,
      -5855, -5921, -5987,  -6118,  -6184,  -6250,  -6447,  -6711,  -6974,  -7237,  -7500,  -7763, -8026, -8289, -8553, -8816, -9079, -9342,
      -9605, -9868, -10132, -10395, -10658, -10921, -11184, -11447, -11711, -11974, -12237, -12500};

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam& getConstParam(uint8_t mode);
  void initLdLuminousMoment(void);
  int32_t getTimeSequenceOffsetHorAngle(float speed, int line);

#ifdef FILTER_DLL_ENABLE
  int* cur_point_info_manager;
  int* pre_point_info_manager;
  Laser760Filter::ExportPointSizeCn export_point_size_cn_;
#endif

  void pointCloudAlgorithmCallbackV1_132();
  bool decodeMsopPktProtocolV1_1(const uint8_t* pkt, size_t size);
  bool decodeMsopPktProtocolV1_132(const uint8_t* pkt, size_t size);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 400;
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee760() {  // = default;
#ifdef FILTER_DLL_ENABLE
    Laser760Filter::delete_one_point_info_manage(cur_point_info_manager);
    Laser760Filter::delete_one_point_info_manage(pre_point_info_manager);
#endif
  }
  explicit DecoderVanjee760(const WJDecoderParam& param);
};

template <typename T_PointCloud>
void DecoderVanjee760<T_PointCloud>::initLdLuminousMoment() {
  double point_ts_offset = luminous_period_of_ld_ / 192;
  double glowing_moments[192] = {0};

  for (int i = 0; i < 192; i++) {
    glowing_moments[i] = point_ts_offset * i;
  }

  all_points_luminous_moment_760_120degree_.resize(153600);
  all_points_luminous_moment_760_140degree_.resize(179712);
  for (uint16_t col = 0; col < 936; col++) {
    for (uint8_t row = 0; row < 192; row++) {
      if (col < 800) {
        all_points_luminous_moment_760_120degree_[col * 192 + row] = col * luminous_period_of_ld_ + glowing_moments[row];
      }
      all_points_luminous_moment_760_140degree_[col * 192 + row] = col * luminous_period_of_ld_ + glowing_moments[row];
    }
  }
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam& DecoderVanjee760<T_PointCloud>::getConstParam(uint8_t mode) {
  static WJDecoderMechConstParam param = {
      1186  // msop len
      ,
      192  // laser number
      ,
      2  // blocks per packet
      ,
      192  // channels per block
      ,
      1.5f  // distance min
      ,
      260.0f  // distance max
      ,
      0.004f  // distance resolution
      ,
      80.0f  // initial value of temperature
  };
  param.BLOCK_DURATION = 0.1 / 800;
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee760<T_PointCloud>::DecoderVanjee760(const WJDecoderParam& param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  publish_mode_ = param.publish_mode;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);

  this->start_angle_ = this->param_.start_angle * 1000;
  this->end_angle_ = this->param_.end_angle * 1000;

#ifdef FILTER_DLL_ENABLE
  Laser760Filter::init_filter();
  cur_point_info_manager = Laser760Filter::create_one_point_info_manage();
  pre_point_info_manager = Laser760Filter::create_one_point_info_manage();
#endif

  initLdLuminousMoment();
}

template <typename T_PointCloud>
inline bool DecoderVanjee760<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size) {
  bool ret = false;
  uint16_t pkt_header = pkt[0] << 8 | pkt[1];
  uint16_t protocol_version = pkt[6] << 8 | pkt[7];
  switch (pkt_header) {
    case 0xFFCC: {
      switch (protocol_version) {
        case 0x0101:
          ret = decodeMsopPktProtocolV1_1(pkt, size);
          break;

        case 0x0184:
          ret = decodeMsopPktProtocolV1_132(pkt, size);
          break;

        default:
          break;
      }
    } break;

    default:
      break;
  }
  return ret;
}

template <typename T_PointCloud>
inline int32_t DecoderVanjee760<T_PointCloud>::getTimeSequenceOffsetHorAngle(float speed, int line) {
  int32_t h_offset = 0;
  if ((line >= 25 && line <= 48) || (line >= 169 && line <= 192)) {
    h_offset = speed * 126 * 1e-3;  // speed * 6 * 7 * 3 * 1e-3;   //18.9e-3
  } else if ((line >= 73 && line <= 96) || (line >= 121 && line <= 144)) {
    h_offset = speed * 252 * 1e-3;  // speed * 6 * 14 * 3 * 1e-3;    //37.8e-3
  } else if ((line >= 49 && line <= 72) || (line >= 97 && line <= 120)) {
    h_offset = speed * 216 * 1e-3;  // speed * 6 * 12 * 3 * 1e-3;    //32.4e-3
  } else if ((line >= 1 && line <= 24) || (line >= 145 && line <= 168)) {
    h_offset = speed * 342 * 1e-3;  // speed * 6 * 19 * 3 * 1e-3;    //51.3e-3
  }
  return h_offset;
}

template <typename T_PointCloud>
inline bool DecoderVanjee760<T_PointCloud>::decodeMsopPktProtocolV1_1(const uint8_t* pkt, size_t size) {
  bool ret = false;
  uint16_t offset = 0;
  PointCloudPacketHeader760V1_1& vanjee_lidar_point_cloud_packet_header = *((PointCloudPacketHeader760V1_1*)(pkt + offset));
  uint32_t point_num = vanjee_lidar_point_cloud_packet_header.col_channel_num_ * vanjee_lidar_point_cloud_packet_header.row_channel_num_;

  double pkt_ts = 0.0, host_pkt_ts = 0.0, lidar_pkt_ts = 0.0;
  host_pkt_ts = getTimeHost() * 1e-6;
  lidar_pkt_ts =
      vanjee_lidar_point_cloud_packet_header.timestamp_info_.second_ + vanjee_lidar_point_cloud_packet_header.timestamp_info_.nanosecond_ * 1e-9;
  if (!this->param_.use_lidar_clock) {
    pkt_ts = host_pkt_ts;
  } else {
    pkt_ts = lidar_pkt_ts < 0 ? 0 : lidar_pkt_ts;
  }

  if (!this->param_.point_cloud_enable)
    return false;

  uint32_t loss_circles_num = (vanjee_lidar_point_cloud_packet_header.circle_id_ + 0x10000 - pre_circle_id_) % 0x10000;
  pre_circle_id_ = vanjee_lidar_point_cloud_packet_header.circle_id_;
  if (loss_circles_num > 1) {
    this->point_cloud_->points.clear();
#ifdef FILTER_DLL_ENABLE
    Laser760Filter::clear_one_point_info_manage(pre_point_info_manager);
    Laser760Filter::clear_one_point_info_manage(cur_point_info_manager);
    export_point_size_cn_.echo1_data_size = 0;
    export_point_size_cn_.echo2_data_size = 0;
#endif
  }

#ifdef FILTER_DLL_ENABLE
  if ((loss_circles_num == 1 && (export_point_size_cn_.echo1_data_size + export_point_size_cn_.echo2_data_size) != 0)) {
    this->last_point_ts_ = this->first_point_ts_ + all_points_luminous_moment_760_140degree_[point_num - 1];

    pointCloudAlgorithmCallbackV1_132();
    this->cb_split_frame_(vanjee_lidar_point_cloud_packet_header.row_channel_num_ + update_row_num_, this->cloudTs());
    update_row_num_ = 0;
    ret = true;
  }
#endif

  uint16_t data_block_num_in_packet = vanjee_lidar_point_cloud_packet_header.data_block_info_.data_block_num_in_packet_;

  for (uint16_t i = 0; i < data_block_num_in_packet; i++) {
    if (i >= vanjee_lidar_point_cloud_packet_header.data_block_info_.valid_data_block_num_in_packet_)
      break;

    int32_t col_index =
        ((vanjee_lidar_point_cloud_packet_header.pkt_id_ - 1) * data_block_num_in_packet + i) / vanjee_lidar_point_cloud_packet_header.echo_num_;
    uint32_t point_offset_num = 0;
    if (!this->param_.use_lidar_clock) {
      point_offset_num = (col_index + 1) * vanjee_lidar_point_cloud_packet_header.row_channel_num_;
    } else {
      point_offset_num = col_index * vanjee_lidar_point_cloud_packet_header.row_channel_num_;
    }
    this->first_point_ts_ = pkt_ts - all_points_luminous_moment_760_140degree_[point_offset_num];

    uint16_t hor_resolution =
        (uint16_t)(vanjee_lidar_point_cloud_packet_header.reserved_field_1_[0] + (vanjee_lidar_point_cloud_packet_header.reserved_field_1_[1] << 8));
    uint8_t mirror_id = vanjee_lidar_point_cloud_packet_header.reserved_field_1_[4];
    uint16_t rpm =
        (uint16_t)(vanjee_lidar_point_cloud_packet_header.reserved_field_1_[5] + (vanjee_lidar_point_cloud_packet_header.reserved_field_1_[6] << 8));

    if (hor_resolution_ != hor_resolution / 10)
      hor_resolution_ = hor_resolution / 10;

    uint8_t cur_echo = 1;
    if (vanjee_lidar_point_cloud_packet_header.echo_num_ == 2) {
      cur_echo =
          ((vanjee_lidar_point_cloud_packet_header.pkt_id_ - 1) * vanjee_lidar_point_cloud_packet_header.data_block_info_.data_block_num_in_packet_) %
              2 +
          1;
    }

    uint8_t confidence[192];
    for (uint32_t j = 0; j < sizeof(confidence); j++) {
      int byte_index = (j * 3) / 8;
      int bit_offset = (j * 3) % 8;
      uint8_t value = (vanjee_lidar_point_cloud_packet_header.block_.confidence[byte_index] >> bit_offset) & 0x07;
      if (bit_offset > 5) {
        value |= (vanjee_lidar_point_cloud_packet_header.block_.confidence[byte_index + 1] << (8 - bit_offset)) & 0x07;
      }
      confidence[j] = value & 0x03;
    }
    uint32_t cur_blk_first_point_id = col_index * vanjee_lidar_point_cloud_packet_header.data_block_info_.row_channel_num_in_data_block_;

    if (vanjee_lidar_point_cloud_packet_header.data_block_info_.data_block_packet_type_ == 1) {
      for (int chan_id = 0; chan_id < vanjee_lidar_point_cloud_packet_header.data_block_info_.row_channel_num_in_data_block_; chan_id++) {
        int32_t row_index = chan_id;
        uint32_t point_id = cur_blk_first_point_id + row_index;
        double timestamp_point = 0.0;
        if (this->param_.ts_first_point)
          timestamp_point = all_points_luminous_moment_760_140degree_[point_id];
        else
          timestamp_point = all_points_luminous_moment_760_140degree_[point_id] - all_points_luminous_moment_760_140degree_[point_num - 1];

        int32_t rpm_offset_hor_angle = getTimeSequenceOffsetHorAngle(rpm / 100.0, chan_id + 1);
        int32_t hor_angle_y =
            ((hor_resolution * col_index + vanjee_lidar_point_cloud_packet_header.block_.azimuth_offset + rpm_offset_hor_angle) + 30000 + 270000) %
            360000;
        int32_t ver_angle = (ver_angle_type1_[row_index] + vanjee_lidar_point_cloud_packet_header.block_.elevation_offset + 360000) % 360000;
#ifdef FILTER_DLL_ENABLE
        size_t index = Laser760Filter::create_one_point_info_index(chan_id, col_index);
        bool old;
        Laser760Filter::ExportOnePointInfo* res = Laser760Filter::create_one_point_info(
            cur_point_info_manager, index, cur_echo, chan_id + this->first_line_id_, hor_resolution_ * col_index, old);
        res->index = index;
        res->line_id = chan_id + this->first_line_id_;
        res->col_index = col_index;
        res->mirror = mirror_id;
        res->hor_angle = (float)hor_angle_y * 1e-3;
        res->ver_angle = (float)ver_angle * 1e-3;

        res->distance = vanjee_lidar_point_cloud_packet_header.block_.channel[chan_id].distance * this->const_param_.DISTANCE_RES;
        res->distance_mm = (vanjee_lidar_point_cloud_packet_header.block_.channel[chan_id].distance * 4);
        res->cos_ride_sin = COS(ver_angle) * SIN(hor_angle_y);
        res->cos_rid_cos = COS(ver_angle) * COS(hor_angle_y);
        res->sin_rid_thousandth = SIN(ver_angle);
        res->x = res->distance * res->cos_ride_sin;
        res->y = res->distance * res->cos_rid_cos;
        res->z = res->distance * res->sin_rid_thousandth;
        res->reflectivity = vanjee_lidar_point_cloud_packet_header.block_.channel[chan_id].reflectivity;

        res->time = (uint64_t)((int64_t)(timestamp_point * 1e9));
        res->echo_type = cur_echo;
        res->hor_azimuth = hor_resolution_ * col_index;
        res->motor_speed = rpm;

        res->confidence = confidence[chan_id];
        res->low_pulse_width = confidence[chan_id];

        res->ground_flag = 0;
        res->non_filterable = 0;

        // res->tall_pulse_width = 0;
        // res->strong_weak_flag = 0;

        if (cur_echo == 1) {
          export_point_size_cn_.echo1_data_size++;
        } else {
          export_point_size_cn_.echo2_data_size++;
        }
#endif
      }
    }
  }

  if (vanjee_lidar_point_cloud_packet_header.pkt_id_ == vanjee_lidar_point_cloud_packet_header.total_pkt_num_) {
    this->last_point_ts_ = this->first_point_ts_ + all_points_luminous_moment_760_140degree_[point_num - 1];

    pointCloudAlgorithmCallbackV1_132();
    this->cb_split_frame_(vanjee_lidar_point_cloud_packet_header.row_channel_num_ + update_row_num_, this->cloudTs());
    update_row_num_ = 0;
    ret = true;
  }
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee760<T_PointCloud>::pointCloudAlgorithmCallbackV1_132() {
#ifdef FILTER_DLL_ENABLE
  Laser760Filter::update_add_point_size(cur_point_info_manager, export_point_size_cn_);
  Laser760Filter::filter(cur_point_info_manager, pre_point_info_manager);

  this->point_cloud_->points.clear();
  for (int col_index = 0; col_index < 802; col_index++) {
    for (int line_id = 0; line_id < 192; line_id++) {
      Laser760Filter::ExportOnePointInfo* res = Laser760Filter::get_one_point_info(cur_point_info_manager, 1, line_id + 1, col_index * 15);
      if (res->line_id == 0)
        continue;

      int32_t angle_horiz_mask = (360000 - (int32_t)(res->hor_angle * 1e3)) % 360000;
      if (this->start_angle_ < this->end_angle_ && (angle_horiz_mask < this->start_angle_ || angle_horiz_mask > this->end_angle_)) {
        res->distance = 0;
      } else if (this->start_angle_ > this->end_angle_ && (angle_horiz_mask > this->end_angle_ && angle_horiz_mask < this->start_angle_)) {
        res->distance = 0;
      }

      if (this->hide_range_params_.size() > 0 && res->distance != 0 &&
          this->isValueInRange(res->line_id, angle_horiz_mask, res->distance, this->hide_range_params_)) {
        res->distance = 0;
      }

      typename T_PointCloud::PointT point;
      if (this->distance_section_.in(res->distance) && res->non_filterable != 3 && res->non_filterable != 5 && res->non_filterable != 6 &&
          res->non_filterable != 10 && res->non_filterable != 11 && res->non_filterable != 12) {
        float x = res->y;
        float y = -res->x;
        float z = res->z;
        this->transformPoint(x, y, z);
        setX(point, x);
        setY(point, y);
        setZ(point, z);

        setIntensity(point, res->reflectivity);
      } else {
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
      }
      setTimestamp(point, (int64_t)res->time * 1e-9);
      setRing(point, res->line_id);
      setTag(point, 0);
#ifdef ENABLE_GTEST
      setPointId(point, res->col_index * ver_angle_type1_.size() + res->line_id - this->first_line_id_);
      setHorAngle(point, res->hor_angle);
      setVerAngle(point, res->ver_angle);
      setDistance(point, res->distance);
      setFlag(point, res->non_filterable);
#endif
      this->point_cloud_->points.emplace_back(std::move(point));
    }
  }
  Laser760Filter::ExportOnePointInfo* points = nullptr;
  size_t count = Laser760Filter::get_making_points(&points);
  update_row_num_ = count / 802;
  if (points && count > 0) {
    for (size_t i = 0; i < count; ++i) {
      typename T_PointCloud::PointT point;
      if (points[i].non_filterable == 13) {
        float x = points[i].y;
        float y = -points[i].x;
        float z = points[i].z;
        this->transformPoint(x, y, z);
        setX(point, x);
        setY(point, y);
        setZ(point, z);

        setIntensity(point, points[i].reflectivity);
      } else {
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
      }
      setTimestamp(point, (int64_t)points[i].time * 1e-9);
      setRing(point, points[i].line_id);
      setTag(point, 0);
#ifdef ENABLE_GTEST
      setPointId(point, points[i].col_index * ver_angle_type1_.size() + points[i].line_id - this->first_line_id_);
      setHorAngle(point, points[i].hor_angle);
      setVerAngle(point, points[i].ver_angle);
      setDistance(point, points[i].distance);
      setFlag(point, points[i].non_filterable);
#endif
      this->point_cloud_->points.emplace_back(std::move(point));
    }
  }

  std::swap(cur_point_info_manager, pre_point_info_manager);
  Laser760Filter::clear_one_point_info_manage(cur_point_info_manager);
  export_point_size_cn_.echo1_data_size = 0;
  export_point_size_cn_.echo2_data_size = 0;
#endif
}

template <typename T_PointCloud>
inline bool DecoderVanjee760<T_PointCloud>::decodeMsopPktProtocolV1_132(const uint8_t* pkt, size_t size) {
  bool ret = false;
  uint16_t offset = 0;
  PointCloudPacketHeader760V1_132& vanjee_lidar_point_cloud_packet_header = *((PointCloudPacketHeader760V1_132*)(pkt + offset));
  uint32_t point_num = vanjee_lidar_point_cloud_packet_header.col_channel_num_ * vanjee_lidar_point_cloud_packet_header.row_channel_num_;

  double pkt_ts = 0.0, host_pkt_ts = 0.0, lidar_pkt_ts = 0.0;
  host_pkt_ts = getTimeHost() * 1e-6;
  lidar_pkt_ts =
      vanjee_lidar_point_cloud_packet_header.timestamp_info_.second_ + vanjee_lidar_point_cloud_packet_header.timestamp_info_.nanosecond_ * 1e-9;
  if (!this->param_.use_lidar_clock) {
    pkt_ts = host_pkt_ts;
  } else {
    pkt_ts = lidar_pkt_ts < 0 ? 0 : lidar_pkt_ts;
  }

  if (!this->param_.point_cloud_enable)
    return false;

  uint32_t loss_circles_num = (vanjee_lidar_point_cloud_packet_header.circle_id_ + 0x10000 - pre_circle_id_) % 0x10000;
  pre_circle_id_ = vanjee_lidar_point_cloud_packet_header.circle_id_;
  if (loss_circles_num > 1) {
    this->point_cloud_->points.clear();
#ifdef FILTER_DLL_ENABLE
    Laser760Filter::clear_one_point_info_manage(pre_point_info_manager);
    Laser760Filter::clear_one_point_info_manage(cur_point_info_manager);
    export_point_size_cn_.echo1_data_size = 0;
    export_point_size_cn_.echo2_data_size = 0;
#endif
  }

#ifdef FILTER_DLL_ENABLE
  if ((loss_circles_num == 1 && (export_point_size_cn_.echo1_data_size + export_point_size_cn_.echo2_data_size) != 0)) {
    this->last_point_ts_ = this->first_point_ts_ + all_points_luminous_moment_760_140degree_[point_num - 1];

    pointCloudAlgorithmCallbackV1_132();
    this->cb_split_frame_(vanjee_lidar_point_cloud_packet_header.row_channel_num_ + update_row_num_, this->cloudTs());
    update_row_num_ = 0;
    ret = true;
  }
#endif

  uint16_t data_block_num_in_packet = vanjee_lidar_point_cloud_packet_header.data_block_info_.data_block_num_in_packet_;

  for (uint16_t i = 0; i < data_block_num_in_packet; i++) {
    if (i >= vanjee_lidar_point_cloud_packet_header.data_block_info_.valid_data_block_num_in_packet_)
      break;

    int32_t col_index =
        ((vanjee_lidar_point_cloud_packet_header.pkt_id_ - 1) * data_block_num_in_packet + i) / vanjee_lidar_point_cloud_packet_header.echo_num_;
    uint32_t point_offset_num = 0;
    if (!this->param_.use_lidar_clock) {
      point_offset_num = (col_index + 1) * vanjee_lidar_point_cloud_packet_header.row_channel_num_;
    } else {
      point_offset_num = col_index * vanjee_lidar_point_cloud_packet_header.row_channel_num_;
    }
    this->first_point_ts_ = pkt_ts - all_points_luminous_moment_760_140degree_[point_offset_num];

    uint16_t hor_resolution =
        (uint16_t)(vanjee_lidar_point_cloud_packet_header.reserved_field_1_[0] + (vanjee_lidar_point_cloud_packet_header.reserved_field_1_[1] << 8));
    uint8_t mirror_id = vanjee_lidar_point_cloud_packet_header.reserved_field_1_[4];
    uint16_t rpm =
        (uint16_t)(vanjee_lidar_point_cloud_packet_header.reserved_field_1_[5] + (vanjee_lidar_point_cloud_packet_header.reserved_field_1_[6] << 8));

    if (hor_resolution_ != hor_resolution / 10)
      hor_resolution_ = hor_resolution / 10;

    uint8_t cur_echo = 1;
    if (vanjee_lidar_point_cloud_packet_header.echo_num_ == 2) {
      cur_echo =
          ((vanjee_lidar_point_cloud_packet_header.pkt_id_ - 1) * vanjee_lidar_point_cloud_packet_header.data_block_info_.data_block_num_in_packet_) %
              2 +
          1;
    }
    uint32_t cur_blk_first_point_id = col_index * vanjee_lidar_point_cloud_packet_header.data_block_info_.row_channel_num_in_data_block_;

    if (vanjee_lidar_point_cloud_packet_header.data_block_info_.data_block_packet_type_ == 1) {
      for (int chan_id = 0; chan_id < vanjee_lidar_point_cloud_packet_header.data_block_info_.row_channel_num_in_data_block_; chan_id++) {
        int32_t row_index = chan_id;
        uint32_t point_id = cur_blk_first_point_id + row_index;
        double timestamp_point = 0.0;
        if (this->param_.ts_first_point)
          timestamp_point = all_points_luminous_moment_760_140degree_[point_id];
        else
          timestamp_point = all_points_luminous_moment_760_140degree_[point_id] - all_points_luminous_moment_760_140degree_[point_num - 1];

        int32_t rpm_offset_hor_angle = getTimeSequenceOffsetHorAngle(rpm / 100.0, chan_id + 1);
        int32_t hor_angle_y =
            ((hor_resolution * col_index + vanjee_lidar_point_cloud_packet_header.block_.azimuth_offset + rpm_offset_hor_angle) + 30000 + 270000) %
            360000;
        int32_t ver_angle = (ver_angle_type1_[row_index] + vanjee_lidar_point_cloud_packet_header.block_.elevation_offset + 360000) % 360000;
#ifdef FILTER_DLL_ENABLE
        size_t index = Laser760Filter::create_one_point_info_index(chan_id, col_index);
        bool old;
        Laser760Filter::ExportOnePointInfo* res = Laser760Filter::create_one_point_info(
            cur_point_info_manager, index, cur_echo, chan_id + this->first_line_id_, hor_resolution_ * col_index, old);
        res->index = index;
        res->line_id = chan_id + this->first_line_id_;
        res->col_index = col_index;
        res->mirror = mirror_id;
        res->hor_angle = (float)hor_angle_y * 1e-3;
        res->ver_angle = (float)ver_angle * 1e-3;

        res->distance = vanjee_lidar_point_cloud_packet_header.block_.channel[chan_id].distance * this->const_param_.DISTANCE_RES;
        res->distance_mm = (vanjee_lidar_point_cloud_packet_header.block_.channel[chan_id].distance * 4);
        res->cos_ride_sin = COS(ver_angle) * SIN(hor_angle_y);
        res->cos_rid_cos = COS(ver_angle) * COS(hor_angle_y);
        res->sin_rid_thousandth = SIN(ver_angle);
        res->x = res->distance * res->cos_ride_sin;
        res->y = res->distance * res->cos_rid_cos;
        res->z = res->distance * res->sin_rid_thousandth;

        res->reflectivity = vanjee_lidar_point_cloud_packet_header.block_.channel[chan_id].reflectivity;
        res->low_pulse_width = vanjee_lidar_point_cloud_packet_header.block_.channel[chan_id].tag;

        res->time = (uint64_t)((int64_t)(timestamp_point * 1e9));
        res->echo_type = cur_echo;
        res->hor_azimuth = hor_resolution_ * col_index;
        res->motor_speed = rpm;
        res->ground_flag = (vanjee_lidar_point_cloud_packet_header.block_.channel[chan_id].tag >> 7) & 0x01;
        res->confidence = 0;

        res->non_filterable = 0;

        // res->tall_pulse_width = 0;
        // res->strong_weak_flag = 0;

        if (cur_echo == 1) {
          export_point_size_cn_.echo1_data_size++;
        } else {
          export_point_size_cn_.echo2_data_size++;
        }
#endif
      }
    }
  }

  if (vanjee_lidar_point_cloud_packet_header.pkt_id_ == vanjee_lidar_point_cloud_packet_header.total_pkt_num_) {
    this->last_point_ts_ = this->first_point_ts_ + all_points_luminous_moment_760_140degree_[point_num - 1];

    pointCloudAlgorithmCallbackV1_132();
    this->cb_split_frame_(vanjee_lidar_point_cloud_packet_header.row_channel_num_ + update_row_num_, this->cloudTs());
    update_row_num_ = 0;
    ret = true;
  }
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee760<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);
  std::shared_ptr<ProtocolAbstract> p;
  if (*sp_cmd == *(CmdRepository760::CreateInstance()->sp_heart_beat_)) {
    p = std::make_shared<Protocol_HeartBeat760>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Param_HeartBeat760)) {
    std::shared_ptr<Param_HeartBeat760> param = std::dynamic_pointer_cast<Param_HeartBeat760>(params);
    if (param->heartbeat_flag_) {
      WJ_INFOL << "Handshake with wlr760 succ" << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    } else {
      WJ_WARNING << "Handshake with wlr760 fail" << WJ_REND;
      if ((*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_)).count(0x0001) >= 1) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[0x0001].setStopFlag(true);
      }
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(false);
    }
  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}
}  // namespace lidar

}  // namespace vanjee
