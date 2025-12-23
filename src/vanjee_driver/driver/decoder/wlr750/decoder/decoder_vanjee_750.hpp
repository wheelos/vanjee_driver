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
#include <vanjee_driver/driver/decoder/wlr750/protocol/frames/cmd_repository_750.hpp>
#include <vanjee_driver/driver/decoder/wlr750/protocol/frames/protocol_imu_packet_get.hpp>
#include <vanjee_driver/driver/decoder/wlr750/protocol/frames/protocol_protocol_version_set.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee750ABlockXYZ {
  int16_t x;
  int16_t y;
  int16_t z;
  uint32_t front_edg_value1;
  // uint32_t pulse_width1;
  uint16_t pulse_width1;
  uint8_t echo1_flag;
  uint8_t remain1;
  uint16_t peak_value1;
  uint8_t echo_num1;
  uint32_t front_edg_value2;
  uint32_t pulse_width2;
  uint16_t peak_value2;
  uint8_t echo_num2;

} Vanjee750ABlockXYZ;

typedef struct _Vanjee750ADifopPkt {
  uint8_t device_id[16];
  uint16_t hor_start_angle;
  uint16_t hor_end_angle;
  uint16_t ver_angle;
  uint16_t frame_num;
  uint16_t frame_id;
  uint32_t second;
  uint32_t microsecond;
  uint8_t time_sync_state;
  uint8_t time_sync_source;
  uint8_t time_sync_error;
  uint8_t protocol_ver;
} Vanjee750ADifopPkt;

typedef struct _Vanjee750AMsopPktXYZ {
  uint16_t header;
  Vanjee750ABlockXYZ blocks[48];
  Vanjee750ADifopPkt difop;

  void ToLittleEndian() {
    difop.hor_start_angle = ntohs(difop.hor_start_angle);
    difop.hor_end_angle = ntohs(difop.hor_end_angle);
    difop.ver_angle = ntohs(difop.ver_angle);
    difop.frame_num = ntohs(difop.frame_num);
    difop.frame_id = ntohs(difop.frame_id);
    difop.second = ntohl(difop.second);
    difop.microsecond = ntohl(difop.microsecond);
  }
} Vanjee750AMsopPktXYZ;

typedef struct _Vanjee750ABlockDistance {
  uint16_t distance;
  uint8_t hor_id;
  uint8_t ver_id;
  uint32_t front_edg_value1;
  // uint32_t pulse_width1;
  uint16_t pulse_width1;
  uint8_t echo1_flag;
  uint8_t remain1;
  uint16_t peak_value1;
  uint8_t echo_num1;
  uint32_t front_edg_value2;
  uint32_t pulse_width2;
  uint16_t peak_value2;
  uint8_t echo_num2;

} Vanjee750ABlockDistance;

typedef struct _Vanjee750AMsopPktDistance {
  uint16_t header;
  Vanjee750ABlockDistance blocks[48];
  Vanjee750ADifopPkt difop;

  void ToLittleEndian() {
    difop.hor_start_angle = ntohs(difop.hor_start_angle);
    difop.hor_end_angle = ntohs(difop.hor_end_angle);
    difop.ver_angle = ntohs(difop.ver_angle);
    difop.frame_num = ntohs(difop.frame_num);
    difop.frame_id = ntohs(difop.frame_id);
    difop.second = ntohl(difop.second);
    difop.microsecond = ntohl(difop.microsecond);
  }
} Vanjee750AMsopPktDistance;

#pragma pack(pop)

#pragma pack(push, 1)

typedef struct _Vanjee750MsopPktChannel {
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t Confidence;
} Vanjee750MsopPktChannel;

typedef struct _Vanjee750MsopPktTimeStamp {
  uint8_t clock_source;
  uint32_t second;
  uint32_t microsecond;
} Vanjee750MsopPktTimeStamp;

typedef struct _Vanjee750MsopPktBlock {
  uint8_t start_flag;
  uint8_t remain3[4];
  Vanjee750MsopPktChannel channel[150];
} Vanjee750MsopPktBlock;

typedef struct _Vanjee750MsopPktIMU {
  int32_t microsecond;
  int32_t x_acceleration;
  int32_t y_acceleration;
  int32_t z_acceleration;
  int32_t x_palstance;
  int32_t y_palstance;
  int32_t z_palstance;

} Vanjee750MsopPktIMU;

typedef struct _Vanjee750MsopPktFuncSafe {
  uint8_t lidar_status;
  uint16_t lidar_fault_codes;
  uint8_t fault_code_counter;
} Vanjee750MsopPktFuncSafe;

typedef struct _Vanjee750MsopPkt {
  uint8_t head[2];
  uint16_t frame_len;
  uint8_t device_type[2];
  uint8_t protocol_main_cmd;
  uint8_t protocol_sub_cmd;
  uint16_t circle_id;
  uint16_t frame_num;
  uint16_t frame_id;
  uint16_t operate_frequency;
  uint8_t return_wave_num;
  uint16_t col_channel_num;
  uint16_t row_channel_num;
  // uint8_t remain1[8];
  uint16_t hor_resolution;
  uint16_t ver_resolution;
  uint8_t remain1[4];

  uint8_t block_packaging_method;
  uint16_t block_num;
  uint16_t effective_block_num;
  uint16_t block_row_num;
  uint16_t block_col_num;
  uint8_t distance_resolution;

  Vanjee750MsopPktTimeStamp timestamp;

  uint8_t remain2[16];

  uint16_t data_flag;

  Vanjee750MsopPktBlock blocks;

  Vanjee750MsopPktIMU imu;

  Vanjee750MsopPktFuncSafe func_safe;

  uint8_t digital_signature[32];
  uint8_t check[2];
  uint8_t tail[2];

} Vanjee750MsopPkt;

#pragma pack(pop)

#pragma pack(push, 1)

typedef struct _Vanjee750DataUnit {
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t Confidence;
} Vanjee750DataUnit;

typedef struct _Vanjee750DataBlock {
  uint8_t start_flag;
  uint8_t remain3[4];
  Vanjee750DataUnit channel[150];
} Vanjee750DataBlock;

typedef struct _Vanjee750CDataUnit {
  uint16_t distance;
  uint8_t reflectivity;
  uint16_t Confidence;
} Vanjee750CDataUnit;

typedef struct _Vanjee750CDataBlock {
  uint8_t start_flag;
  uint8_t remain3[4];
  Vanjee750CDataUnit channel[256];
} Vanjee750CDataBlock;

#pragma pack(pop)
template <typename T_PointCloud>
class DecoderVanjee750 : public DecoderMech<T_PointCloud> {
 private:
  std::vector<double> all_points_luminous_moment_750_;

  uint32_t frame_id_trans_pre_ = 0;
  uint16_t pre_operate_frequency_ = 0;
  uint8_t pre_data_format_ = 0;
  uint32_t pre_circle_id_ = 0;
  int32_t pre_frame_id_ = -1;
  ChanAngles chan_angles_;
  uint8_t publish_mode_ = 0;
  bool publish_flag_ = false;
  bool first_pkg_flag_ = true;

  double pre_imu_timestamp_ = 0.0;

  std::vector<::vector<double>> lidar_hor_angle_;
  std::vector<::vector<double>> lidar_ver_angle_;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);

  int loadFromFile(uint16_t row, uint16_t col, const std::string &angle_path, std::vector<std::vector<double>> &angles_value);

  void configParamsInit(uint16_t row, uint16_t col);
  void initLdLuminousMoment(uint16_t operate_frequency, uint16_t row, uint16_t col, uint8_t data_format = 2);

  DecoderPacketBase<T_PointCloud, Vanjee750DataBlock, Vanjee750DataUnit> *decoder_packet_base_ = nullptr;
  DecoderPacketBase<T_PointCloud, Vanjee750CDataBlock, Vanjee750CDataUnit> *decoder_packet_base_c_ = nullptr;
  void update750BAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info, Vanjee750DataBlock *data_block);
  void update750CAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info, Vanjee750CDataBlock *data_block);
  void update750EAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info, Vanjee750DataBlock *data_block);
  void decoderDataUnitCallback(Vanjee750DataUnit *data_unit, PointInfo &point_info);
  void decoderDataUnitCCallback(Vanjee750CDataUnit *data_unit, PointInfo &point_info);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 900;
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee750() {  // = default;
    delete decoder_packet_base_;
    delete decoder_packet_base_c_;
  }
  explicit DecoderVanjee750(const WJDecoderParam &param);

  bool decodeMsopPkt750AXYZ(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt750ADistance(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt750BBaseProtocol(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt750CBaseProtocol(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt750EBaseProtocol(const uint8_t *pkt, size_t size);
};

template <typename T_PointCloud>
void DecoderVanjee750<T_PointCloud>::initLdLuminousMoment(uint16_t operate_frequency, uint16_t row, uint16_t col, uint8_t data_format) {
  if (operate_frequency != pre_operate_frequency_ || data_format != pre_data_format_) {
    size_t point_cloud_size = row * col;
    all_points_luminous_moment_750_.resize(point_cloud_size);

    if (decoder_packet_base_->decoder_packet_general_version_base_ptr_ != nullptr) {
      double offset = 1.0 / operate_frequency / (col / 4);

      for (int i = 0; i < col; i++) {
        for (int j = 0; j < row; j++) {
          all_points_luminous_moment_750_[i * row + j] = (i / 4) * offset;
        }
      }
      decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_ = all_points_luminous_moment_750_;
    }

    if (decoder_packet_base_c_->decoder_packet_general_version_base_ptr_ != nullptr) {
      if (data_format == 1) {
        // row 1-192
        uint32_t group_num = 16;  // 256 / 16
        uint32_t block_num_of_group = 32;
        double block_offset_ts = 3.125e-3;
        double peer_group_offset_ts = 1.0 / operate_frequency / group_num;
        double peer_block_offset_ts = block_offset_ts / block_num_of_group;

        for (int col_index = 0; col_index < col; col_index++) {
          for (int row_index = 0; row_index < row; row_index++) {
            all_points_luminous_moment_750_[col_index * row + row_index] =
                ((col_index / 16) * peer_group_offset_ts) + ((row_index / block_num_of_group) * peer_block_offset_ts);
          }
        }
      } else if (data_format == 2) {
        // col 1-256
        uint32_t group_num = 32;  // 192/6;
        uint32_t block_num_of_group = 16;
        double block_offset_ts = 3.125e-3;
        double peer_group_offset_ts = 1.0 / operate_frequency / group_num;
        double peer_block_offset_ts = block_offset_ts / block_num_of_group;

        for (int row_index = 0; row_index < row; row_index++) {
          for (int col_index = 0; col_index < col; col_index++) {
            all_points_luminous_moment_750_[row_index * col + col_index] =
                ((row_index / 6) * peer_group_offset_ts) + ((col_index / block_num_of_group) * peer_block_offset_ts);
          }
        }
      }
      decoder_packet_base_c_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_ = all_points_luminous_moment_750_;
    }
    pre_operate_frequency_ = operate_frequency;
    pre_data_format_ = data_format;
  }
}

template <typename T_PointCloud>
void DecoderVanjee750<T_PointCloud>::configParamsInit(uint16_t row, uint16_t col) {
  if (first_pkg_flag_ && this->param_.config_from_file) {
    loadFromFile(row, col, this->param_.angle_path_hor, lidar_hor_angle_);
    loadFromFile(row, col, this->param_.angle_path_ver, lidar_ver_angle_);

    first_pkg_flag_ = false;
  }
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee750<T_PointCloud>::getConstParam(uint8_t mode) {
  // WJDecoderConstParam
  // WJDecoderMechConstParam
  static WJDecoderMechConstParam param = {
      1382  // msop len
      ,
      360  // laser number
      ,
      30  // blocks per packet
      ,
      360  // channels per block
      ,
      0.01f  // distance min
      ,
      5.0f  // distance max
      ,
      0.002f  // distance resolution
      ,
      80.0f  // initial value of temperature
  };
  param.BLOCK_DURATION = 0.1 / 360;
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee750<T_PointCloud>::DecoderVanjee750(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  if (param.imu_enable != -1) {
    this->point_cloud_ready_ = false;
    WJ_INFO << "Waiting for IMU calibration..." << WJ_REND;
  } else {
    this->point_cloud_ready_ = true;
  }

  this->m_imu_params_get_ = std::make_shared<ImuParamGet>(90, param.transform_param);

  publish_mode_ = param.publish_mode;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);

  this->start_angle_ = this->param_.start_angle * 1000;
  this->end_angle_ = this->param_.end_angle * 1000;

  decoder_packet_base_ = new DecoderPacketBase<T_PointCloud, Vanjee750DataBlock, Vanjee750DataUnit>(this);
  decoder_packet_base_c_ = new DecoderPacketBase<T_PointCloud, Vanjee750CDataBlock, Vanjee750CDataUnit>(this);
}

template <typename T_PointCloud>
int DecoderVanjee750<T_PointCloud>::loadFromFile(uint16_t row, uint16_t col, const std::string &angle_path,
                                                 std::vector<std::vector<double>> &angles_value) {
  angles_value.clear();
  angles_value.resize(row);
  for (int i = 0; i < row; i++) {
    angles_value[i].resize(col);
  }

  std::ifstream fd(angle_path.c_str(), std::ios::in);
  if (!fd.is_open()) {
    WJ_WARNING << "fail to open vangle file:" << angle_path << WJ_REND;
    return -1;
  }

  try {
    std::string line;
    int rowid = 0;
    while (std::getline(fd, line)) {
      std::stringstream sinFile(line);
      std::vector<std::string> LineData = this->chan_angles_.vStrSplit(sinFile.str(), ',');

      for (int i = 0; i < col; i++) {
        angles_value[rowid][i] = std::stod(LineData[i]);
      }
      rowid++;
    }
  } catch (...) {
    WJ_ERROR << "The format of angle config file " << angle_path << " is wrong. Please check (e.g. indentation)." << WJ_REND;
  }
  fd.close();
  return 0;
}

template <typename T_PointCloud>
inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  uint16_t l_pktheader = pkt[0] << 8 | pkt[1];
  uint16_t version_id = pkt[6] << 8 | pkt[7];
  switch (l_pktheader) {
    case 0xFFCC: {
      if (size == 1384) {
        ret = decodeMsopPkt750AXYZ(pkt, size);
      } else if (size == 739 && version_id == 0x0100) {
        configParamsInit(150, 360);
        ret = decodeMsopPkt750BBaseProtocol(pkt, size);
      } else if ((size == 1419 || size == 1099) && version_id == 0x0101) {
        configParamsInit(212, 276);
        ret = decodeMsopPkt750CBaseProtocol(pkt, size);
      } else if (size == 739) {
        configParamsInit(190, 340);
        ret = decodeMsopPkt750EBaseProtocol(pkt, size);
      }

    } break;
    case 0xFFDD: {
      if (size == 1288) {
        configParamsInit(112, 192);
        initLdLuminousMoment(10, 112, 192);
        ret = decodeMsopPkt750ADistance(pkt, size);
      }
    } break;
    default:
      break;
  }
  return ret;
}

template <typename T_PointCloud>
inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt750AXYZ(const uint8_t *packet, size_t size) {
  Vanjee750AMsopPktXYZ &pkt = *(Vanjee750AMsopPktXYZ *)packet;
  pkt.ToLittleEndian();
  bool ret = false;
  double pkt_ts = 0;

  int32_t loss_packets_num = (pkt.difop.frame_id + 448 - pre_frame_id_) % 448;
  if (loss_packets_num > 20 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
  pre_frame_id_ = pkt.difop.frame_id;

  if (!this->param_.use_lidar_clock) {
    pkt_ts = getTimeHost() * 1e-6;
  } else {
    double sec = pkt.difop.second;
    double usec = pkt.difop.microsecond;
    pkt_ts = sec + usec * 1e-6;
    pkt_ts = pkt_ts < 0 ? 0 : pkt_ts;
  }

  if (split_strategy_->newBlock(pkt.difop.frame_id)) {
    if (publish_flag_) {
      this->cb_split_frame_(112, this->cloudTs());
      this->first_point_ts_ = pkt_ts;
      ret = true;
      publish_flag_ = false;
    } else {
      publish_flag_ = true;
    }
  }

  for (uint16_t blk = 0; blk < 48; blk++) {
    if (pkt.blocks[blk].front_edg_value1 != 0) {
      float x = pkt.blocks[blk].x / 500.0;
      float y = pkt.blocks[blk].y / 500.0;
      float z = pkt.blocks[blk].z / 500.0;

      typename T_PointCloud::PointT point;
      if (x == 0 && y == 0 && z == 0) {
        if (!this->param_.dense_points) {
          setX(point, NAN);
          setY(point, NAN);
          setZ(point, NAN);
        } else {
          this->transformPoint(x, y, z);
          setX(point, x);
          setY(point, y);
          setZ(point, z);
        }
        setIntensity(point, 0);
      } else {
        this->transformPoint(x, y, z);
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, pkt.blocks[blk].peak_value1);
      }
      setTimestamp(point, pkt_ts);
      setRing(point, pkt.difop.ver_angle);
      setTag(point, 0);

      this->point_cloud_->points.emplace_back(point);
    }
    this->last_point_ts_ = pkt_ts;
  }
  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt750ADistance(const uint8_t *packet, size_t size) {
  Vanjee750AMsopPktDistance &pkt = *(Vanjee750AMsopPktDistance *)packet;
  pkt.ToLittleEndian();
  bool ret = false;
  double pkt_ts = 0;

  int32_t loss_packets_num = (pkt.difop.frame_id + 448 - pre_frame_id_) % 448;
  if (loss_packets_num > 20 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
  pre_frame_id_ = pkt.difop.frame_id;

  if (!this->param_.use_lidar_clock) {
    pkt_ts = getTimeHost() * 1e-6;
  } else {
    double sec = pkt.difop.second;
    double usec = pkt.difop.microsecond;
    pkt_ts = sec + usec * 1e-6;
    pkt_ts = pkt_ts < 0 ? 0 : pkt_ts;
  }

  if (split_strategy_->newBlock(pkt.difop.frame_id)) {
    if (publish_flag_) {
      this->cb_split_frame_(112, this->cloudTs());
      this->first_point_ts_ = pkt_ts;
      ret = true;
      publish_flag_ = false;
    } else {
      publish_flag_ = true;
    }
  }

  if (lidar_hor_angle_.size() < 112 || lidar_ver_angle_.size() < 112) {
    WJ_ERROR << "Please configure parameter table path!Absolute Path (<PROJECT_PATH>/src/vanjee_lidar_sdk/param)" << WJ_REND;
    return ret;
  }
  double timestamp_point = 0.0;
  for (uint16_t blk = 0; blk < 48; blk++) {
    int32_t col_index = pkt.difop.hor_start_angle + blk;
    int32_t row_index = pkt.difop.ver_angle;

    int32_t angle_horiz = int32_t(lidar_hor_angle_[row_index][col_index] * 1000 + 360000) % 360000;
    int32_t angle_vert = int32_t(lidar_ver_angle_[row_index][col_index] * 1000 + 360000) % 360000;
    float distance = pkt.blocks[blk].distance / 500.0;

    uint32_t point_id = col_index * 112 + row_index;
    if (this->param_.ts_first_point == true) {
      timestamp_point = all_points_luminous_moment_750_[point_id];
    } else {
      timestamp_point = all_points_luminous_moment_750_[point_id] - all_points_luminous_moment_750_[all_points_luminous_moment_750_.size() - 1];
    }

    if (this->start_angle_ < this->end_angle_) {
      if (angle_horiz < this->start_angle_ || angle_horiz > this->end_angle_) {
        distance = 0;
      }
    } else {
      if (angle_horiz > this->end_angle_ && angle_horiz < this->start_angle_) {
        distance = 0;
      }
    }

    if (this->hide_range_params_.size() > 0 && distance != 0 &&
        this->isValueInRange(row_index + this->first_line_id_, angle_horiz / 1000.0, distance, this->hide_range_params_)) {
      distance = 0;
    }

    float dxy = distance * COS(angle_vert);
    float x = dxy * COS(angle_horiz);
    float y = dxy * SIN(angle_horiz);
    float z = distance * SIN(angle_vert);

    if (pkt.blocks[blk].front_edg_value1 != 0) {
      typename T_PointCloud::PointT point;
      if (!this->distance_section_.in(distance)) {
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
      } else {
        this->transformPoint(x, y, z);
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, pkt.blocks[blk].peak_value1);
      }
      setTimestamp(point, timestamp_point);
      setRing(point, row_index);
      setTag(point, 0);
#ifdef ENABLE_GTEST
      setPointId(point, point_id);
      setHorAngle(point, angle_horiz / 1000.0);
      setVerAngle(point, angle_vert / 1000.0);
      setDistance(point, distance);
#endif
      this->point_cloud_->points.emplace_back(point);
    }
    this->last_point_ts_ = pkt_ts;
  }
  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee750<T_PointCloud>::update750BAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info,
                                                                             Vanjee750DataBlock *data_block) {
  initLdLuminousMoment((uint16_t)(data_block_angle_and_timestamp_info.operate_frequency_ * 0.01),
                       data_block_angle_and_timestamp_info.row_channel_num_, data_block_angle_and_timestamp_info.col_channel_num_);

  if (data_block_angle_and_timestamp_info.data_block_index_in_packet_ ==
      data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_ - 1) {
    if (!this->param_.use_lidar_clock) {
      uint16_t group_id = (data_block_angle_and_timestamp_info.packet_id_ + 3) / 4;
      data_block_angle_and_timestamp_info.first_point_ts_ -= group_id * (1 / (data_block_angle_and_timestamp_info.operate_frequency_ * 0.01) /
                                                                         (data_block_angle_and_timestamp_info.col_channel_num_ / 4));
    } else {
      uint16_t group_id = (data_block_angle_and_timestamp_info.packet_id_ - 1) / 4;
      if (group_id > 0) {
        data_block_angle_and_timestamp_info.first_point_ts_ -= group_id * (1 / (data_block_angle_and_timestamp_info.operate_frequency_ * 0.01) /
                                                                           (data_block_angle_and_timestamp_info.col_channel_num_ / 4));
      }
    }
  }

  if (data_block_angle_and_timestamp_info.data_block_info_.data_block_packet_type_ == 1) {
    for (int chan_id = 0; chan_id < data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_; chan_id++) {
      int32_t col_index =
          ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
           data_block_angle_and_timestamp_info.data_block_index_in_packet_) /
          data_block_angle_and_timestamp_info.echo_num_;
      int32_t row_index = chan_id;
      uint32_t point_id = col_index * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ + row_index;
      double timestamp_point = 0.0;
      if (this->param_.ts_first_point)
        timestamp_point = decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
      else
        timestamp_point =
            decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id] -
            decoder_packet_base_->decoder_packet_general_version_base_ptr_
                ->all_points_luminous_moment_[decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_.size() - 1];

      int32_t hor_angle = (int32_t)(lidar_hor_angle_[row_index][col_index] * 1000 + 270000) % 360000;
      PointInfo point_info;
      // point_info.distance_ = 0;
      point_info.azimuth_ = hor_angle * 1e-3;
      point_info.elevation_ = lidar_ver_angle_[row_index][col_index];

      // point_info.reflectivity_ = 0;
      point_info.ring_ = chan_id + this->first_line_id_;
      point_info.timestamp_ = timestamp_point;
      point_info.tag_ = 0;
      point_info.id_ = point_id;
      data_block_angle_and_timestamp_info.point_info_vector_.emplace_back(point_info);
    }
  }
}

template <typename T_PointCloud>
void DecoderVanjee750<T_PointCloud>::update750CAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info,
                                                                             Vanjee750CDataBlock *data_block) {
  initLdLuminousMoment((uint16_t)(data_block_angle_and_timestamp_info.operate_frequency_ * 0.01),
                       data_block_angle_and_timestamp_info.row_channel_num_, data_block_angle_and_timestamp_info.col_channel_num_,
                       data_block_angle_and_timestamp_info.data_block_info_.data_block_packet_type_);

  if (data_block_angle_and_timestamp_info.data_block_index_in_packet_ ==
      data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_ - 1) {
    if (!this->param_.use_lidar_clock) {
      data_block_angle_and_timestamp_info.first_point_ts_ -=
          (data_block_angle_and_timestamp_info.packet_id_ * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ *
           (1 / (data_block_angle_and_timestamp_info.operate_frequency_ * 0.01)));
    } else {
      data_block_angle_and_timestamp_info.first_point_ts_ -=
          ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ *
           (1 / (data_block_angle_and_timestamp_info.operate_frequency_ * 0.01)));
    }
  }

  uint8_t scan_direction = data_block_angle_and_timestamp_info.reserved_field_1_[0];
  int16_t row_offset =
      (int16_t)(data_block_angle_and_timestamp_info.reserved_field_2_[0] + (data_block_angle_and_timestamp_info.reserved_field_2_[1] << 8));
  int16_t col_offset =
      (int16_t)(data_block_angle_and_timestamp_info.reserved_field_2_[2] + (data_block_angle_and_timestamp_info.reserved_field_2_[3] << 8));
  if (row_offset < -10 || row_offset > 10 || col_offset < -10 || col_offset > 10) {
    WJ_WARNING << "angle table offset err" << WJ_REND;
    return;
  }

  if (data_block_angle_and_timestamp_info.data_block_info_.data_block_packet_type_ == 2) {
    for (int col_id = 0; col_id < data_block_angle_and_timestamp_info.data_block_info_.col_channel_num_in_data_block_; col_id++) {
      int32_t row_index =
          ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
           data_block_angle_and_timestamp_info.data_block_index_in_packet_) /
          data_block_angle_and_timestamp_info.echo_num_;
      int32_t col_index = col_id;
      uint32_t point_id = row_index * data_block_angle_and_timestamp_info.data_block_info_.col_channel_num_in_data_block_ + col_index;
      double timestamp_point = 0.0;
      if (this->param_.ts_first_point) {
        timestamp_point = decoder_packet_base_c_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
      } else {
        timestamp_point =
            decoder_packet_base_c_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id] -
            decoder_packet_base_c_->decoder_packet_general_version_base_ptr_
                ->all_points_luminous_moment_[decoder_packet_base_c_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_.size() -
                                              1];
      }
      PointInfo point_info;
      double hor_angle = 0.0;
      double ver_angle = 0.0;
      if (scan_direction == 1) {
        hor_angle = lidar_hor_angle_[row_index + row_offset + 10][col_index + col_offset + 10];
        ver_angle = lidar_ver_angle_[row_index + row_offset + 10][col_index + col_offset + 10];
      } else if (scan_direction == 2) {
        hor_angle =
            lidar_hor_angle_[row_index + row_offset + 10][lidar_hor_angle_[row_index + row_offset + 10].size() - 1 - (col_index - col_offset + 10)];
        ver_angle =
            lidar_ver_angle_[row_index + row_offset + 10][lidar_ver_angle_[row_index + row_offset + 10].size() - 1 - (col_index - col_offset + 10)];
      } else if (scan_direction == 3) {
        hor_angle = lidar_hor_angle_[lidar_hor_angle_.size() - 1 - (row_index - row_offset + 10)][col_index + col_offset + 10];
        ver_angle = lidar_ver_angle_[lidar_ver_angle_.size() - 1 - (row_index - row_offset + 10)][col_index + col_offset + 10];
      } else if (scan_direction == 4) {
        hor_angle = lidar_hor_angle_[lidar_hor_angle_.size() - 1 - (row_index - row_offset + 10)]
                                    [lidar_hor_angle_[row_index + row_offset + 10].size() - 1 - (col_index - col_offset + 10)];
        ver_angle = lidar_ver_angle_[lidar_ver_angle_.size() - 1 - (row_index - row_offset + 10)]
                                    [lidar_ver_angle_[row_index + row_offset + 10].size() - 1 - (col_index - col_offset + 10)];
      }

      point_info.azimuth_ = hor_angle;
      point_info.elevation_ = ver_angle;
      // point_info.distance_ = 0;
      // point_info.reflectivity_ = 0;
      if (scan_direction == 1 || scan_direction == 2) {
        point_info.ring_ = (data_block_angle_and_timestamp_info.row_channel_num_ - (row_index + 1)) + this->first_line_id_;
      } else {
        point_info.ring_ = row_index + this->first_line_id_;
      }
      point_info.timestamp_ = timestamp_point;
      point_info.tag_ = 0;
      point_info.id_ = point_id;
      data_block_angle_and_timestamp_info.point_info_vector_.emplace_back(point_info);
    }
  } else if (data_block_angle_and_timestamp_info.data_block_info_.data_block_packet_type_ == 1) {
    for (int row_id = 0; row_id < data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_; row_id++) {
      int32_t col_index =
          ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
           data_block_angle_and_timestamp_info.data_block_index_in_packet_) /
          data_block_angle_and_timestamp_info.echo_num_;
      int32_t row_index = row_id;
      uint32_t point_id = col_index * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ + row_index;
      double timestamp_point = 0.0;
      if (this->param_.ts_first_point) {
        timestamp_point = decoder_packet_base_c_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
      } else {
        timestamp_point =
            decoder_packet_base_c_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id] -
            decoder_packet_base_c_->decoder_packet_general_version_base_ptr_
                ->all_points_luminous_moment_[decoder_packet_base_c_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_.size() -
                                              1];
      }
      PointInfo point_info;
      double hor_angle = 0.0;
      double ver_angle = 0.0;
      if (scan_direction == 5) {
        hor_angle = lidar_hor_angle_[row_index + row_offset + 10][col_index + col_offset + 10];
        ver_angle = lidar_ver_angle_[row_index + row_offset + 10][col_index + col_offset + 10];
      } else if (scan_direction == 6) {
        hor_angle = lidar_hor_angle_[lidar_hor_angle_.size() - 1 - (row_index - row_offset + 10)][col_index + col_offset + 10];
        ver_angle = lidar_ver_angle_[lidar_ver_angle_.size() - 1 - (row_index - row_offset + 10)][col_index + col_offset + 10];
      } else if (scan_direction == 7) {
        hor_angle =
            lidar_hor_angle_[row_index + row_offset + 10][lidar_hor_angle_[row_index + row_offset + 10].size() - 1 - (col_index - col_offset + 10)];
        ver_angle =
            lidar_ver_angle_[row_index + row_offset + 10][lidar_ver_angle_[row_index + row_offset + 10].size() - 1 - (col_index - col_offset + 10)];
      } else if (scan_direction == 8) {
        hor_angle = lidar_hor_angle_[lidar_hor_angle_.size() - 1 - (row_index - row_offset + 10)]
                                    [lidar_hor_angle_[row_index + row_offset + 10].size() - 1 - (col_index - col_offset + 10)];
        ver_angle = lidar_ver_angle_[lidar_ver_angle_.size() - 1 - (row_index - row_offset + 10)]
                                    [lidar_ver_angle_[row_index + row_offset + 10].size() - 1 - (col_index - col_offset + 10)];
      }

      point_info.azimuth_ = hor_angle;
      point_info.elevation_ = ver_angle;
      // point_info.distance_ = 0;
      // point_info.reflectivity_ = 0;
      if (scan_direction == 5 || scan_direction == 7) {
        point_info.ring_ = (data_block_angle_and_timestamp_info.row_channel_num_ - (row_index + 1)) + this->first_line_id_;
      } else {
        point_info.ring_ = row_index + this->first_line_id_;
      }
      point_info.timestamp_ = timestamp_point;
      point_info.tag_ = 0;
      point_info.id_ = point_id;
      data_block_angle_and_timestamp_info.point_info_vector_.emplace_back(point_info);
    }
  }
}

template <typename T_PointCloud>
void DecoderVanjee750<T_PointCloud>::update750EAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info,
                                                                             Vanjee750DataBlock *data_block) {
  initLdLuminousMoment((uint16_t)(data_block_angle_and_timestamp_info.operate_frequency_ * 0.01),
                       data_block_angle_and_timestamp_info.row_channel_num_, data_block_angle_and_timestamp_info.col_channel_num_);

  if (data_block_angle_and_timestamp_info.data_block_index_in_packet_ ==
      data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_ - 1) {
    if (!this->param_.use_lidar_clock) {
      uint16_t group_id = (data_block_angle_and_timestamp_info.packet_id_ + 3) / 4;
      data_block_angle_and_timestamp_info.first_point_ts_ -= group_id * (1 / (data_block_angle_and_timestamp_info.operate_frequency_ * 0.01) /
                                                                         (data_block_angle_and_timestamp_info.col_channel_num_ / 4));
    } else {
      uint16_t group_id = (data_block_angle_and_timestamp_info.packet_id_ - 1) / 4;
      if (group_id > 0) {
        data_block_angle_and_timestamp_info.first_point_ts_ -= group_id * (1 / (data_block_angle_and_timestamp_info.operate_frequency_ * 0.01) /
                                                                           (data_block_angle_and_timestamp_info.col_channel_num_ / 4));
      }
    }
  }

  if (data_block_angle_and_timestamp_info.data_block_info_.data_block_packet_type_ == 1) {
    uint8_t scan_direction = data_block_angle_and_timestamp_info.reserved_field_1_[0];
    int16_t row_offset =
        (int16_t)(data_block_angle_and_timestamp_info.reserved_field_2_[0] + (data_block_angle_and_timestamp_info.reserved_field_2_[1] << 8));
    int16_t col_offset =
        (int16_t)(data_block_angle_and_timestamp_info.reserved_field_2_[2] + (data_block_angle_and_timestamp_info.reserved_field_2_[3] << 8));
    if (row_offset < -20 || row_offset > 20 || col_offset < -20 || col_offset > 20) {
      WJ_WARNING << "angle table offset err" << WJ_REND;
      return;
    }

    for (int chan_id = 0; chan_id < data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_; chan_id++) {
      int32_t col_index =
          ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
           data_block_angle_and_timestamp_info.data_block_index_in_packet_) /
          data_block_angle_and_timestamp_info.echo_num_;
      int32_t row_index = chan_id;
      uint32_t point_id = col_index * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ + row_index;
      double timestamp_point = 0.0;
      if (this->param_.ts_first_point)
        timestamp_point = decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
      else
        timestamp_point =
            decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id] -
            decoder_packet_base_->decoder_packet_general_version_base_ptr_
                ->all_points_luminous_moment_[decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_.size() - 1];

      int32_t hor_angle = 0;
      if (scan_direction == 0) {
        hor_angle = (int32_t)(lidar_hor_angle_[row_index + row_offset + 20][col_index + col_offset + 20] * 1000 + 270000) % 360000;
      } else {
        hor_angle = (int32_t)(lidar_hor_angle_[row_index + row_offset + 20]
                                              [lidar_hor_angle_[row_index + row_offset + 20].size() - 1 - (col_index + col_offset + 20)] *
                                  1000 +
                              270000) %
                    360000;
      }
      PointInfo point_info;
      // point_info.distance_ = 0;
      point_info.azimuth_ = hor_angle * 1e-3;
      if (scan_direction == 0) {
        point_info.elevation_ = lidar_ver_angle_[row_index + row_offset + 20][col_index + col_offset + 20];
      } else {
        point_info.elevation_ =
            lidar_ver_angle_[row_index + row_offset + 20][lidar_ver_angle_[row_index + row_offset + 20].size() - 1 - (col_index + col_offset + 20)];
      }

      // point_info.reflectivity_ = 0;
      point_info.ring_ = chan_id + this->first_line_id_;
      point_info.timestamp_ = timestamp_point;
      point_info.tag_ = 0;
      point_info.id_ = point_id;
      data_block_angle_and_timestamp_info.point_info_vector_.emplace_back(point_info);
    }
  }
}

template <typename T_PointCloud>
void DecoderVanjee750<T_PointCloud>::decoderDataUnitCallback(Vanjee750DataUnit *data_unit, PointInfo &point_info) {
  point_info.distance_ = data_unit->distance;
  // point_info.azimuth_ = point_info.azimuth_;
  // point_info.elevation_ = point_info.elevation_;
  point_info.reflectivity_ = data_unit->reflectivity;
  // point_info.ring_ = point_info.ring_;
  // point_info.timestamp_ = point_info.timestamp_;
  // point_info.tag_ = point_info.tag_;
  // point_info.id_ = point_info.id_;
}

template <typename T_PointCloud>
void DecoderVanjee750<T_PointCloud>::decoderDataUnitCCallback(Vanjee750CDataUnit *data_unit, PointInfo &point_info) {
  point_info.distance_ = data_unit->distance;
  // point_info.azimuth_ = point_info.azimuth_;
  // point_info.elevation_ = point_info.elevation_;
  point_info.reflectivity_ = data_unit->reflectivity;
  // point_info.ring_ = point_info.ring_;
  // point_info.timestamp_ = point_info.timestamp_;
  // point_info.tag_ = point_info.tag_;
  // point_info.id_ = point_info.id_;
}

template <typename T_PointCloud>
inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt750BBaseProtocol(const uint8_t *pkt, size_t size) {
  return decoder_packet_base_->decoderPacket(
      pkt, size, std::bind(&DecoderVanjee750::update750BAngleAndTimestampInfoCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DecoderVanjee750::decoderDataUnitCallback, this, std::placeholders::_1, std::placeholders::_2));
}

template <typename T_PointCloud>
inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt750CBaseProtocol(const uint8_t *pkt, size_t size) {
  return decoder_packet_base_c_->decoderPacket(
      pkt, size, std::bind(&DecoderVanjee750::update750CAngleAndTimestampInfoCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DecoderVanjee750::decoderDataUnitCCallback, this, std::placeholders::_1, std::placeholders::_2));
}

template <typename T_PointCloud>
inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt750EBaseProtocol(const uint8_t *pkt, size_t size) {
  return decoder_packet_base_->decoderPacket(
      pkt, size, std::bind(&DecoderVanjee750::update750EAngleAndTimestampInfoCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DecoderVanjee750::decoderDataUnitCallback, this, std::placeholders::_1, std::placeholders::_2));
}

template <typename T_PointCloud>
void DecoderVanjee750<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract750> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository750::CreateInstance()->set_protocol_version_cmd_id_ptr_)) {
    p = std::make_shared<Protocol_ProtocolVersionSet>();
  } else if (*sp_cmd == *(CmdRepository750::CreateInstance()->sp_get_imu_packet_)) {
    p = std::make_shared<Protocol_ImuPacketGet750>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Param_ProtocolVersionSetWlr750)) {
  } else if (typeid(*params) == typeid(Params_ImuPacketGet750)) {
    std::shared_ptr<Params_ImuPacketGet750> param = std::dynamic_pointer_cast<Params_ImuPacketGet750>(params);
    if (this->param_.imu_enable == -1)
      return;
    double pkt_ts = 0.0;
    double imu_timestamp = param->imu_sec_ + (param->imu_nsec_ * 1e-9);
    if (this->param_.use_lidar_clock)
      pkt_ts = imu_timestamp;
    else
      pkt_ts = getTimeHost() * 1e-6;

    if (pre_imu_timestamp_ > 0 && imu_timestamp > pre_imu_timestamp_) {
      this->imu_ready_ = this->m_imu_params_get_->imuGet(param->imu_angle_voc_x_, param->imu_angle_voc_y_, param->imu_angle_voc_z_,
                                                         param->imu_linear_acce_x_, param->imu_linear_acce_y_, param->imu_linear_acce_z_,
                                                         imu_timestamp, this->param_.imu_orientation_enable, -100.0, false, false, false, true, true);
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

  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}
}  // namespace lidar

}  // namespace vanjee
