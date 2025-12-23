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
#include <vanjee_driver/driver/decoder/wlr722z/protocol/frames/cmd_repository_722z.hpp>
#include <vanjee_driver/driver/decoder/wlr722z/protocol/frames/protocol_acceleration_range_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722z/protocol/frames/protocol_acceleration_range_set.hpp>
#include <vanjee_driver/driver/decoder/wlr722z/protocol/frames/protocol_firmware_version_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722z/protocol/frames/protocol_ldvalue_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722z/protocol/frames/protocol_scan_data_state_set.hpp>
#include <vanjee_driver/driver/decoder/wlr722z/protocol/frames/protocol_sn_get.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee722zSerialBlockChannel {
  uint16_t distance;
  uint8_t reflectivity;
} Vanjee722zSerialBlockChannel;

typedef struct _Vanjee722zSerialPointCloud {
  uint16_t azimuth;
  Vanjee722zSerialBlockChannel channel[16];
  uint32_t dirty_degree;
  uint8_t lidar_state;
  uint8_t reserved_id;
  uint16_t reserved_info;
  uint16_t sequence_num;
} Vanjee722zSerialPointCloud;

typedef struct _Vanjee722zSerialPointCloudMsopPkt {
  uint8_t head[2];
  uint8_t protocol_major_version;
  uint8_t protocol_minor_version;
  uint8_t diagnostic_information_version;
  uint8_t data_type;
  uint8_t datetime[6];
  uint8_t timestamp[4];
  Vanjee722zSerialPointCloud blocks;
  uint32_t crc;
} Vanjee722zSerialPointCloudMsopPkt;

typedef struct _Vanjee722zSerialImu {
  int16_t imu_linear_acce_x;
  int16_t imu_linear_acce_y;
  int16_t imu_linear_acce_z;
  int16_t imu_angle_voc_x;
  int16_t imu_angle_voc_y;
  int16_t imu_angle_voc_z;
  uint16_t sequence_num;
} Vanjee722zSerialImu;

typedef struct _Vanjee722zSerialImuMsopPkt {
  uint8_t head[2];
  uint8_t protocol_major_version;
  uint8_t protocol_minor_version;
  uint8_t diagnostic_information_version;
  uint8_t data_type;
  uint8_t datetime[6];
  uint8_t timestamp[4];
  Vanjee722zSerialImu blocks;
  uint32_t crc;
} Vanjee722zSerialImuMsopPkt;

typedef struct _Vanjee722zSerialFaultCodeMsopPkt {
  uint8_t head[2];
  uint8_t fs_version;
  uint8_t datetime[6];
  uint8_t timestamp[4];
  uint8_t state_and_counter;
  uint8_t fault_code_info;
  uint16_t fault_code;
  uint8_t reserved[20];
  uint32_t crc;
} Vanjee722zSerialFaultCodeMsopPkt;

typedef struct _Vanjee722zResolutionState {
  int count_60 = 0;
  int count_120 = 0;
  int last_value = -1;
  int last_non_negative_result = -1;
  bool non_negative_result_flag = false;
} Vanjee722zResolutionState;
#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee722Z : public DecoderMech<T_PointCloud> {
 private:
  int32_t optcent_2_lidar_arg_ = 21570;
  float optcent_2_lidar_l_ = 2.067 * 1e-2;
  float optcent_2_lidar_z_ = 7.95 * 1e-3;

  std::vector<std::vector<double>> all_points_luminous_moment_serial_;  // Cache 16 channels for one circle of point cloud time difference

  const double luminous_period_of_ld_ = 3.33333 * 1e-4;           // Time interval at adjacent horizontal angles
  const double luminous_period_of_adjacent_ld_ = 2.08333 * 1e-5;  // Time interval between adjacent vertical angles within the group

  int32_t pre_hor_angle_ = -1;

  int16_t lidar_temperature_ = -27315;
  double temperature_update_ts_ = 0;

  int8_t pre_fault_code_frame_id_ = -1;
  int32_t pre_imu_frame_id_ = -1;
  int32_t pre_point_cloud_frame_id_ = -1;
  uint8_t publish_mode_ = 0;

  bool angle_param_get_flag_ = false;
  bool firmware_version_get_flag_ = false;
  bool sn_get_flag_ = false;
  bool acceleration_protocol_flag = false;
  bool acceleration_range_get_flag_ = false;
  bool scan_data_state_set_flag_ = true;
  bool scan_data_state_flag_ = true;
  double pre_temperature_data_publish_ts_ = 0;
  std::vector<int32_t> eccentricity_angles_;
  std::vector<int32_t> eccentricity_angles_real_;

  Vanjee722zResolutionState resolution_state_;

  std::map<uint16, std::string> get_lidar_param_;

  std::vector<uint8_t> buf_cache_;
  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);
  void initLdLuminousMoment(void);
  void lidarFormatParameterPublish(double pkt_ts, double pkt_host_ts);
  void setScanDataState(bool state);
  int32_t checkResolution(int32_t resolution, Vanjee722zResolutionState &state);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 100;

  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  void decodeMsopPktImu(const uint8_t *pkt, size_t size);
  bool decodeMsopPktSerialPointCloud(const uint8_t *pkt, size_t size);
  void decodeMsopPktFaultCode(const uint8_t *pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee722Z() = default;
  explicit DecoderVanjee722Z(const WJDecoderParam &param);

  void SendSerialImuData(Vanjee722zSerialImu difop, double timestamp, double lidar_timestamp);
};

template <typename T_PointCloud>
void DecoderVanjee722Z<T_PointCloud>::initLdLuminousMoment() {
  all_points_luminous_moment_serial_.resize(2);
  all_points_luminous_moment_serial_[0].resize(9600);
  all_points_luminous_moment_serial_[1].resize(4800);
  for (uint16_t col = 0; col < 600; col++) {
    for (uint8_t row = 0; row < 16; row++) {
      if (col < 300) {
        for (int i = 0; i < 2; i++) {
          all_points_luminous_moment_serial_[i][col * 16 + row] = col * luminous_period_of_ld_ + row * luminous_period_of_adjacent_ld_;
        }
      }
      all_points_luminous_moment_serial_[0][col * 16 + row] = col * luminous_period_of_ld_ + row * luminous_period_of_adjacent_ld_;
    }
  }
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee722Z<T_PointCloud>::getConstParam(uint8_t mode) {
  // WJ_INFOL << "publish_mode ============mode=================" << mode << WJ_REND;
  uint16_t msop_len = 80;
  uint16_t laser_num = 16;
  uint16_t block_num = 1;
  uint16_t chan_num = 16;
  float distance_min = 0.01f;
  float distance_max = 100.0f;
  float distance_resolution = 0.002f;
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
inline DecoderVanjee722Z<T_PointCloud>::DecoderVanjee722Z(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  publish_mode_ = param.publish_mode;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);
  this->m_imu_params_get_ = std::make_shared<ImuParamGet>(0, param.transform_param);

  this->start_angle_ = this->param_.start_angle * 1000;
  this->end_angle_ = this->param_.end_angle * 1000;

  if (this->param_.config_from_file) {
    this->chan_angles_.loadFromFile(this->param_.angle_path_ver);
  }
  initLdLuminousMoment();
}

template <typename T_PointCloud>
inline bool DecoderVanjee722Z<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  std::vector<uint8_t> data;
  if (buf_cache_.size() > 0) {
    std::copy(buf_cache_.begin(), buf_cache_.end(), std::back_inserter(data));
    std::copy(pkt, pkt + size, std::back_inserter(data));
  } else {
    std::copy(pkt, pkt + size, std::back_inserter(data));
  }

  buf_cache_.clear();
  buf_cache_.shrink_to_fit();

  uint32 indexLast = 0;
  for (size_t i = 0; i < data.size(); i++) {
    if (data.size() - i < 34)
      break;

    if (data[i] != 0xee || (data[i + 1] != 0xff && data[i + 1] != 0xdd)) {
      indexLast = i + 1;
      continue;
    }

    if (data[i + 1] == 0xff) {
      if (data[i + 5] == 0) {
        if (data.size() - i < 80) {
          indexLast = i;
          break;
        } else {
          uint32_t crc_check_80 = this->crc32Mpeg2Padded(&data[i], 76);
          uint32_t crc_pkg_80 = data[i + 76] | (data[i + 77] << 8) | (data[i + 78] << 16) | (data[i + 79] << 24);
          if (crc_check_80 == crc_pkg_80) {
            if (decodeMsopPktSerialPointCloud(&data[i], 80)) {
              ret = true;
              indexLast = i + 80;
              break;
            } else {
              i += 80 - 1;
              indexLast = i;
              continue;
            }
          } else {
            indexLast = i + 1;
            break;
          }
        }
      } else if (data[i + 5] == 1) {
        uint32_t crc_check_34 = this->crc32Mpeg2Padded(&data[i], 30);
        uint32_t crc_pkg_34 = data[i + 30] | (data[i + 31] << 8) | (data[i + 32] << 16) | (data[i + 33] << 24);
        if (crc_check_34 == crc_pkg_34) {
          decodeMsopPktImu(&data[i], 34);
          i += 34 - 1;
          indexLast = i;
          continue;
        } else {
          indexLast = i + 1;
        }
      } else {
        indexLast = i + 1;
      }
    } else {
      if (data.size() - i < 41) {
        indexLast = i;
        break;
      } else {
        uint32_t crc_check_41 = this->crc32Mpeg2Padded(&data[i + 13], 24);
        uint32_t crc_pkg_41 = data[i + 37] | (data[i + 38] << 8) | (data[i + 39] << 16) | (data[i + 40] << 24);
        if (crc_check_41 == crc_pkg_41) {
          decodeMsopPktFaultCode(&data[i], 41);
          i += 41 - 1;
          indexLast = i;
          continue;
        } else {
          indexLast = i + 1;
          break;
        }
      }
    }
    indexLast = i + 1;
  }

  if (indexLast < data.size()) {
    buf_cache_.assign(data.begin() + indexLast, data.end());
  }

  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee722Z<T_PointCloud>::decodeMsopPktFaultCode(const uint8_t *pkt, size_t size) {
  if ((!this->param_.device_ctrl_state_enable && !this->param_.send_lidar_param_enable) || size != sizeof(Vanjee722zSerialFaultCodeMsopPkt))
    return;

  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;
  pkt_host_ts = getTimeHost() * 1e-6;

  auto &packet = *(Vanjee722zSerialFaultCodeMsopPkt *)pkt;

  WJTimestampYMD tm{
      (uint8_t)(packet.datetime[0] - 100), packet.datetime[1], packet.datetime[2], packet.datetime[3], packet.datetime[4], packet.datetime[5]};
  double usec = (packet.timestamp[0] + (packet.timestamp[1] << 8) + (packet.timestamp[2] << 16) + (packet.timestamp[3] << 24)) * 1e-6;
  pkt_lidar_ts = parseTimeYMD(&tm) * 1e-6 + usec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else
    pkt_ts = pkt_lidar_ts;

  uint8_t frame_id = packet.state_and_counter & 0x07;
  uint32_t loss_packets_num = (frame_id + 8 - pre_fault_code_frame_id_) % 8;
  if (loss_packets_num > 1 && pre_fault_code_frame_id_ >= 0) {
    WJ_WARNING << "fault code: loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  }
  if (frame_id == pre_fault_code_frame_id_) {
    WJ_WARNING << "fault code: loss " << 7 << " packets" << WJ_REND;
  }
  pre_fault_code_frame_id_ = frame_id;

  uint8_t lidar_state = (packet.state_and_counter >> 5) & 0x07;
  if (lidar_state == 1 || lidar_state == 2) {
    this->deviceStatePublish(0, packet.fault_code, lidar_state, pkt_ts);
  }
}

template <typename T_PointCloud>
void DecoderVanjee722Z<T_PointCloud>::decodeMsopPktImu(const uint8_t *pkt, size_t size) {
  if (this->param_.imu_enable == -1 || size != sizeof(Vanjee722zSerialImuMsopPkt))
    return;

  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  pkt_host_ts = getTimeHost() * 1e-6;

  auto &packet = *(Vanjee722zSerialImuMsopPkt *)pkt;

  WJTimestampYMD tm{
      (uint8_t)(packet.datetime[0] - 100), packet.datetime[1], packet.datetime[2], packet.datetime[3], packet.datetime[4], packet.datetime[5]};
  double usec = (packet.timestamp[0] + (packet.timestamp[1] << 8) + (packet.timestamp[2] << 16) + (packet.timestamp[3] << 24)) * 1e-6;
  pkt_lidar_ts = parseTimeYMD(&tm) * 1e-6 + usec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else
    pkt_ts = pkt_lidar_ts;

  uint32_t loss_packets_num = (packet.blocks.sequence_num + 65536 - pre_imu_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_imu_frame_id_ >= 0)
    WJ_WARNING << "imu: loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_imu_frame_id_ = packet.blocks.sequence_num;

  if (pkt_lidar_ts != this->prev_pkt_ts_) {
    SendSerialImuData(packet.blocks, pkt_ts, pkt_lidar_ts);
  }
}

template <typename T_PointCloud>
void DecoderVanjee722Z<T_PointCloud>::lidarFormatParameterPublish(double pkt_ts, double pkt_host_ts) {
  this->mtx_lidar_param_.lock();
  if (this->get_lidar_param_msg_.count((uint16_t)LidarParam::temperature) > 0) {
    LidarParameterInterface get_lidar_param_msg = this->get_lidar_param_msg_.at((uint16_t)LidarParam::temperature);
    if (get_lidar_param_msg.cmd_type == 0) {
      LidarParameterInterface lidar_param;
      lidar_param.cmd_id = get_lidar_param_msg.cmd_id;
      lidar_param.cmd_type = get_lidar_param_msg.cmd_type;
      lidar_param.repeat_interval = get_lidar_param_msg.repeat_interval;
      this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
      if (get_lidar_param_msg.repeat_interval == 0) {
        this->lidarParameterPublish(lidar_param, pkt_ts);
        this->get_lidar_param_msg_.erase(get_lidar_param_msg.cmd_id);
      } else {
        if (pre_temperature_data_publish_ts_ == 0) {
          pre_temperature_data_publish_ts_ = pkt_host_ts;
          this->lidarParameterPublish(lidar_param, pkt_ts);
        } else {
          if ((pkt_host_ts - pre_temperature_data_publish_ts_) > get_lidar_param_msg.repeat_interval * 1e-3) {
            pre_temperature_data_publish_ts_ = pkt_host_ts;
            this->lidarParameterPublish(lidar_param, pkt_ts);
          } else if (pkt_host_ts < pre_temperature_data_publish_ts_) {
            pre_temperature_data_publish_ts_ = pkt_host_ts;
          }
        }
      }
    } else {
      this->get_lidar_param_msg_.erase(get_lidar_param_msg.cmd_id);
    }
  }

  if (this->get_lidar_param_msg_.count((uint16_t)LidarParam::firmware_version) > 0) {
    LidarParameterInterface get_lidar_param_msg = this->get_lidar_param_msg_.at((uint16_t)LidarParam::firmware_version);
    if (get_lidar_param_msg.cmd_type == 0) {
      LidarParameterInterface lidar_param;
      lidar_param.cmd_id = get_lidar_param_msg.cmd_id;
      lidar_param.cmd_type = get_lidar_param_msg.cmd_type;
      lidar_param.repeat_interval = 0;
      this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
      this->lidarParameterPublish(lidar_param, pkt_ts);
    }
    this->get_lidar_param_msg_.erase(get_lidar_param_msg.cmd_id);
  }

  if (this->get_lidar_param_msg_.count((uint16_t)LidarParam::sn) > 0) {
    LidarParameterInterface get_lidar_param_msg = this->get_lidar_param_msg_.at((uint16_t)LidarParam::sn);
    if (get_lidar_param_msg.cmd_type == 0) {
      LidarParameterInterface lidar_param;
      lidar_param.cmd_id = get_lidar_param_msg.cmd_id;
      lidar_param.cmd_type = get_lidar_param_msg.cmd_type;
      lidar_param.repeat_interval = 0;
      this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
      this->lidarParameterPublish(lidar_param, pkt_ts);
    }
    this->get_lidar_param_msg_.erase(get_lidar_param_msg.cmd_id);
  }

  if (this->get_lidar_param_msg_.count((uint16_t)LidarParam::acceleration) > 0) {
    if (acceleration_protocol_flag) {
      LidarParameterInterface get_lidar_param_msg = this->get_lidar_param_msg_.at((uint16_t)LidarParam::acceleration);
      if (get_lidar_param_msg.cmd_type == 0) {
        LidarParameterInterface lidar_param;
        lidar_param.cmd_id = get_lidar_param_msg.cmd_id;
        lidar_param.cmd_type = get_lidar_param_msg.cmd_type;
        lidar_param.repeat_interval = 0;
        this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
        this->lidarParameterPublish(lidar_param, pkt_ts);
      } else if (get_lidar_param_msg.cmd_type == 1) {
        uint8_t state = 0xff;
        JsonParser parser(get_lidar_param_msg.data);
        auto json_value = parser.parse();
        if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
          if (obj->has("acceleration_range")) {
            auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("acceleration_range"));
            if (result != nullptr && result->value() <= 1) {
              state = static_cast<uint8_t>(result->value());
            }
          }
        }
        if (state <= 1) {
          std::shared_ptr<Params_AccelerationRangeSet722Z> param =
              std::shared_ptr<Params_AccelerationRangeSet722Z>(new Params_AccelerationRangeSet722Z());
          param->state_ = state;
          GetDifoCtrlClass param_set(*(std::make_shared<Protocol_AccelerationRangeSet722Z>(param)->SetRequest()), false);

          uint16_t cmd = CmdRepository722Z::CreateInstance()->sp_acceleration_range_set_->GetCmdKey();
          auto it = (*(this->get_difo_ctrl_map_ptr_)).find(cmd);
          if (it != (*(this->get_difo_ctrl_map_ptr_)).end()) {
            (*(this->get_difo_ctrl_map_ptr_))[cmd] = param_set;
          } else {
            (*(this->get_difo_ctrl_map_ptr_)).emplace(cmd, param_set);
          }
        }
      } else {
        ;
      }
    } else {
      WJ_WARNING << "This firmware version does not support the acceleration range query and setting functions." << WJ_REND;
    }
    this->get_lidar_param_msg_.erase((uint16_t)LidarParam::acceleration);
  }
  this->mtx_lidar_param_.unlock();
}

template <typename T_PointCloud>
int32_t DecoderVanjee722Z<T_PointCloud>::checkResolution(int32_t resolution, Vanjee722zResolutionState &state) {
  if (resolution != 60 && resolution != 120) {
    return -1;
  }

  if (state.non_negative_result_flag) {
    if (resolution == state.last_value) {
      if (resolution == 60) {
        state.count_60++;
        state.count_120 = 0;
      } else {
        state.count_120++;
        state.count_60 = 0;
      }
    } else {
      state.count_60 = (resolution == 60) ? 1 : 0;
      state.count_120 = (resolution == 120) ? 1 : 0;
      state.last_value = resolution;
    }

    if ((state.last_non_negative_result == 0 && state.count_120 >= 300) || (state.last_non_negative_result == 1 && state.count_60 >= 300)) {
      state.non_negative_result_flag = true;
      state.last_non_negative_result = (state.last_non_negative_result == 0) ? 1 : 0;
      state.count_60 = 0;
      state.count_120 = 0;
      state.last_value = -1;
      return state.last_non_negative_result;
    }

    return state.last_non_negative_result;
  }

  if (resolution == state.last_value) {
    if (resolution == 60) {
      state.count_60++;
      state.count_120 = 0;
    } else {
      state.count_120++;
      state.count_60 = 0;
    }
  } else {
    state.count_60 = (resolution == 60) ? 1 : 0;
    state.count_120 = (resolution == 120) ? 1 : 0;
    state.last_value = resolution;
  }

  if (state.count_60 >= 300) {
    state.count_60 = 0;
    state.count_120 = 0;
    state.last_value = -1;
    state.non_negative_result_flag = true;
    state.last_non_negative_result = 0;
    return 0;
  }

  if (state.count_120 >= 300) {
    state.count_60 = 0;
    state.count_120 = 0;
    state.last_value = -1;
    state.non_negative_result_flag = true;
    state.last_non_negative_result = 1;
    return 1;
  }

  return -1;
}

template <typename T_PointCloud>
bool DecoderVanjee722Z<T_PointCloud>::decodeMsopPktSerialPointCloud(const uint8_t *pkt, size_t size) {
  if (!this->param_.point_cloud_enable || size != sizeof(Vanjee722zSerialPointCloudMsopPkt))
    return false;

  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  pkt_host_ts = getTimeHost() * 1e-6;

  auto &packet = *(Vanjee722zSerialPointCloudMsopPkt *)pkt;

  WJTimestampYMD tm{
      (uint8_t)(packet.datetime[0] - 100), packet.datetime[1], packet.datetime[2], packet.datetime[3], packet.datetime[4], packet.datetime[5]};
  double usec = (packet.timestamp[0] + (packet.timestamp[1] << 8) + (packet.timestamp[2] << 16) + (packet.timestamp[3] << 24)) * 1e-6;
  pkt_lidar_ts = parseTimeYMD(&tm) * 1e-6 + usec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else
    pkt_ts = pkt_lidar_ts;

  uint32_t loss_packets_num = (packet.blocks.sequence_num + 65536 - pre_point_cloud_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_point_cloud_frame_id_ >= 0)
    WJ_WARNING << "point cloud: loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_point_cloud_frame_id_ = packet.blocks.sequence_num;

  if (packet.blocks.reserved_id == 0x01) {
    lidar_temperature_ = packet.blocks.reserved_info;
    if (get_lidar_param_.count((uint16_t)LidarParam::temperature) > 0) {
      get_lidar_param_[(uint16_t)LidarParam::temperature] = std::to_string(lidar_temperature_ * 1e-2);
    } else {
      get_lidar_param_.emplace((uint16_t)LidarParam::temperature, std::to_string(lidar_temperature_ * 1e-2));
    }
  }

  lidarFormatParameterPublish(pkt_ts, pkt_host_ts);

  if (pkt_host_ts - temperature_update_ts_ > 30) {
    temperature_update_ts_ = pkt_host_ts;
    if (lidar_temperature_ > -27315) {
      WJ_INFOL << "Temperature: " << (double)lidar_temperature_ * 1e-2 << " °C" << WJ_REND;
      if (this->param_.device_ctrl_state_enable) {
        this->device_ctrl_->cmd_id = 2;
        this->device_ctrl_->cmd_param = (uint16_t)lidar_temperature_;
        this->device_ctrl_->cmd_state = 1;
        this->cb_device_ctrl_state_(pkt_ts);
      }
    }
  } else if (temperature_update_ts_ - pkt_host_ts > 30) {
    temperature_update_ts_ = pkt_host_ts;
    WJ_WARNING << "Server time update, temperature param publish delayed" << WJ_REND;
  }

  int32_t resolution = 60;
  uint8_t resolution_index = 0;
  if (pre_hor_angle_ == -1) {
    pre_hor_angle_ = packet.blocks.azimuth % 36000;
    return false;
  } else {
    if (loss_packets_num >= 300) {
      pre_hor_angle_ = packet.blocks.azimuth % 36000;
      return false;
    }
    resolution = ((packet.blocks.azimuth + 36000 - pre_hor_angle_) % 36000) / loss_packets_num;
    if (resolution < 90) {
      resolution = 60;
      resolution_index = 0;
    } else {
      resolution = 120;
      resolution_index = 1;
    }

    int32_t ret_resolution = checkResolution(resolution, resolution_state_);
    if (ret_resolution == 0) {
      resolution = 60;
      resolution_index = 0;
    } else if (ret_resolution == 1) {
      resolution = 120;
      resolution_index = 1;
    } else {
      ;
    }
  }

  const Vanjee722zSerialPointCloud &block = packet.blocks;
  int32_t azimuth = block.azimuth % 36000;
  int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

  if (this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans >= resolution) {
    uint32_t point_gap_num = (azimuth / resolution) * 16;
    this->last_point_ts_ = pkt_ts - all_points_luminous_moment_serial_[resolution_index][point_gap_num];
    this->first_point_ts_ =
        this->last_point_ts_ - all_points_luminous_moment_serial_[resolution_index][all_points_luminous_moment_serial_[resolution_index].size() - 1];
    this->cb_split_frame_(16, this->cloudTs());
    ret = true;
  }

  double timestamp_point;
  uint32_t cur_blk_first_point_id = azimuth / resolution * 16;
  for (uint16_t chan = 0; chan < 16; chan++) {
    uint8_t tag = (uint8_t)((block.dirty_degree & (0x00000003 << (chan * 2))) >> (chan * 2));
    float x, y, z, xy;

    uint32_t point_id = cur_blk_first_point_id + chan;
    if (this->param_.ts_first_point == true) {
      timestamp_point = all_points_luminous_moment_serial_[resolution_index][point_id];
    } else {
      timestamp_point = all_points_luminous_moment_serial_[resolution_index][point_id] -
                        all_points_luminous_moment_serial_[resolution_index][all_points_luminous_moment_serial_[resolution_index].size() - 1];
    }

    const Vanjee722zSerialBlockChannel &channel = block.channel[chan];

    float distance = channel.distance * this->const_param_.DISTANCE_RES;
    int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
    int32_t angle_horiz = (this->chan_angles_.horizAdjust(chan, azimuth * 10) + 360000) % 360000;

    if (this->distance_section_.in(distance)) {
      int32_t optcent_2_lidar_angle_hor = (azimuth * 10 + optcent_2_lidar_arg_ + 360000) % 360000;
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
      setTag(point, tag);
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
      setTag(point, tag);
#ifdef ENABLE_GTEST
      setPointId(point, point_id);
      setHorAngle(point, angle_horiz / 1000.0);
      setVerAngle(point, angle_vert / 1000.0);
      setDistance(point, distance);
#endif
      this->point_cloud_->points.emplace_back(point);
    }
  }

  if (azimuth_trans < resolution) {
    if (((pre_hor_angle_ + resolution) % 36000) < resolution) {
      this->point_cloud_->points.clear();
    } else {
      this->last_point_ts_ = pkt_ts;
      this->first_point_ts_ = this->last_point_ts_ -
                              all_points_luminous_moment_serial_[resolution_index][all_points_luminous_moment_serial_[resolution_index].size() - 1];
      this->cb_split_frame_(16, this->cloudTs());
      ret = true;
    }
  }
  pre_hor_angle_ = azimuth;
  this->prev_pkt_ts_ = pkt_lidar_ts;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee722Z<T_PointCloud>::SendSerialImuData(Vanjee722zSerialImu difop, double timestamp, double lidar_timestamp) {
  double imu_angle_voc_x = difop.imu_angle_voc_x / 32.8 * 0.0174533;
  double imu_angle_voc_y = difop.imu_angle_voc_y / 32.8 * 0.0174533;
  double imu_angle_voc_z = difop.imu_angle_voc_z / 32.8 * 0.0174533;

  double imu_linear_acce_x = difop.imu_linear_acce_x / 8192.0 * 9.81;
  double imu_linear_acce_y = difop.imu_linear_acce_y / 8192.0 * 9.81;
  double imu_linear_acce_z = difop.imu_linear_acce_z / 8192.0 * 9.81;

  this->m_imu_params_get_->rotateImu(imu_linear_acce_x, imu_linear_acce_y, imu_linear_acce_z, imu_angle_voc_x, imu_angle_voc_y, imu_angle_voc_z);

  this->imu_packet_->timestamp = timestamp;
  this->imu_packet_->angular_voc[0] = imu_angle_voc_x;
  this->imu_packet_->angular_voc[1] = imu_angle_voc_y;
  this->imu_packet_->angular_voc[2] = imu_angle_voc_z;

  this->imu_packet_->linear_acce[0] = imu_linear_acce_x;
  this->imu_packet_->linear_acce[1] = imu_linear_acce_y;
  this->imu_packet_->linear_acce[2] = imu_linear_acce_z;

  this->imu_packet_->orientation[0] = 0;
  this->imu_packet_->orientation[1] = 0;
  this->imu_packet_->orientation[2] = 0;
  this->imu_packet_->orientation[3] = 0;

  this->cb_imu_pkt_();
}

template <typename T_PointCloud>
void DecoderVanjee722Z<T_PointCloud>::setScanDataState(bool state) {
  if (scan_data_state_set_flag_) {
    std::shared_ptr<Params_ScanDataStateSet722Z> params_ScanDataStateSet =
        std::shared_ptr<Params_ScanDataStateSet722Z>(new Params_ScanDataStateSet722Z());
    if (state) {
      params_ScanDataStateSet->state_ = 0;
    } else {
      params_ScanDataStateSet->state_ = 1;
    }
    GetDifoCtrlClass getDifoCtrlData_ScanDataStateSet(*(std::make_shared<Protocol_ScanDataStateSet722Z>(params_ScanDataStateSet)->SetRequest()),
                                                      false, 100);
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository722Z::CreateInstance()->sp_scan_data_state_set_->GetCmdKey()] =
        getDifoCtrlData_ScanDataStateSet;
    scan_data_state_set_flag_ = false;
  }
}

template <typename T_PointCloud>
void DecoderVanjee722Z<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract722Z> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository722Z::CreateInstance()->sp_ld_value_get_)) {
    p = std::make_shared<Protocol_LDValueGet722Z>();
  } else if (*sp_cmd == *(CmdRepository722Z::CreateInstance()->sp_firmware_version_get_)) {
    p = std::make_shared<Protocol_FirmwareVersionGet722Z>();
  } else if (*sp_cmd == *(CmdRepository722Z::CreateInstance()->sp_sn_get_)) {
    p = std::make_shared<Protocol_SnGet722Z>();
  } else if (*sp_cmd == *(CmdRepository722Z::CreateInstance()->sp_scan_data_state_set_)) {
    p = std::make_shared<Protocol_ScanDataStateSet722Z>();
  } else if (*sp_cmd == *(CmdRepository722Z::CreateInstance()->sp_acceleration_range_get_)) {
    p = std::make_shared<Protocol_AccelerationRangeGet722Z>();
  } else if (*sp_cmd == *(CmdRepository722Z::CreateInstance()->sp_acceleration_range_set_)) {
    p = std::make_shared<Protocol_AccelerationRangeSet722Z>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_LDValue722Z)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }
    if (!angle_param_get_flag_) {
      std::shared_ptr<Params_LDValue722Z> param = std::dynamic_pointer_cast<Params_LDValue722Z>(params);
      std::vector<double> vert_angles;
      std::vector<double> offset_angles;
      for (int num_of_lines = 0; num_of_lines < param->num_of_lines_; num_of_lines++) {
        vert_angles.push_back((double)(param->ver_angle_[num_of_lines] / 1000.0));
        offset_angles.push_back((double)(param->offset_angle_[num_of_lines] / 1000.0));
      }
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver, param->num_of_lines_, vert_angles, offset_angles);
        WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
      }
      if (this->param_.send_packet_enable) {
        this->protocol_ver_angle_table.assign(param->protocol_data_.begin(), param->protocol_data_.end());
      }
      angle_param_get_flag_ = true;
    }

    if (!this->param_.recv_lidar_param_cmd_enable) {
      if (angle_param_get_flag_) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
        acceleration_range_get_flag_ = true;
        Decoder<T_PointCloud>::angles_ready_ = true;
      }
    } else {
      if (angle_param_get_flag_ && firmware_version_get_flag_ && sn_get_flag_) {
        if (this->param_.query_via_external_interface_enable) {
          if (acceleration_range_get_flag_) {
            if (!Decoder<T_PointCloud>::angles_ready_) {
              Decoder<T_PointCloud>::angles_ready_ = true;
            }
          }
        } else {
          if ((*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository722Z::CreateInstance()->sp_acceleration_range_get_->GetCmdKey()]
                      .getSendCnt() <= 3 &&
              !acceleration_range_get_flag_) {
            return;
          } else {
            (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
            (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository722Z::CreateInstance()->sp_acceleration_range_get_->GetCmdKey()]
                .setStopFlag(true);
            acceleration_range_get_flag_ = true;
            Decoder<T_PointCloud>::angles_ready_ = true;
            if (!scan_data_state_flag_) {
              setScanDataState(true);
            }
          }
        }
      }
    }
  } else if (typeid(*params) == typeid(Params_FirmwareVersion722Z)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }
    if (!firmware_version_get_flag_) {
      std::shared_ptr<Params_FirmwareVersion722Z> param = std::dynamic_pointer_cast<Params_FirmwareVersion722Z>(params);
      if (get_lidar_param_.count((uint16_t)LidarParam::firmware_version) > 0) {
        get_lidar_param_[(uint16_t)LidarParam::firmware_version] = param->firmware_version_;
      } else {
        get_lidar_param_.emplace((uint16_t)LidarParam::firmware_version, param->firmware_version_);
      }

      WJ_INFOL << "Get lidar firmware version succ ( " << param->firmware_version_ << " )." << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      firmware_version_get_flag_ = true;
      if (this->param_.query_via_external_interface_enable) {
        if (angle_param_get_flag_ && firmware_version_get_flag_ && sn_get_flag_ && acceleration_range_get_flag_) {
          if (!Decoder<T_PointCloud>::angles_ready_) {
            Decoder<T_PointCloud>::angles_ready_ = true;
          }
        }
      }
    }

  } else if (typeid(*params) == typeid(Params_Sn722Z)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }
    if (!sn_get_flag_) {
      std::shared_ptr<Params_Sn722Z> param = std::dynamic_pointer_cast<Params_Sn722Z>(params);
      if (get_lidar_param_.count((uint16_t)LidarParam::sn) > 0) {
        get_lidar_param_[(uint16_t)LidarParam::sn] = param->sn_;
      } else {
        get_lidar_param_.emplace((uint16_t)LidarParam::sn, param->sn_);
      }

      WJ_INFOL << "Get lidar sn succ ( " << param->sn_ << " )." << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      sn_get_flag_ = true;
      if (this->param_.query_via_external_interface_enable) {
        if (angle_param_get_flag_ && firmware_version_get_flag_ && sn_get_flag_ && acceleration_range_get_flag_) {
          if (!Decoder<T_PointCloud>::angles_ready_) {
            Decoder<T_PointCloud>::angles_ready_ = true;
          }
        }
      }
    }

  } else if (typeid(*params) == typeid(Params_ScanDataStateSet722Z)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_ScanDataStateSet722Z> param = std::dynamic_pointer_cast<Params_ScanDataStateSet722Z>(params);
    if (param->flag_) {
      if (scan_data_state_flag_ && !angle_param_get_flag_ && !firmware_version_get_flag_ && !sn_get_flag_ && !acceleration_range_get_flag_) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
        scan_data_state_flag_ = false;
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository722Z::CreateInstance()->sp_ld_value_get_->GetCmdKey()].setStopFlag(false);
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository722Z::CreateInstance()->sp_firmware_version_get_->GetCmdKey()].setStopFlag(
            false);
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository722Z::CreateInstance()->sp_sn_get_->GetCmdKey()].setStopFlag(false);
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository722Z::CreateInstance()->sp_acceleration_range_get_->GetCmdKey()].setStopFlag(
            false);
        WJ_INFOL << "Set lidar scan data state succ (stop)." << WJ_REND;
      } else if (!scan_data_state_flag_ && angle_param_get_flag_ && firmware_version_get_flag_ && sn_get_flag_ && acceleration_range_get_flag_) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
        WJ_INFOL << "Set lidar scan data state succ (start)." << WJ_REND;
        scan_data_state_flag_ = true;
      } else {
        ;
      }
    } else {
      WJ_WARNING << "Set lidar scan data state err." << WJ_REND;
    }
  } else if (typeid(*params) == typeid(Params_AccelerationRangeGet722Z)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_AccelerationRangeGet722Z> param = std::dynamic_pointer_cast<Params_AccelerationRangeGet722Z>(params);
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);

    acceleration_protocol_flag = true;
    if (param->state_ <= 3) {
      if (get_lidar_param_.count((uint16_t)LidarParam::acceleration) > 0) {
        get_lidar_param_[(uint16_t)LidarParam::acceleration] = std::to_string((uint16_t)param->state_);
      } else {
        get_lidar_param_.emplace((uint16_t)LidarParam::acceleration, std::to_string((uint16_t)param->state_));
      }

      WJ_INFOL << "Get acceleration range succ ( ±" << std::exp2((double)param->state_ + 1.0) << "G )." << WJ_REND;

      if (this->param_.send_lidar_param_enable) {
        LidarParameterInterface lidar_param;
        lidar_param.cmd_id = (uint16_t)LidarParam::acceleration;
        lidar_param.cmd_type = 0;
        lidar_param.repeat_interval = 0;
        this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
        this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);
      }
      acceleration_range_get_flag_ = true;
      if (this->param_.query_via_external_interface_enable) {
        if (angle_param_get_flag_ && firmware_version_get_flag_ && sn_get_flag_ && acceleration_range_get_flag_) {
          if (!Decoder<T_PointCloud>::angles_ready_) {
            Decoder<T_PointCloud>::angles_ready_ = true;
          }
        }
      }
    } else {
      WJ_WARNING << "Get acceleration range err." << WJ_REND;
    }
  } else if (typeid(*params) == typeid(Params_AccelerationRangeSet722Z)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_AccelerationRangeSet722Z> param = std::dynamic_pointer_cast<Params_AccelerationRangeSet722Z>(params);
    if (param->flag_) {
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository722Z::CreateInstance()->sp_acceleration_range_get_->GetCmdKey()].setStopFlag(
          false);
    } else {
      WJ_WARNING << "Set acceleration range err." << WJ_REND;
    }
  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar

}  // namespace vanjee
