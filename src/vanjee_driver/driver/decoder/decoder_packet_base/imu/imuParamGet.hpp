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

#include <math.h>

#include <iostream>
#include <memory>
#include <numeric>
#include <vector>
#ifdef ENABLE_TRANSFORM
#include <Eigen/Dense>
#endif

#include <vanjee_driver/driver/decoder/decoder_packet_base/imu/complementary_filter.hpp>
#include <vanjee_driver/driver/driver_param.hpp>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/common/wj_log.hpp"

#define HDL_Grabber_toRadians(x) ((x)*M_PI / 180.0)

namespace vanjee {
namespace lidar {
class ImuParamGet {
 public:
  double x_ang_k_;
  double x_ang_b_;
  double y_ang_k_;
  double y_ang_b_;
  double z_ang_k_;
  double z_ang_b_;

  double x_acc_k_;
  double x_acc_b_;
  double y_acc_k_;
  double y_acc_b_;
  double z_acc_k_;
  double z_acc_b_;

  typedef struct _ZeroParam_ {
    double x;
    double y;
    double z;
  } ZeroParam_;

  ZeroParam_ zero_param_;
  std::vector<ZeroParam_> zero_param_buf_;

  double rotate_b_[2][2];

  int32_t me_zero_param_time_;
  int32_t aim_me_zero_param_time_;

  double pre_pkt_time_;

  struct imuGetResultPa {
    double q0;
    double q1;
    double q2;
    double q3;

    double x_angle;
    double y_angle;
    double z_angle;

    double x_acc;
    double y_acc;
    double z_acc;

    void init() {
      q0 = q1 = q2 = q3 = 0;
      x_angle = y_angle = z_angle = 0;
      x_acc = y_acc = z_acc = 0;
    }
  };

  imuGetResultPa imu_result_stu_;

  // std::shared_ptr<imu_tools::ComplementaryFilter> m_filter_;
  imu_tools::ComplementaryFilter m_filter_;

  WJTransfromParam param_;
#ifdef ENABLE_TRANSFORM
  Eigen::Matrix3d transform_r_;
  Eigen::Vector3d transform_t_;

  inline void getImuTransMatrix(double x, double y, double z, double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    transform_r_ = yawAngle * pitchAngle * rollAngle;
    transform_t_ << x, y, z;
  }

  inline void imuTransform(double &x_acc, double &y_acc, double &z_acc, double &x_ang, double &y_ang, double &z_ang) {
    if (  // param_.x_imu != 0 || param_.y_imu  != 0 || param_.z_imu != 0 ||
        param_.roll_imu != 0 || param_.pitch_imu != 0 || param_.yaw_imu != 0) {
      Eigen::Vector3d acc_in(x_acc, y_acc, z_acc);
      Eigen::Vector3d gyro_in(x_ang, y_ang, z_ang);
      Eigen::Vector3d acc_out = transform_r_ * acc_in;
      Eigen::Vector3d gyro_out = transform_r_ * gyro_in;
      // acc_out += gyro_out.cross(gyro_out.cross(transform_t_));
      x_acc = acc_out[0];
      y_acc = acc_out[1];
      z_acc = acc_out[2];
      x_ang = gyro_out[0];
      y_ang = gyro_out[1];
      z_ang = gyro_out[2];
    }
  }
#endif

  ImuParamGet(double axis_offset = 0.0, const WJTransfromParam &param = WJTransfromParam()) {
    param_ = param;
    x_ang_k_ = y_ang_k_ = z_ang_k_ = 1;
    x_ang_b_ = y_ang_b_ = z_ang_b_ = 0;

    x_acc_k_ = y_acc_k_ = z_acc_k_ = 1;
    x_acc_b_ = y_acc_b_ = z_acc_b_ = 0;

    zero_param_.x = 0;
    zero_param_.y = 0;
    zero_param_.z = 0;

    rotate_b_[0][0] = cos(HDL_Grabber_toRadians(axis_offset));
    rotate_b_[0][1] = -sin(HDL_Grabber_toRadians(axis_offset));
    rotate_b_[1][0] = sin(HDL_Grabber_toRadians(axis_offset));
    rotate_b_[1][1] = cos(HDL_Grabber_toRadians(axis_offset));

    me_zero_param_time_ = 0;
    aim_me_zero_param_time_ = 40;

    pre_pkt_time_ = 0;

    imu_result_stu_.init();
#ifdef ENABLE_TRANSFORM
    getImuTransMatrix(param.x_imu, param.y_imu, param.z_imu, param.roll_imu, param.pitch_imu, param.yaw_imu);
#endif
  }

  ~ImuParamGet() {
  }

  inline void setImuTempCalibrationParams(double x_k, double x_b, double y_k, double y_b, double z_k, double z_b) {
    x_ang_k_ = x_k;
    x_ang_b_ = x_b;
    y_ang_k_ = y_k;
    y_ang_b_ = y_b;
    z_ang_k_ = z_k;
    z_ang_b_ = z_b;
  }

  inline void setImuAcceCalibrationParams(double x_k, double x_b, double y_k, double y_b, double z_k, double z_b) {
    x_acc_k_ = x_k;
    x_acc_b_ = x_b;
    y_acc_k_ = y_k;
    y_acc_b_ = y_b;
    z_acc_k_ = z_k;
    z_acc_b_ = z_b;
  }

  inline void imuAngCorrbyTemp(double &x_ang, double &y_ang, double &z_ang, double imu_temperature) {
    x_ang -= x_ang_k_ * (imu_temperature - 40) + x_ang_b_;
    y_ang -= y_ang_k_ * (imu_temperature - 40) + y_ang_b_;
    z_ang -= z_ang_k_ * (imu_temperature - 40) + z_ang_b_;
  }

  inline void imuAccCorrbyKB(double &x_acc, double &y_acc, double &z_acc) {
    x_acc = x_acc_k_ * x_acc - x_acc_b_;
    y_acc = y_acc_k_ * y_acc - y_acc_b_;
    z_acc = z_acc_k_ * z_acc - z_acc_b_;
  }

  inline void rotateImu(double &x_acc, double &y_acc, double &z_acc, double &x_ang, double &y_ang, double &z_ang) {
    double rotate_acc[2] = {x_acc, y_acc};
    double rotate_Ang[2] = {x_ang, y_ang};

    x_acc = rotate_acc[0] * rotate_b_[0][0] + rotate_acc[1] * rotate_b_[1][0];
    y_acc = rotate_acc[0] * rotate_b_[0][1] + rotate_acc[1] * rotate_b_[1][1];

    x_ang = rotate_Ang[0] * rotate_b_[0][0] + rotate_Ang[1] * rotate_b_[1][0];
    y_ang = rotate_Ang[0] * rotate_b_[0][1] + rotate_Ang[1] * rotate_b_[1][1];
  }

  int32_t imuGetZeroPa(double x_ang, double y_ang, double z_ang) {
    zero_param_.x += x_ang;
    zero_param_.y += y_ang;
    zero_param_.z += z_ang;

    return me_zero_param_time_++;
  }

  bool imuGet(double x_ang, double y_ang, double z_ang, double x_acc, double y_acc, double z_acc, double time, bool imu_orientation_enable,
              double temperature = -100.0, bool temperature_enable = true, bool old_coefficient_flag = true, bool calibration_param_enable = true,
              bool rotate_enable = true, bool zero_calibrate_enable = true) {
    if (temperature_enable) {
      if (temperature <= -100.0)
        return false;
      imuAngCorrbyTemp(x_ang, y_ang, z_ang, temperature);
    }

    if (old_coefficient_flag) {
      x_acc = x_acc / 1000 * 0.061 * 9.81;
      y_acc = y_acc / 1000 * 0.061 * 9.81;
      z_acc = z_acc / 1000 * 0.061 * 9.81;

      x_ang = x_ang / 1000 * 8.75 * 0.0174533;
      y_ang = y_ang / 1000 * 8.75 * 0.0174533;
      z_ang = z_ang / 1000 * 8.75 * 0.0174533;
    } else {
      x_acc = x_acc * 1e-6 * 9.81;
      y_acc = y_acc * 1e-6 * 9.81;
      z_acc = z_acc * 1e-6 * 9.81;

      x_ang = x_ang * 1e-3 * 0.0174533;
      y_ang = y_ang * 1e-3 * 0.0174533;
      z_ang = z_ang * 1e-3 * 0.0174533;
    }

    if (calibration_param_enable)
      imuAccCorrbyKB(x_acc, y_acc, z_acc);

    if (rotate_enable)
      rotateImu(x_acc, y_acc, z_acc, x_ang, y_ang, z_ang);

#ifdef ENABLE_TRANSFORM
    imuTransform(x_acc, y_acc, z_acc, x_ang, y_ang, z_ang);
#endif

    double offset_time;
    if (pre_pkt_time_ < time) {
      offset_time = time - pre_pkt_time_;
    } else {
      pre_pkt_time_ = time;
      return false;
    }
    pre_pkt_time_ = time;

    if (zero_calibrate_enable) {
      if (zero_param_buf_.size() < 5) {
        imuGetZeroPa(x_ang, y_ang, z_ang);
        if (me_zero_param_time_ % aim_me_zero_param_time_ == 0) {
          ZeroParam_ angle_offset;
          angle_offset.x = zero_param_.x / aim_me_zero_param_time_;
          angle_offset.y = zero_param_.y / aim_me_zero_param_time_;
          angle_offset.z = zero_param_.z / aim_me_zero_param_time_;
          zero_param_buf_.push_back(angle_offset);

          zero_param_.x = 0;
          zero_param_.y = 0;
          zero_param_.z = 0;

          if (zero_param_buf_.size() > 1) {
            if (std::abs(zero_param_buf_[zero_param_buf_.size() - 1].x - zero_param_buf_[zero_param_buf_.size() - 2].x) > 0.008 ||
                std::abs(zero_param_buf_[zero_param_buf_.size() - 1].y - zero_param_buf_[zero_param_buf_.size() - 2].y) > 0.008 ||
                std::abs(zero_param_buf_[zero_param_buf_.size() - 1].z - zero_param_buf_[zero_param_buf_.size() - 2].z) > 0.008) {
              zero_param_buf_.erase(zero_param_buf_.begin() + zero_param_buf_.size() - 2);
              WJ_WARNING << "Do not move during IMU calibration" << WJ_REND;
              WJ_INFO << "Waiting for IMU calibration..." << WJ_REND;
            }
          }
        }
        return false;
      } else if (zero_param_buf_.size() == 5) {
        zero_param_.x =
            std::accumulate(zero_param_buf_.begin(), zero_param_buf_.end(), 0.0, [](double sum, const ZeroParam_ &param) { return sum + param.x; }) /
            zero_param_buf_.size();
        zero_param_.y =
            std::accumulate(zero_param_buf_.begin(), zero_param_buf_.end(), 0.0, [](double sum, const ZeroParam_ &param) { return sum + param.y; }) /
            zero_param_buf_.size();
        zero_param_.z =
            std::accumulate(zero_param_buf_.begin(), zero_param_buf_.end(), 0.0, [](double sum, const ZeroParam_ &param) { return sum + param.z; }) /
            zero_param_buf_.size();
        zero_param_buf_.push_back(zero_param_);
        WJ_INFO << "Begin publish Imu" << WJ_REND;
      }
    } else {
      if (me_zero_param_time_ == 0) {
        WJ_INFO << "Begin publish Imu" << WJ_REND;
        me_zero_param_time_ = 1;
      }
    }

    x_ang = x_ang - zero_param_.x;
    y_ang = y_ang - zero_param_.y;
    z_ang = z_ang - zero_param_.z;

    imu_result_stu_.x_angle = x_ang;
    imu_result_stu_.y_angle = y_ang;
    imu_result_stu_.z_angle = z_ang;

    imu_result_stu_.x_acc = x_acc;
    imu_result_stu_.y_acc = y_acc;
    imu_result_stu_.z_acc = z_acc;

    static double q0 = 0;
    static double q1 = 0;
    static double q2 = 0;
    static double q3 = 0;

    if (imu_orientation_enable) {
      m_filter_.update(x_acc, y_acc, z_acc, x_ang, y_ang, z_ang, offset_time, q0, q1, q2, q3);
      m_filter_.getOrientation(q0, q1, q2, q3);
    }

    imu_result_stu_.q0 = q0;
    imu_result_stu_.q1 = q1;
    imu_result_stu_.q2 = q2;
    imu_result_stu_.q3 = q3;

    return true;
  }
};

}  // namespace lidar
}  // namespace vanjee