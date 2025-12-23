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
#include <memory>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"

namespace vanjee {
namespace lidar {
class DifopVanjee722H : public DifopBase {
 public:
  explicit DifopVanjee722H();
  virtual void initGetDifoCtrlDataMapPtr();
  virtual void addItem2GetDifoCtrlDataMapPtr(const DeviceCtrl& device_ctrl);
  virtual void addItem2GetDifoCtrlDataMapPtr(const LidarParameterInterface& lidar_parm);
};

inline DifopVanjee722H::DifopVanjee722H() {
  ProtocolBase pb;
  pb.SetByteOrder(ProtocolBase::DataEndiannessMode::little_endian);
  pb.SetProtocolVersion(ProtocolBase::ProtocolVersionDifop::version_v1);
  this->setOrgProtocolBase(pb);
}

void DifopVanjee722H::initGetDifoCtrlDataMapPtr() {
  getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16, GetDifoCtrlClass>>();

  GetDifoCtrlClass getDifoCtrlData_LdValueGet(*(std::make_shared<Protocol_LDValueGet722H>()->GetRequest()));
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722H::CreateInstance()->sp_ld_value_get_->GetCmdKey(), getDifoCtrlData_LdValueGet);

  GetDifoCtrlClass getDifoCtrlData_WorkModeGet(*(std::make_shared<Protocol_WorkModeGet722H>()->GetRequest()), true);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722H::CreateInstance()->sp_get_work_mode_->GetCmdKey(), getDifoCtrlData_WorkModeGet);

  GetDifoCtrlClass getDifoCtrlData_TemperatureParamGet(*(std::make_shared<Protocol_TemperatureParamGet722H>()->GetRequest()), false, 1000);
  (*getDifoCtrlData_map_ptr_)
      .emplace(CmdRepository722H::CreateInstance()->sp_temperature_param_get_->GetCmdKey(), getDifoCtrlData_TemperatureParamGet);

  GetDifoCtrlClass getDifoCtrlData_FirmwareGet(*(std::make_shared<Protocol_FirmwareVersionGet722H>()->GetRequest()), true, 1000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722H::CreateInstance()->sp_firmware_version_get_->GetCmdKey(), getDifoCtrlData_FirmwareGet);

  GetDifoCtrlClass getDifoCtrlData_SnGet(*(std::make_shared<Protocol_SnGet722H>()->GetRequest()), true, 1000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722H::CreateInstance()->sp_sn_get_->GetCmdKey(), getDifoCtrlData_SnGet);
}

void DifopVanjee722H::addItem2GetDifoCtrlDataMapPtr(const DeviceCtrl& device_ctrl) {
  if (device_ctrl.cmd_id == 1) {
    if (device_ctrl.cmd_param == 0 || device_ctrl.cmd_param == 1) {
      std::shared_ptr<Params_WorkModeSet722H> params_WorkModeSet722 = std::shared_ptr<Params_WorkModeSet722H>(new Params_WorkModeSet722H());
      params_WorkModeSet722->work_mode_ = device_ctrl.cmd_param == 1 ? 0 : 1;
      GetDifoCtrlClass getDifoCtrlData_WorkModeSet(*(std::make_shared<Protocol_WorkModeSet722H>(params_WorkModeSet722)->SetRequest()), false);

      uint16_t cmd = CmdRepository722H::CreateInstance()->sp_set_work_mode_->GetCmdKey();
      auto it = (*(getDifoCtrlData_map_ptr_)).find(cmd);
      if (it != (*getDifoCtrlData_map_ptr_).end()) {
        ((*getDifoCtrlData_map_ptr_))[cmd] = getDifoCtrlData_WorkModeSet;
      } else {
        ((*getDifoCtrlData_map_ptr_)).emplace(cmd, getDifoCtrlData_WorkModeSet);
      }
    }
  } else if (device_ctrl.cmd_id == 2) {
    if (device_ctrl.cmd_param == 1) {
      // (*getDifoCtrlData_map_ptr_)[CmdRepository722H::CreateInstance()->sp_temperature_param_get_->GetCmdKey()].setStopFlag(false);
    }
  }
}

void DifopVanjee722H::addItem2GetDifoCtrlDataMapPtr(const LidarParameterInterface& lidar_parm) {
  if (lidar_parm.cmd_id == 1) {
    if (lidar_parm.cmd_type == 1) {
      uint8_t work_mode = 0xff;
      JsonParser parser(lidar_parm.data);
      auto json_value = parser.parse();
      if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
        if (obj->has("work_mode")) {
          auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("work_mode"));
          if (result == nullptr || result->value() > 1) {
            return;
          }
          work_mode = static_cast<uint8_t>(result->value());
        }
      }
      if (work_mode > 1) {
        return;
      }
      std::shared_ptr<Params_WorkModeSet722H> params_WorkModeSet722 = std::shared_ptr<Params_WorkModeSet722H>(new Params_WorkModeSet722H());
      params_WorkModeSet722->work_mode_ = work_mode == 1 ? 0 : 1;
      GetDifoCtrlClass getDifoCtrlData_WorkModeSet(*(std::make_shared<Protocol_WorkModeSet722H>(params_WorkModeSet722)->SetRequest()), false);

      uint16_t cmd = CmdRepository722H::CreateInstance()->sp_set_work_mode_->GetCmdKey();
      auto it = (*(getDifoCtrlData_map_ptr_)).find(cmd);
      if (it != (*getDifoCtrlData_map_ptr_).end()) {
        ((*getDifoCtrlData_map_ptr_))[cmd] = getDifoCtrlData_WorkModeSet;
      } else {
        ((*getDifoCtrlData_map_ptr_)).emplace(cmd, getDifoCtrlData_WorkModeSet);
      }
    } else {
      uint16_t cmd = CmdRepository722H::CreateInstance()->sp_get_work_mode_->GetCmdKey();
      if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
        ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
      }
    }
  } else if (lidar_parm.cmd_id == (uint16_t)LidarParam::temperature) {
    uint16_t cmd = CmdRepository722F::CreateInstance()->sp_temperature_param_get_->GetCmdKey();
    ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(true);

    if (lidar_parm.repeat_interval <= 100) {
      ((*getDifoCtrlData_map_ptr_))[cmd].setSendInterval(100);
    } else {
      ((*getDifoCtrlData_map_ptr_))[cmd].setSendInterval(lidar_parm.repeat_interval);
    }

    ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
  } else if (lidar_parm.cmd_id == (uint16_t)LidarParam::firmware_version) {
    uint16_t cmd = CmdRepository722F::CreateInstance()->sp_firmware_version_get_->GetCmdKey();
    if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
      ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
    }
  } else if (lidar_parm.cmd_id == (uint16_t)LidarParam::sn) {
    uint16_t cmd = CmdRepository722F::CreateInstance()->sp_sn_get_->GetCmdKey();
    if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
      ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
    }
  } else {
    ;
  }
}
}  // namespace lidar
}  // namespace vanjee
