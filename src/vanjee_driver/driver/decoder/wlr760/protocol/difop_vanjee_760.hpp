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
class DifopVanjee760 : public DifopBase {
 private:
  std::thread get_heartbeat_flag_thread_;
  bool heartbeat_enable_;
  uint32_t pre_send_cnt_;
  uint32_t heartbeat_cnt_;

 public:
  void processHeartBeat();
  virtual void initGetDifoCtrlDataMapPtr();

  DifopVanjee760() : heartbeat_enable_(true), pre_send_cnt_(0), heartbeat_cnt_(0) {
  }
  ~DifopVanjee760() {
    heartbeat_enable_ = false;
    get_heartbeat_flag_thread_.join();
  }
};

void DifopVanjee760::initGetDifoCtrlDataMapPtr() {
  getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16, GetDifoCtrlClass>>();

  std::shared_ptr<std::vector<uint8_t>> content = std::make_shared<std::vector<uint8_t>>(std::initializer_list<uint8_t>{0x01, 0x00});

  GetDifoCtrlClass getDifoCtrlData_ProtocolVersionSet(*(std::make_shared<Protocol_ProtocolVersionSet760>()->GetRequest(content)), true);
  (*getDifoCtrlData_map_ptr_)
      .emplace(CmdRepository760::CreateInstance()->set_protocol_version_cmd_id_ptr_->GetCmdKey(), getDifoCtrlData_ProtocolVersionSet);

  GetDifoCtrlClass getDifoCtrlData_Handshake(*(std::make_shared<Protocol_HeartBeat760>()->GetRequest()), false, 2000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository760::CreateInstance()->sp_heart_beat_->GetCmdKey(), getDifoCtrlData_Handshake);

  get_heartbeat_flag_thread_ = std::thread(std::bind(&DifopVanjee760::processHeartBeat, this));
}

void DifopVanjee760::processHeartBeat() {
  bool handshark_flag = true;
  while (heartbeat_enable_) {
    this->mtx.lock();
    if ((*getDifoCtrlData_map_ptr_)[CmdRepository760::CreateInstance()->sp_heart_beat_->GetCmdKey()].getStopFlag()) {
      if (handshark_flag) {
        pre_send_cnt_ = 1;
        if ((*getDifoCtrlData_map_ptr_).count(0x0001) >= 1) {
          (*getDifoCtrlData_map_ptr_)[0x0001].setStopFlag(true);
          (*getDifoCtrlData_map_ptr_).erase(0x0001);
        }

        std::shared_ptr<Param_HeartBeat760> param_set = std::shared_ptr<Param_HeartBeat760>(new Param_HeartBeat760());
        param_set->heartbeat_state_ = 2;
        param_set->heartbeat_cnt_ = 1;
        GetDifoCtrlClass get_difo_ctrl_class_heartbeat_set(*(std::make_shared<Protocol_HeartBeat760>(param_set)->SetRequest()), false, 2000);
        get_difo_ctrl_class_heartbeat_set.setSendCnt(pre_send_cnt_);

        (*getDifoCtrlData_map_ptr_).emplace(0x0001, get_difo_ctrl_class_heartbeat_set);
        handshark_flag = false;
      } else {
        if (heartbeat_cnt_ > 0 && (*getDifoCtrlData_map_ptr_)[0x0001].getSendCnt() != pre_send_cnt_) {
          pre_send_cnt_ = (*getDifoCtrlData_map_ptr_)[0x0001].getSendCnt();

          if ((*getDifoCtrlData_map_ptr_).count(0x0001) >= 1) {
            (*getDifoCtrlData_map_ptr_)[0x0001].setStopFlag(true);
            (*getDifoCtrlData_map_ptr_).erase(0x0001);
          }

          std::shared_ptr<Param_HeartBeat760> param_set = std::shared_ptr<Param_HeartBeat760>(new Param_HeartBeat760());
          param_set->heartbeat_state_ = 2;
          param_set->heartbeat_cnt_ = (++heartbeat_cnt_) == 0 ? (++heartbeat_cnt_) : heartbeat_cnt_;
          GetDifoCtrlClass get_difo_ctrl_class_heartbeat_set(*(std::make_shared<Protocol_HeartBeat760>(param_set)->SetRequest()), false, 2000);
          get_difo_ctrl_class_heartbeat_set.setSendCnt(pre_send_cnt_);
          (*getDifoCtrlData_map_ptr_).emplace(0x0001, get_difo_ctrl_class_heartbeat_set);
        }
      }
    } else {
      if (!handshark_flag) {
        handshark_flag = true;
      }
    }
    this->mtx.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}
}  // namespace lidar
}  // namespace vanjee