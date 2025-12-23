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

#include <vanjee_driver/driver/difop/cmd_class.hpp>

namespace vanjee {
namespace lidar {
class CmdRepository722H {
 public:
  const std::shared_ptr<CmdClass> sp_ld_value_get_ = std::make_shared<CmdClass>(0x05, 0x14);
  const std::shared_ptr<CmdClass> sp_get_work_mode_ = std::make_shared<CmdClass>(0x06, 0x16);
  const std::shared_ptr<CmdClass> sp_set_work_mode_ = std::make_shared<CmdClass>(0x06, 0x17);
  const std::shared_ptr<CmdClass> sp_get_imu_packet_ = std::make_shared<CmdClass>(0x04, 0x08);
  const std::shared_ptr<CmdClass> sp_temperature_param_get_ = std::make_shared<CmdClass>(0x04, 0x02);
  const std::shared_ptr<CmdClass> sp_get_error_code_ = std::make_shared<CmdClass>(0x04, 0x10);
  const std::shared_ptr<CmdClass> sp_firmware_version_get_ = std::make_shared<CmdClass>(0x06, 0x06);
  const std::shared_ptr<CmdClass> sp_sn_get_ = std::make_shared<CmdClass>(0x04, 0x01);

  static CmdRepository722H* CreateInstance() {
    if (p_CmdRepository722h == nullptr)
      p_CmdRepository722h = new CmdRepository722H();

    return p_CmdRepository722h;
  }

 private:
  static CmdRepository722H* p_CmdRepository722h;
  CmdRepository722H() {
  }
  CmdRepository722H(const CmdRepository722H&) = delete;
  CmdRepository722H& operator=(const CmdRepository722H&) = delete;
};

CmdRepository722H* CmdRepository722H::p_CmdRepository722h = nullptr;
}  // namespace lidar
}  // namespace vanjee