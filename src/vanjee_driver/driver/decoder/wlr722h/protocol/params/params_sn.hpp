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

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee {
namespace lidar {
class Params_Sn722H : public ParamsAbstract {
 public:
  std::string sn_;

 public:
  virtual std::shared_ptr<std::vector<uint8>> GetBytes() {
    std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
    return nullptr;
  }

  virtual void Load(ProtocolBase& protocol) {
    std::array<uint8_t, 20> sn;
    auto buf = protocol.Content.data();
    uint8_t* data = reinterpret_cast<uint8_t*>(buf);
    std::copy(data, data + sn.size(), std::begin(sn));

    for (uint32_t i = 0; i < sn.size(); i++) {
      if (sn[i] == '\0')
        break;
      sn_ += sn[i];
    }
  }
};
}  // namespace lidar
}  // namespace vanjee