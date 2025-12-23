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
class DifopVanjee750 : public DifopBase {
 public:
  explicit DifopVanjee750();
  virtual void initGetDifoCtrlDataMapPtr();
  virtual void singleParsingProcess();
};

inline DifopVanjee750::DifopVanjee750() {
  ProtocolBase pb;
  pb.SetHeader(0xFFFE);
  this->setOrgProtocolBase(pb);
}

void DifopVanjee750::initGetDifoCtrlDataMapPtr() {
  getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16, GetDifoCtrlClass>>();

  std::shared_ptr<std::vector<uint8_t>> content = std::make_shared<std::vector<uint8_t>>(std::initializer_list<uint8_t>{0x01, 0x00});

  GetDifoCtrlClass getDifoCtrlData_ProtocolVersionSet(*(std::make_shared<Protocol_ProtocolVersionSet>()->GetRequest(content)), true);
  (*getDifoCtrlData_map_ptr_)
      .emplace(CmdRepository750::CreateInstance()->set_protocol_version_cmd_id_ptr_->GetCmdKey(), getDifoCtrlData_ProtocolVersionSet);
}

void DifopVanjee750::singleParsingProcess() {
  std::shared_ptr<BufInfo> bufInfo;
  while (!to_exit_recv_) {
    bufInfo = BufInfo_Queue_.popWait(100000);
    if (bufInfo.get() == NULL)
      continue;
    std::vector<uint8> &data = *(bufInfo->Buf);

    if (data.size() < ProtocolBase::FRAME_MIN_LENGTH)
      continue;

    if (!(data[0] == 0xFF && data[1] == 0xFE))
      continue;

    uint32 frameLen = 0;
    if (OrgProtocol.GetByteOrder() == ProtocolBase::DataEndiannessMode::big_endian)
      frameLen = (data[2] << 8) + data[3] + 4;
    else
      frameLen = data[2] + (data[3] << 8) + 4;

    if (frameLen > data.size())
      continue;

    if (!(data[frameLen - 2] == 0xEE && data[frameLen - 1] == 0xEE))
      continue;

    auto protocol = OrgProtocol.CreateNew();
    if (protocol->Parse(data)) {
      frameCb_(protocol);
    }
  }
}

}  // namespace lidar
}  // namespace vanjee