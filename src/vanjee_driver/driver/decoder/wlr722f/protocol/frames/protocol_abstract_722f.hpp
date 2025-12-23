#pragma once

#include <memory>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"

namespace vanjee {
namespace lidar {
class ProtocolAbstract722F : public ProtocolAbstract {
 public:
  ProtocolAbstract722F(const std::shared_ptr<CmdClass> sp_cmd, std::shared_ptr<ParamsAbstract> content, uint8 checkType = 1, uint8 type = 0)
      : ProtocolAbstract(checkType, type, {0x00, 0x0C}, sp_cmd, content) {
  }

  virtual bool Load(ProtocolBase& protocol) {
    CheckType = protocol.CheckType;
    Type = protocol.Type;
    Sp_Cmd.reset(new CmdClass(protocol.MainCmd, protocol.SubCmd));
    Params->Load(protocol);
    return true;
  }

  virtual std::shared_ptr<std::vector<uint8>> GetRequest(std::shared_ptr<std::vector<uint8>> content = nullptr) override {
    if (content == nullptr) {
      const uint8 arr[] = {0x00, 0x00, 0x00, 0x00};
      content = std::make_shared<std::vector<uint8>>();
      content->insert(content->end(), arr, arr + sizeof(arr) / sizeof(uint8));
    }

    return (std::make_shared<ProtocolBase>(CheckType, Type, DeviceType, Sp_Cmd->MainCmd, Sp_Cmd->SubCmd, *content,
                                           ProtocolBase::DataEndiannessMode::little_endian, 0xFFAA))
        ->GetBytes(ProtocolBase::ProtocolVersionDifop::version_v1);
  }

  virtual std::shared_ptr<std::vector<uint8>> SetRequest() override {
    return (std::make_shared<ProtocolBase>(CheckType, Type, DeviceType, Sp_Cmd->MainCmd, Sp_Cmd->SubCmd, *Params->GetBytes(),
                                           ProtocolBase::DataEndiannessMode::little_endian, 0xFFAA))
        ->GetBytes(ProtocolBase::ProtocolVersionDifop::version_v1);
  }
};
}  // namespace lidar
}  // namespace vanjee