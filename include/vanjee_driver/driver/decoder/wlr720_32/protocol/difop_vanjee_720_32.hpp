#pragma once
#include <memory>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"


namespace vanjee
{
  namespace lidar
  {
    class DifopVanjee720_32 : public DifopBase
    {
      public:
        virtual void initGetDifoCtrlDataMapPtr();
    };

    void DifopVanjee720_32::initGetDifoCtrlDataMapPtr()
    {
      getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16,GetDifoCtrlClass>>(); 

      GetDifoCtrlClass getDifoCtrlData_LdAngleGet(*(std::make_shared<Protocol_LDAngleGet720_32>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720_32::CreateInstance()->sp_ld_angle_get_->GetCmdKey(),getDifoCtrlData_LdAngleGet);
    }
  }
}
