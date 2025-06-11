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

3. Neither the names of the Vanjee, nor Suteng Innovation Technology, nor the
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

#include <functional>
#include <iostream>
#include <thread>

#include "vanjee_lidar_driver_lib.hpp"

// @brief this callback which would return the point cloud to your own
// application from sdk
void pointCloudCallback(std::shared_ptr<PointCloudMsg> msg) {
#if false
  std::cout << "msg: " << msg->seq << " , timestamp: " << std::to_string(msg->timestamp) << " , point cloud size: " << msg->points.size() << std::endl;
  // for (auto it = msg->points.begin(); it != msg->points.end(); it++)
  // {
  //   std::cout << std::fixed << std::setprecision(3) 
  //             << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")" 
  //             << std::endl;
  // }
#endif
}

// @brief this callback which would return the imu to your own application from
// sdk
void imuPacketCallback(std::shared_ptr<vanjee::lidar::ImuPacket> msg) {
#if false
  std::cout << "timestamp: " << std::to_string(msg->timestamp) << ", seq: " << msg->seq << std::endl;
  std::cout << "x_acc: " << msg->linear_acce[0] << ", y_acc: " << msg->linear_acce[1]<< ", z_acc: " << msg->linear_acce[2] << std::endl;
  std::cout << "angular_voc_x: " << msg->angular_voc[0] << ", angular_voc_y: " << msg->angular_voc[1]<< ", angular_voc_z: " << msg->angular_voc[2] <<std::endl;
#endif
}

// @brief this callback which would return the laserScan to your own application
// from sdk
void laserScanCallback(std::shared_ptr<vanjee::lidar::ScanData> msg) {
#if false
    std::cout << "timestamp: " << std::to_string(msg->timestamp) << ", seq: " << msg->seq << "data_size: " << msg->ranges.size() << std::endl;
#endif
}

// @brief this callback which would return the device ctrl state to your own application
// from sdk
void deviceCtrlStateCallback(std::shared_ptr<vanjee::lidar::DeviceCtrl> msg) {
#if false
  std::cout << "seq: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) << ", cmd_id: " << msg->cmd_id
         << ", cmd_param: " << msg->cmd_param << ", cmd_state: " << (uint16_t)msg->cmd_state << std::endl;
#endif
}

bool to_exit_keyboard_detect_process = false;
bool to_exit_process = false;

void getKeyboard(void) {
  std::string input;
  while (!to_exit_keyboard_detect_process) {
    std::getline(std::cin, input);
    if (!input.empty()) {
      std::cout << "detect keyboard input: " << input << std::endl;
      to_exit_keyboard_detect_process = true;
      to_exit_process = true;

      if (vanjeeLidarDriverLibStop()) {
        std::cout << "vanjee driver exit!" << std::endl;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main() {
  std::thread detect_handle_thread = std::thread(getKeyboard);
  std::string config_path =
      "/home/vanjee/ROS/01vanjee_sdk_dev/00temp_workspace/src/vanjee_lidar_sdk/"
      "config/config.yaml";  // absolute path of config file
  vanjeeLidarDriverLibStart(config_path, pointCloudCallback, imuPacketCallback, laserScanCallback, deviceCtrlStateCallback);
  detect_handle_thread.join();
  return 0;
}
