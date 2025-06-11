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

#include <vanjee_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <vanjee_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <vanjee_driver/msg/point_cloud_msg.hpp>
#endif

using namespace vanjee::lidar;

typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

// @brief this function which would return the point cloud to your own
// application from sdk
void pointCloudHandler(std::shared_ptr<PointCloudMsg> msg) {
#if false
  WJ_MSG << "msg: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) << " point cloud size: " << msg->points.size() << WJ_REND;
  // for (auto it = msg->points.begin(); it != msg->points.end(); it++)
  // {
  //   std::cout << std::fixed << std::setprecision(3)
  //             << "(" << it->x << ", " << it->y << ", " << it->z << ", " 
  //             << (int)it->intensity << ", " << (int)it->ring<< ", " << (int)it->timestamp << ")"
  //             << std::endl;
  // }
#endif
}

// @brief this function which would return the imu to your own application from
// sdk
void imuPacketHandler(std::shared_ptr<ImuPacket> msg) {
#if false
  WJ_MSG << "seq: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) << WJ_REND;
  WJ_MSG << "x_acc: " << msg->linear_acce[0] << ", y_acc: " << msg->linear_acce[1]<< ", z_acc: " << msg->linear_acce[2] << WJ_REND;
  WJ_MSG << "angular_voc_x: " << msg->angular_voc[0] << ", angular_voc_y: " << msg->angular_voc[1]<< ", angular_voc_z: " << msg->angular_voc[2] <<WJ_REND;
#endif
}

// @brief this function which would return the laser scan to your own
// application from sdk
void laserScanHandler(std::shared_ptr<ScanData> msg) {
#if false
    WJ_MSG << "seq: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) << "data_size: " << msg->ranges.size() << WJ_REND;
#endif
}

// @brief this function which would return the device control state to your own
// application from sdk
void deviceCtrlStateHandler(std::shared_ptr<DeviceCtrl> msg) {
#if false
  WJ_MSG << "seq: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) << ", cmd_id: " << msg->cmd_id
         << ", cmd_param: " << msg->cmd_param << ", cmd_state: " << (uint16_t)msg->cmd_state << WJ_REND;
#endif
}

class DriverClient {
 public:
  DriverClient(const std::string name) : name_(name) {
  }

  bool init(const WJDriverParam& param) {
    WJ_INFO << "------------------------------------------------------" << WJ_REND;
    WJ_INFO << "                      " << name_ << WJ_REND;
    WJ_INFO << "------------------------------------------------------" << WJ_REND;
    param.print();
    // @brief these two callback,allocatePointCloudMemoryCallback and
    // pointCloudCallback, have to be implemented,and as the parameter of
    // regPointCloudCallback function.
    driver_.regPointCloudCallback(std::bind(&DriverClient::allocatePointCloudMemoryCallback, this),
                                  std::bind(&DriverClient::pointCloudCallback, this, std::placeholders::_1));
    // @brief exceptionCallback have to be implemented,and as the parameter of
    // regExceptionCallback function.
    driver_.regExceptionCallback(std::bind(&DriverClient::exceptionCallback, this, std::placeholders::_1));
    // @brief these two callback,allocateImuPacketMemoryCallback and
    // imuPacketCallback, have to be implemented,and as the parameter of
    // regImuPacketCallback function.s
    driver_.regImuPacketCallback(std::bind(&DriverClient::allocateImuPacketMemoryCallback, this),
                                 std::bind(&DriverClient::imuPacketCallback, this, std::placeholders::_1));
    // @brief these two callback,allocateLaserScanMemoryCallback and
    // laserScanCallback, have to be implemented,and as the parameter of
    // regScanDataCallback function.
    driver_.regScanDataCallback(std::bind(&DriverClient::allocateLaserScanMemoryCallback, this),
                                std::bind(&DriverClient::laserScanCallback, this, std::placeholders::_1));
    // @brief these two callback,allocateDeviceCtrlMemoryCallback and
    // deviceCtrlCallback, have to be implemented,and as the parameter of
    // regDeviceCtrlCallback function.
    driver_.regDeviceCtrlCallback(std::bind(&DriverClient::allocateDeviceCtrlMemoryCallback, this),
                                  std::bind(&DriverClient::deviceCtrlCallback, this, std::placeholders::_1));

    if (!driver_.init(param)) {
      WJ_ERROR << name_ << ": Failed to initialize driver." << WJ_REND;
      return false;
    }
    return true;
  }

  bool start() {
    to_exit_process_ = false;
    point_cloud_thread_ = std::thread(&DriverClient::processPointCloud, this);
    imu_thread_ = std::thread(&DriverClient::processImu, this);
    laser_scan_thread_ = std::thread(&DriverClient::processLaserScan, this);
    device_ctrl_thread_ = std::thread(&DriverClient::processDeviceCtrlState, this);
    driver_.start();
    WJ_DEBUG << name_ << ": Started driver." << WJ_REND;

    return true;
  }

  void stop() {
    driver_.stop();
    to_exit_process_ = true;
    point_cloud_thread_.join();
    imu_thread_.join();
    laser_scan_thread_.join();
    device_ctrl_thread_.join();
  }

 protected:
  // @brief allocate memory for point cloud，sdk would call this callback to get
  // the memory for point cloud storage.
  std::shared_ptr<PointCloudMsg> allocatePointCloudMemoryCallback(void) {
    std::shared_ptr<PointCloudMsg> msg = free_cloud_queue_.pop();
    if (msg.get() != NULL) {
      return msg;
    }

    return std::make_shared<PointCloudMsg>();
  }

  // @brief this callback which would return the point cloud to queue
  void pointCloudCallback(std::shared_ptr<PointCloudMsg> msg) {
    stuffed_cloud_queue_.push(msg);
  }

  // @brief allocate memory for imu，sdk would call this callback to get the
  // memory for imu storage.
  std::shared_ptr<ImuPacket> allocateImuPacketMemoryCallback() {
    std::shared_ptr<ImuPacket> pkt = free_imu_queue_.pop();
    if (pkt.get() != NULL) {
      return pkt;
    }

    return std::make_shared<ImuPacket>();
  }

  // @brief this callback which would return the imu to queue
  void imuPacketCallback(std::shared_ptr<ImuPacket> msg) {
    stuffed_imu_queue_.push(msg);
  }

  // @brief allocate memory for laserScan，sdk would call this callback to get
  // the memory for laserScan storage.
  std::shared_ptr<ScanData> allocateLaserScanMemoryCallback() {
    std::shared_ptr<ScanData> pkt = free_scan_data_queue_.pop();
    if (pkt.get() != NULL) {
      return pkt;
    }

    return std::make_shared<ScanData>();
  }

  // @brief this callback which would return the laserScan to queue
  void laserScanCallback(std::shared_ptr<ScanData> msg) {
    stuffed_scan_data_queue_.push(msg);
  }

  // @brief allocate memory for device control state，sdk would call this callback to get the
  // memory for device contrl state storage.
  std::shared_ptr<DeviceCtrl> allocateDeviceCtrlMemoryCallback() {
    std::shared_ptr<DeviceCtrl> pkt = free_device_ctrl_queue_.pop();
    if (pkt.get() != NULL) {
      return pkt;
    }

    return std::make_shared<DeviceCtrl>();
  }

  // @brief this callback which would return the device control to queue
  void deviceCtrlCallback(std::shared_ptr<DeviceCtrl> msg) {
    stuffed_device_ctrl_queue_.push(msg);
  }

  // @brief this callback which would return the exception prompt information to
  // terminal
  void exceptionCallback(const Error& code) {
    WJ_WARNING << code.toString() << WJ_REND;
  }

  void processPointCloud() {
    while (!to_exit_process_) {
      std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue_.popWait();
      if (msg.get() == NULL) {
        continue;
      }
      pointCloudHandler(msg);
      free_cloud_queue_.push(msg);
    }
  }

  void processImu() {
    while (!to_exit_process_) {
      std::shared_ptr<ImuPacket> msg = stuffed_imu_queue_.popWait();
      if (msg.get() == NULL) {
        continue;
      }
      imuPacketHandler(msg);
      free_imu_queue_.push(msg);
    }
  }

  void processLaserScan() {
    while (!to_exit_process_) {
      std::shared_ptr<ScanData> msg = stuffed_scan_data_queue_.popWait();
      if (msg.get() == NULL) {
        continue;
      }
      laserScanHandler(msg);
      free_scan_data_queue_.push(msg);
    }
  }

  void processDeviceCtrlState(void) {
    while (!to_exit_process_) {
      std::shared_ptr<DeviceCtrl> msg = stuffed_device_ctrl_queue_.popWait();
      if (msg.get() == NULL) {
        continue;
      }
      deviceCtrlStateHandler(msg);
      free_device_ctrl_queue_.push(msg);
    }
  }

 protected:
  std::string name_;
  LidarDriver<PointCloudMsg> driver_;

  SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue_;
  SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue_;
  SyncQueue<std::shared_ptr<ImuPacket>> free_imu_queue_;
  SyncQueue<std::shared_ptr<ImuPacket>> stuffed_imu_queue_;
  SyncQueue<std::shared_ptr<ScanData>> free_scan_data_queue_;
  SyncQueue<std::shared_ptr<ScanData>> stuffed_scan_data_queue_;
  SyncQueue<std::shared_ptr<DeviceCtrl>> free_device_ctrl_queue_;
  SyncQueue<std::shared_ptr<DeviceCtrl>> stuffed_device_ctrl_queue_;

  bool to_exit_process_;
  std::thread point_cloud_thread_;
  std::thread imu_thread_;
  std::thread laser_scan_thread_;
  std::thread device_ctrl_thread_;
};

bool to_exit_keyboard_detect_process_ = false;
void getKeyboard(void) {
  std::string input;
  while (!to_exit_keyboard_detect_process_) {
    std::getline(std::cin, input);
    if (!input.empty()) {
      WJ_INFO << "detect keyboard input: " << input << WJ_REND;
      to_exit_keyboard_detect_process_ = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char* argv[]) {
  WJDriverParam param_left;
  param_left.lidar_type = LidarType::vanjee_720_16;
  param_left.input_type = InputType::ONLINE_LIDAR;
  param_left.input_param.host_msop_port = 3001;
  param_left.input_param.lidar_msop_port = 3333;
  param_left.input_param.host_address = "192.168.2.88";
  param_left.input_param.lidar_address = "192.168.2.86";
  param_left.input_param.group_address = "0.0.0.0";
  param_left.decoder_param.config_from_file = false;
  param_left.decoder_param.angle_path_ver = ".../src/vanjee_lidar_sdk/param/Vanjee_720_16.csv";
  param_left.decoder_param.imu_param_path = ".../src/vanjee_lidar_sdk/param/vanjee_720_imu_param.csv";
  param_left.decoder_param.wait_for_difop = true;
  param_left.decoder_param.imu_enable = 1;          // -1: disable IMU ; 0: enable IMU but calibrate IMU using default
                                                    // parameters ; 1: enable IMU;
  param_left.decoder_param.hide_points_range = "";  // For details about the configuration format, refer to the following
                                                    // documents: "./doc/intro/02_parameter_intro_CN.md"

  DriverClient client_left("LEFT ");
  if (!client_left.init(param_left)) {
    return -1;
  }

  WJDriverParam param_right;
  param_right.lidar_type = LidarType::vanjee_719;
  param_right.input_type = InputType::ONLINE_LIDAR;
  param_right.input_param.host_msop_port = 3002;
  param_right.input_param.lidar_msop_port = 6050;
  param_right.input_param.host_address = "192.168.2.88";
  param_right.input_param.lidar_address = "192.168.2.85";
  param_right.decoder_param.config_from_file = false;
  param_right.decoder_param.wait_for_difop = false;

  DriverClient client_right("RIGHT");
  if (!client_right.init(param_right)) {
    return -1;
  }

  client_left.start();
  client_right.start();

#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  client_left.stop();
  client_right.stop();
#else
  std::thread detect_handle_thread = std::thread(getKeyboard);
  WJ_MSG << "Enter any character and press enter to exit! " << WJ_REND;
  while (true) {
    if (to_exit_keyboard_detect_process_) {
      client_left.stop();
      client_right.stop();
      detect_handle_thread.join();
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}
