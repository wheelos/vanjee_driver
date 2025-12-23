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
#ifdef _WIN32
#include <fcntl.h>
#include <windows.h>

#include <iostream>
#include <string>

#include <vanjee_driver/driver/decoder/basic_attr.hpp>
#include <vanjee_driver/driver/input/input.hpp>

namespace vanjee {
namespace lidar {

class InputSerialPort : public Input {
 public:
  InputSerialPort(const WJInputParam &input_param, double sec_to_delay);
  virtual bool init();
  virtual bool start();
  virtual ~InputSerialPort();

 private:
  void recvPacket();

  bool openPort(std::string port_name, uint32_t baud_rate);
  void closePort();
  int sendData(const uint8 *data, uint32 size);
  int waitRead(int32_t timeout);
  virtual int32 send_(uint8 *buf, uint32 size);

 private:
  std::string portName;
  int baudRate;
  HANDLE fd_;
};

InputSerialPort::InputSerialPort(const WJInputParam &input_param, double sec_to_delay) : Input(input_param), fd_(INVALID_HANDLE_VALUE) {
}

inline bool InputSerialPort::init() {
  if (init_flag_) {
    return true;
  }
  bool ret = openPort(input_param_.port_name, input_param_.baud_rate);
  if (!ret) {
    WJ_ERROR << "failed to create serial port!" << WJ_REND;
    goto failMsop;
  }

  init_flag_ = true;
  return true;

failMsop:
  return false;
}

inline bool InputSerialPort::start() {
  if (start_flag_) {
    return true;
  }
  if (!init_flag_) {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputSerialPort::recvPacket, this));
  start_flag_ = true;
  return true;
}

inline InputSerialPort::~InputSerialPort() {
  stop();
  if (fd_ != INVALID_HANDLE_VALUE) {
    closePort();
  }
}

bool InputSerialPort::openPort(std::string port_name, uint32_t baud_rate) {
  closePort();
  fd_ = CreateFile(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
  if (fd_ == INVALID_HANDLE_VALUE) {
    std::cerr << "Failed to open port: " << port_name << std::endl;
    return false;
  }

  DCB dcbSerialParams = {0};
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  dcbSerialParams.BaudRate = baud_rate;
  dcbSerialParams.ByteSize = 8;
  dcbSerialParams.StopBits = ONESTOPBIT;
  dcbSerialParams.Parity = NOPARITY;

  if (!SetCommState(fd_, &dcbSerialParams)) {
    CloseHandle(fd_);
    fd_ = INVALID_HANDLE_VALUE;
    return false;
  }

  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = 1;
  timeouts.ReadTotalTimeoutConstant = 1000;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 50;
  timeouts.WriteTotalTimeoutMultiplier = 1;

  if (!SetCommTimeouts(fd_, &timeouts)) {
    std::cerr << "Error setting timeouts. Error code: " << GetLastError() << std::endl;
    return false;
  }

  return true;
}

void InputSerialPort::closePort() {
  if (fd_ != INVALID_HANDLE_VALUE) {
    CloseHandle(fd_);
    fd_ = INVALID_HANDLE_VALUE;
  }
}

int InputSerialPort::sendData(const uint8 *data, uint32 size) {
  if (fd_ == INVALID_HANDLE_VALUE) {
    std::cerr << "Port not open" << std::endl;
    return -1;
  }
  DWORD bytes_read;
  return WriteFile(fd_, data, size, &bytes_read, nullptr);
}

int32 InputSerialPort::send_(uint8 *buf, uint32 size) {
  int32 ret = -1;
  if (fd_ != INVALID_HANDLE_VALUE) {
    ret = sendData(buf, size);
  }
  return ret;
}

int InputSerialPort::waitRead(int32_t timeout) {
  int ret;
  SetCommMask(fd_, EV_RXCHAR);
  DWORD dwaitResult = WaitForSingleObject(fd_, timeout);
  if (dwaitResult == WAIT_OBJECT_0) {
    ret = 1;
  } else if (dwaitResult == WAIT_TIMEOUT) {
    ret = 0;
  } else {
    ret = -1;
  }

  return ret;
}

inline void InputSerialPort::recvPacket() {
  while (!to_exit_recv_) {
    if (fd_ != INVALID_HANDLE_VALUE) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      DWORD bytesRead;
      ReadFile(fd_, pkt->buf(), pkt->bufSize(), &bytesRead, nullptr);
      if (bytesRead > 0) {
        pkt->setData(0, (int)bytesRead, "0.0.0.0");
        pushPacket(pkt);
      } else if (bytesRead == 0) {
        cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
      }
    }
  }
}

}  // namespace lidar
}  // namespace vanjee
#else
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

#include <vanjee_driver/driver/decoder/basic_attr.hpp>
#include <vanjee_driver/driver/input/input.hpp>
#include <vanjee_driver/driver/input/serial/my_termbits.hpp>

namespace vanjee {
namespace lidar {

class InputSerialPort : public Input {
 public:
  InputSerialPort(const WJInputParam &input_param, double sec_to_delay);
  virtual bool init();
  virtual bool start();
  virtual ~InputSerialPort();

 private:
  void recvPacket();

  bool openPort(std::string port_name, uint32_t baud_rate);
  void closePort();
  ssize_t sendData(const uint8 *data, uint32 size);
  int waitRead(int32_t timeout);
  virtual int32 send_(uint8 *buf, uint32 size);

 private:
  std::string portName;
  int baudRate;
  int fd_;
};

InputSerialPort::InputSerialPort(const WJInputParam &input_param, double sec_to_delay) : Input(input_param), fd_(-1) {
}

inline bool InputSerialPort::init() {
  if (init_flag_) {
    return true;
  }
  bool ret = openPort(input_param_.port_name, input_param_.baud_rate);
  if (!ret) {
    WJ_ERROR << "failed to create serial port!" << WJ_REND;
    goto failMsop;
  }

  init_flag_ = true;
  return true;

failMsop:
  return false;
}

inline bool InputSerialPort::start() {
  if (start_flag_) {
    return true;
  }
  if (!init_flag_) {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputSerialPort::recvPacket, this));
  start_flag_ = true;
  return true;
}

inline InputSerialPort::~InputSerialPort() {
  stop();
  if (fd_ != -1) {
    closePort();
  }
}

bool InputSerialPort::openPort(std::string port_name, uint32_t baud_rate) {
  fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ == -1) {
    std::cerr << "Failed to open port: " << port_name << std::endl;
    return false;
  }

  struct termios2 newtio, oldtio;
  if (ioctl(fd_, TCGETS2, &oldtio) != 0) {
    return false;
  }
  memset(&newtio, 0, sizeof(newtio));
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  newtio.c_cflag |= CS8;

  newtio.c_cflag &= ~PARENB;

  newtio.c_cflag |= BOTHER;
  newtio.c_ispeed = baud_rate;
  newtio.c_ospeed = baud_rate;

  newtio.c_cflag &= ~CSTOPB;

  newtio.c_cc[VMIN] = 0;
  newtio.c_cc[VTIME] = 10;

  tcflush(fd_, TCIFLUSH);
  if (ioctl(fd_, TCSETS2, &newtio) != 0) {
    return false;
  }
  return true;
}

void InputSerialPort::closePort() {
  if (fd_ != -1) {
    close(fd_);
    fd_ = -1;
  }
}

ssize_t InputSerialPort::sendData(const uint8 *data, uint32 size) {
  if (fd_ == -1) {
    std::cerr << "Port not open" << std::endl;
    return -1;
  }
  return write(fd_, data, size);
}

int32 InputSerialPort::send_(uint8 *buf, uint32 size) {
  int32 ret = -1;
  if (fd_ != -1) {
    ret = sendData(buf, size);
  }
  return ret;
}

int InputSerialPort::waitRead(int32_t timeout) {
  int ret;
  fd_set rfds;
  struct timeval tv_timeout;

  FD_ZERO(&rfds);
  FD_SET(fd_, &rfds);

  if (timeout >= 0) {
    tv_timeout.tv_sec = timeout / 1000;
    tv_timeout.tv_usec = (timeout % 1000) * 1000;
    ret = select(fd_ + 1, &rfds, NULL, NULL, &tv_timeout);
  } else {
    ret = select(fd_ + 1, &rfds, NULL, NULL, NULL);
  }
  return ret;
}

inline void InputSerialPort::recvPacket() {
  while (!to_exit_recv_) {
    if (fd_ != -1) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      ssize_t ret = read(fd_, pkt->buf(), pkt->bufSize());
      if (ret > 0) {
        pkt->setData(0, ret, "0.0.0.0");
        pushPacket(pkt);
      } else if (ret == 0) {
        cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }
}

}  // namespace lidar
}  // namespace vanjee
#endif
