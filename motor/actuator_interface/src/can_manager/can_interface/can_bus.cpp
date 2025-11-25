/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// CAN Bus implementation for ROS2 - adapted from unified_hw
//

#include "actuator_interface/can_manager/can_interface/can_bus.h"
#include "actuator_interface/can_manager/can_interface/socketcan.h"

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

namespace can_interface
{
CanBus::CanBus(const std::string& bus_name, int thread_priority)
  : bus_name_(bus_name)
{
  socket_can_ = std::make_unique<SocketCAN>();
  
  // Initialize device at can_device, false for no loop back.
  auto handler = std::bind(&CanBus::frameCallback, this, std::placeholders::_1);
  
  while (!socket_can_->open(bus_name, handler, thread_priority)) {
    RCLCPP_WARN(rclcpp::get_logger("can_bus"), "Failed to open %s, retrying...", bus_name.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(rclcpp::get_logger("can_bus"), "Successfully connected to %s.", bus_name.c_str());
}

void CanBus::read(rclcpp::Time time)
{
  std::lock_guard<std::mutex> guard(mutex_);
  read_buffer_.clear();
}

void CanBus::read()
{
  std::lock_guard<std::mutex> guard(mutex_);
  read_buffer_.clear();
}

void CanBus::write()
{
  // This method is for compatibility, actual writing is done through write(can_frame*)
}

void CanBus::write(can_frame* frame)
{
  if (socket_can_) {
    socket_can_->write(frame);
  }
}

void CanBus::frameCallback(const can_frame& frame)
{
  std::lock_guard<std::mutex> guard(mutex_);
  CanFrameStamp can_frame_stamp{ frame, rclcpp::Clock().now() };
  read_buffer_.push_back(can_frame_stamp);
}

bool CanBus::readBuffer(std::vector<CanFrameStamp>& buffer, int max_frames)
{
  std::lock_guard<std::mutex> guard(mutex_);
  buffer = read_buffer_;
  return !buffer.empty();
}

void CanBus::close()
{
  if (socket_can_) {
    socket_can_->close();
  }
}

}  // namespace can_interface 