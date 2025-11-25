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
// CAN Bus interface for ROS2 - adapted from unified_hw
//

#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <functional>
#include <memory>

namespace can_interface
{
struct CanFrameStamp
{
  can_frame frame;
  rclcpp::Time stamp;
  
  CanFrameStamp() : stamp(rclcpp::Clock().now()) {}
  CanFrameStamp(const can_frame& f, const rclcpp::Time& t) : frame(f), stamp(t) {}
};

class CanBus
{
public:
  /** \brief
   * Initialize device at can_device, retry if fail. Set up header of CAN frame.
   *
   * \param bus_name Bus's name(example: can0).
   * \param thread_priority Thread priority for CAN operations.
   */
  CanBus(const std::string& bus_name, int thread_priority);
  
  /** \brief Read active data from read_buffer_ to data_ptr_, such as position, velocity, torque and so on. Clear
   * read_buffer_ after reading.
   *
   * \param time ROS time, but it doesn't be used.
   */
  void read(rclcpp::Time time);
  void read();

  /** \brief Write commands to can bus.
   *
   */
  void write();

  void write(can_frame* frame);

  std::string getName(){return bus_name_;}
  std::string getDeviceName(){return bus_name_;}

  std::vector<CanFrameStamp> getReadBuffer(){std::lock_guard<std::mutex> guard(mutex_); return read_buffer_;}
  bool readBuffer(std::vector<CanFrameStamp>& buffer, int max_frames = 100);
  
  void close();
  
private:
  /** \brief This function will be called when CAN bus receive message. It push frame which received into a vector: read_buffer_.
   *
   * @param frame The frame which socketcan receive.
   */
  void frameCallback(const can_frame& frame);

  const std::string bus_name_;
  std::vector<CanFrameStamp> read_buffer_;

  mutable std::mutex mutex_;
  
  // SocketCAN will be handled separately
  std::unique_ptr<class SocketCAN> socket_can_;
};
}  // namespace can_interface 