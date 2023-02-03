//
// Created by yuchen on 2023/1/27.
//

#pragma once

#include <memory>
#include <std_msgs/String.h>
#include <string>
#include <unordered_map>
#include <vector>

// ROS
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <urdf/model.h>

// ROS control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <transmission_interface/transmission_interface_loader.h>

namespace steering_engine_hw {
class StRobotHW : public hardware_interface::RobotHW {
public:
  StRobotHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load
   * urdf of robot. Set up transmission and joint limit. Set configuration of
   * series.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
  /** \brief Comunicate with hardware. Get datas, status of robot.
   *
   * Call @ref rm_hw::CanBus::read(). Check whether temperature of actuator is
   * too high and whether actuator is offline. Propagate actuator state to joint
   * state for the stored transmission. Set all cmd to zero to avoid crazy soft
   * limit oscillation when not controller loaded(all controllers update after
   * read()).
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time &time, const ros::Duration &period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref
   * rm_hw::CanBus::write(). Publish actuator current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time &time, const ros::Duration &period) override;

  void pack(uint8_t *tx_buffer, uint8_t id, unsigned char data[6]) const;
  void setInterface();

  void clearTxBuffer() {
    for (int i = 0; i < 10; i++)
      tx_buffer_[i] = 0;
    tx_len_ = 0;
  }
  void clearRxBuffer() {
    for (int i = 0; i < rx_buffer_.size(); i++)
      rx_buffer_[i] = 0;
    rx_len_ = 0;
  }
  unsigned char getCrc8(unsigned char *ptr, unsigned short len) {
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while (len--) {
      crc ^= *ptr++;
      for (i = 0; i < 8; i++) {
        if (crc & 0x01)
          crc = (crc >> 1) ^ 0x8C;
        else
          crc >>= 1;
      }
    }
    return crc;
  }

private:
  double angle_, vel_, effort_;
  double cmd_;
  serial::Serial serial_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  int rx_len_;
  std::vector<uint8_t> rx_buffer_;
  uint8_t tx_buffer_[12];
  int tx_len_;
  const int k_frame_length_ = 12, k_header_length_ = 2, k_data_length_8,
            k_tail_length_ = 2;

  //通信协议常量
  const unsigned char header[2] = {0x55, 0xaa};
  const unsigned char ender[2] = {0x0d, 0x0a};
};

struct DataStruct {
  unsigned char id_;
  unsigned char length_;
  unsigned char data_[6];
};
struct SerialFrame {
  unsigned char header_[2];
  DataStruct content_[8];
  unsigned char end_[2];
};
} // namespace steering_engine_hw
