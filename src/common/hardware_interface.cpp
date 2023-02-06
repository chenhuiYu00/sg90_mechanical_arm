//
// Created by yuchen on 2023/1/27.
//

#include "steering_engine/common/hardware_interface.h"

namespace steering_engine_hw {
bool StRobotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  //**series**//

  serial::Timeout to = serial::Timeout::simpleTimeout(100); //创建timeout
  serial::parity_t pt = serial::parity_t::parity_none; //创建校验位为0位
  serial::bytesize_t bt = serial::bytesize_t::eightbits; //创建发送字节数为8位
  serial::flowcontrol_t ft =
      serial::flowcontrol_t::flowcontrol_none; //创建数据流控制，不使用
  serial::stopbits_t st = serial::stopbits_t::stopbits_one; //创建终止位为1位

  serial_.setPort("/dev/usbSteering");
  serial_.setBaudrate(115200);
  serial_.setParity(pt);      //设置校验位
  serial_.setBytesize(bt);    //设置发送字节数
  serial_.setFlowcontrol(ft); //设置数据流控制
  serial_.setStopbits(st);    //设置终止位
  serial_.setTimeout(to);

  ROS_INFO("setPort: /dev/usbSteering");

  // setInterface();
  controller_manager_.reset(new controller_manager::ControllerManager(this));

  if (serial_.isOpen())
    return true;
  try {
    serial_.open();
    return true;
  } catch (serial::IOException &e) {
    ROS_ERROR("Cannot open Steering port");
    return false;
  }
}

void StRobotHW::read(const ros::Time &time, const ros::Duration &period) {
  if (serial_.available()) {
    rx_len_ = static_cast<int>(serial_.available());
    serial_.read(rx_buffer_, rx_len_);

    unpack(rx_buffer_);

  } else {
    return;
  }
  clearRxBuffer();
}

void StRobotHW::write(const ros::Time &time, const ros::Duration &period) {
  static long int i = 0;
  uint8_t ctrl = 0xc0;
  uint8_t data[12] = {0};

  pack(tx_buffer_, ctrl, data);
  tx_len_ = sizeof(tx_buffer_);
  try {
    serial_.write(tx_buffer_, tx_len_);
  } catch (serial::PortNotOpenedException &e) {
  }
  i += 4;
  clearTxBuffer();
}

void StRobotHW::setInterface() {
  hardware_interface::JointStateHandle single_state_handle(
      "test_joint", &angle_[0], &vel_[4], &effort_[4]);
  joint_state_interface_.registerHandle(single_state_handle);
  hardware_interface::JointHandle joint_handle(
      joint_state_interface_.getHandle("test_joint"), &cmd_);
  position_joint_interface_.registerHandle(joint_handle);
}

void StRobotHW::pack(unsigned char *tx_buffer, unsigned char ctrl,
                     unsigned char *data) {
  memset(tx_buffer, 0, k_frame_length_);
  auto *frame = reinterpret_cast<SerialFrame *>(tx_buffer);

  // set header
  for (int i = 0; i < 2; i++) {
    frame->header_[i] = header[i];
  }

  // set control
  frame->ctrl_ = ctrl;

  // set data length
  frame->length_ = k_data_length_;

  // set data
  memcpy(frame->data_, data, k_data_length_);

  // set crc
  frame->crc_ = getCrc8(tx_buffer, k_header_length_ + k_ctrl_length_ +
                                       k_length_ + k_data_length_);

  // ser ender
  for (int i = 0; i < 2; i++) {
    frame->ender_[i] = ender[i];
  }
}

void StRobotHW::unpack(std::vector<uint8_t> rx_buffer) {
  uint8_t ctrl, length;

  // check header and ender
  if (rx_buffer[0] != header[0] || rx_buffer[1] != header[1]) // buf[0] buf[1]
  {
    return;
  } else if (rx_buffer[17] != ender[0] || rx_buffer[18] != ender[1]) {
    ROS_WARN("Received message ender error! Want: %x %x Real: %x %x", ender[0],
             ender[1], rx_buffer[17], rx_buffer[18]);
    return;
  }

  //读取控制位
  ctrl = rx_buffer_[2]; // buf[2]

  //读取数据长度
  length = rx_buffer[3]; // buf[3]

  // check crc
  if (rx_buffer[16] !=
      getCrc8(static_cast<unsigned char *>(&rx_buffer[0]), 4 + length)) {
    ROS_WARN("Received message crc check error! Want: %02x Real: %02x",
             getCrc8(static_cast<unsigned char *>(&rx_buffer[0]), 4 + length),
             rx_buffer[16]);
    return;
  }

  //读取数据
  for (int i = 0; i < 5; i++) {
    angle_[i] = rx_buffer[4 + i];
  }
}
} // namespace steering_engine_hw