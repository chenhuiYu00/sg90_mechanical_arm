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
  unsigned char id, length = 0;
  unsigned char data[6] = {0};
  if (serial_.available()) {
    rx_len_ = static_cast<int>(serial_.available());
    serial_.read(rx_buffer_, rx_len_);
    ROS_INFO("length: %d, read: %x %x", rx_len_, rx_buffer_[0], rx_buffer_[1]);

    // 检查信息头
    if (rx_buffer_[0] != header[0] ||
        rx_buffer_[1] != header[1]) // buf[0] buf[1]
    {
      ROS_WARN("Received message header error!");
      return;
    }

    //读取控制位
    id = rx_buffer_[2]; // buf[2]

    //读取数据长度
    length = rx_buffer_[3]; // buf[3]

    //读取数据
    for (int i = 0; i < 6; i++)
      data[i] = rx_buffer_[4 + i];

  } else {
    return;
  }

  clearRxBuffer();
}

void StRobotHW::write(const ros::Time &time, const ros::Duration &period) {
  static long int i = 0;
  uint8_t id = 0xc0;
  uint8_t data = i % 180;

  unsigned char d[6];
  d[0] = id;
  d[1] = data;

  pack(tx_buffer_, id, &data);
  tx_len_ = sizeof(tx_buffer_);
  try {
    serial_.write(tx_buffer_, tx_len_);
  } catch (serial::PortNotOpenedException &e) {
  }
  i += 1;
  clearTxBuffer();
  ROS_INFO("Write: %d", data);
}

void StRobotHW::setInterface() {
  hardware_interface::JointStateHandle single_state_handle(
      "test_joint", &angle_, &vel_, &effort_);
  joint_state_interface_.registerHandle(single_state_handle);
  hardware_interface::JointHandle joint_handle(
      joint_state_interface_.getHandle("test_joint"), &cmd_);
  position_joint_interface_.registerHandle(joint_handle);
}

void StRobotHW::pack(uint8_t *tx_buffer, uint8_t id,
                     unsigned char data[6]) const {
  memset(tx_buffer, 0, k_frame_length_);
  auto *frame = reinterpret_cast<SerialFrame *>(tx_buffer);

  // 设置消息头
  for (int i = 0; i < 2; i++) {
    frame->header_[i] = header[i];
    frame->end_[0] = ender[0];
  }

  frame->content_->id_ = id;
  frame->content_->length_ = 8;
  memcpy(frame->content_->data_, data, 6);
}
} // namespace steering_engine_hw