// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rescue_interfaces:msg/FpgaEvent.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__BUILDER_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rescue_interfaces/msg/detail/fpga_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rescue_interfaces
{

namespace msg
{

namespace builder
{

class Init_FpgaEvent_acknowledgement_result
{
public:
  explicit Init_FpgaEvent_acknowledgement_result(::rescue_interfaces::msg::FpgaEvent & msg)
  : msg_(msg)
  {}
  ::rescue_interfaces::msg::FpgaEvent acknowledgement_result(::rescue_interfaces::msg::FpgaEvent::_acknowledgement_result_type arg)
  {
    msg_.acknowledgement_result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

class Init_FpgaEvent_acknowledged_sequence
{
public:
  explicit Init_FpgaEvent_acknowledged_sequence(::rescue_interfaces::msg::FpgaEvent & msg)
  : msg_(msg)
  {}
  Init_FpgaEvent_acknowledgement_result acknowledged_sequence(::rescue_interfaces::msg::FpgaEvent::_acknowledged_sequence_type arg)
  {
    msg_.acknowledged_sequence = std::move(arg);
    return Init_FpgaEvent_acknowledgement_result(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

class Init_FpgaEvent_tilt_deg
{
public:
  explicit Init_FpgaEvent_tilt_deg(::rescue_interfaces::msg::FpgaEvent & msg)
  : msg_(msg)
  {}
  Init_FpgaEvent_acknowledged_sequence tilt_deg(::rescue_interfaces::msg::FpgaEvent::_tilt_deg_type arg)
  {
    msg_.tilt_deg = std::move(arg);
    return Init_FpgaEvent_acknowledged_sequence(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

class Init_FpgaEvent_pan_deg
{
public:
  explicit Init_FpgaEvent_pan_deg(::rescue_interfaces::msg::FpgaEvent & msg)
  : msg_(msg)
  {}
  Init_FpgaEvent_tilt_deg pan_deg(::rescue_interfaces::msg::FpgaEvent::_pan_deg_type arg)
  {
    msg_.pan_deg = std::move(arg);
    return Init_FpgaEvent_tilt_deg(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

class Init_FpgaEvent_measured_angles_valid
{
public:
  explicit Init_FpgaEvent_measured_angles_valid(::rescue_interfaces::msg::FpgaEvent & msg)
  : msg_(msg)
  {}
  Init_FpgaEvent_pan_deg measured_angles_valid(::rescue_interfaces::msg::FpgaEvent::_measured_angles_valid_type arg)
  {
    msg_.measured_angles_valid = std::move(arg);
    return Init_FpgaEvent_pan_deg(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

class Init_FpgaEvent_gimbal_settled
{
public:
  explicit Init_FpgaEvent_gimbal_settled(::rescue_interfaces::msg::FpgaEvent & msg)
  : msg_(msg)
  {}
  Init_FpgaEvent_measured_angles_valid gimbal_settled(::rescue_interfaces::msg::FpgaEvent::_gimbal_settled_type arg)
  {
    msg_.gimbal_settled = std::move(arg);
    return Init_FpgaEvent_measured_angles_valid(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

class Init_FpgaEvent_vehicle_stopped
{
public:
  explicit Init_FpgaEvent_vehicle_stopped(::rescue_interfaces::msg::FpgaEvent & msg)
  : msg_(msg)
  {}
  Init_FpgaEvent_gimbal_settled vehicle_stopped(::rescue_interfaces::msg::FpgaEvent::_vehicle_stopped_type arg)
  {
    msg_.vehicle_stopped = std::move(arg);
    return Init_FpgaEvent_gimbal_settled(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

class Init_FpgaEvent_event_id
{
public:
  explicit Init_FpgaEvent_event_id(::rescue_interfaces::msg::FpgaEvent & msg)
  : msg_(msg)
  {}
  Init_FpgaEvent_vehicle_stopped event_id(::rescue_interfaces::msg::FpgaEvent::_event_id_type arg)
  {
    msg_.event_id = std::move(arg);
    return Init_FpgaEvent_vehicle_stopped(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

class Init_FpgaEvent_wire_sequence
{
public:
  explicit Init_FpgaEvent_wire_sequence(::rescue_interfaces::msg::FpgaEvent & msg)
  : msg_(msg)
  {}
  Init_FpgaEvent_event_id wire_sequence(::rescue_interfaces::msg::FpgaEvent::_wire_sequence_type arg)
  {
    msg_.wire_sequence = std::move(arg);
    return Init_FpgaEvent_event_id(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

class Init_FpgaEvent_event_type
{
public:
  explicit Init_FpgaEvent_event_type(::rescue_interfaces::msg::FpgaEvent & msg)
  : msg_(msg)
  {}
  Init_FpgaEvent_wire_sequence event_type(::rescue_interfaces::msg::FpgaEvent::_event_type_type arg)
  {
    msg_.event_type = std::move(arg);
    return Init_FpgaEvent_wire_sequence(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

class Init_FpgaEvent_header
{
public:
  Init_FpgaEvent_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FpgaEvent_event_type header(::rescue_interfaces::msg::FpgaEvent::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FpgaEvent_event_type(msg_);
  }

private:
  ::rescue_interfaces::msg::FpgaEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rescue_interfaces::msg::FpgaEvent>()
{
  return rescue_interfaces::msg::builder::Init_FpgaEvent_header();
}

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__BUILDER_HPP_
