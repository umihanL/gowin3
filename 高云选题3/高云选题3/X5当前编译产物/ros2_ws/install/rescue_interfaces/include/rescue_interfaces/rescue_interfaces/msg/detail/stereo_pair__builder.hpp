// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rescue_interfaces:msg/StereoPair.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__STEREO_PAIR__BUILDER_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__STEREO_PAIR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rescue_interfaces/msg/detail/stereo_pair__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rescue_interfaces
{

namespace msg
{

namespace builder
{

class Init_StereoPair_skew_s
{
public:
  explicit Init_StereoPair_skew_s(::rescue_interfaces::msg::StereoPair & msg)
  : msg_(msg)
  {}
  ::rescue_interfaces::msg::StereoPair skew_s(::rescue_interfaces::msg::StereoPair::_skew_s_type arg)
  {
    msg_.skew_s = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rescue_interfaces::msg::StereoPair msg_;
};

class Init_StereoPair_synchronization_verified
{
public:
  explicit Init_StereoPair_synchronization_verified(::rescue_interfaces::msg::StereoPair & msg)
  : msg_(msg)
  {}
  Init_StereoPair_skew_s synchronization_verified(::rescue_interfaces::msg::StereoPair::_synchronization_verified_type arg)
  {
    msg_.synchronization_verified = std::move(arg);
    return Init_StereoPair_skew_s(msg_);
  }

private:
  ::rescue_interfaces::msg::StereoPair msg_;
};

class Init_StereoPair_right
{
public:
  explicit Init_StereoPair_right(::rescue_interfaces::msg::StereoPair & msg)
  : msg_(msg)
  {}
  Init_StereoPair_synchronization_verified right(::rescue_interfaces::msg::StereoPair::_right_type arg)
  {
    msg_.right = std::move(arg);
    return Init_StereoPair_synchronization_verified(msg_);
  }

private:
  ::rescue_interfaces::msg::StereoPair msg_;
};

class Init_StereoPair_left
{
public:
  explicit Init_StereoPair_left(::rescue_interfaces::msg::StereoPair & msg)
  : msg_(msg)
  {}
  Init_StereoPair_right left(::rescue_interfaces::msg::StereoPair::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_StereoPair_right(msg_);
  }

private:
  ::rescue_interfaces::msg::StereoPair msg_;
};

class Init_StereoPair_sequence
{
public:
  explicit Init_StereoPair_sequence(::rescue_interfaces::msg::StereoPair & msg)
  : msg_(msg)
  {}
  Init_StereoPair_left sequence(::rescue_interfaces::msg::StereoPair::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return Init_StereoPair_left(msg_);
  }

private:
  ::rescue_interfaces::msg::StereoPair msg_;
};

class Init_StereoPair_header
{
public:
  Init_StereoPair_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StereoPair_sequence header(::rescue_interfaces::msg::StereoPair::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StereoPair_sequence(msg_);
  }

private:
  ::rescue_interfaces::msg::StereoPair msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rescue_interfaces::msg::StereoPair>()
{
  return rescue_interfaces::msg::builder::Init_StereoPair_header();
}

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__STEREO_PAIR__BUILDER_HPP_
