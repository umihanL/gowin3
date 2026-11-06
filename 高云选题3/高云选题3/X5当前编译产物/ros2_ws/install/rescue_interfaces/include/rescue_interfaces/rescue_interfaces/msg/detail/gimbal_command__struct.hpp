// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rescue_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rescue_interfaces__msg__GimbalCommand __attribute__((deprecated))
#else
# define DEPRECATED__rescue_interfaces__msg__GimbalCommand __declspec(deprecated)
#endif

namespace rescue_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GimbalCommand_
{
  using Type = GimbalCommand_<ContainerAllocator>;

  explicit GimbalCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->owner = "";
      this->pan_deg = 0.0f;
      this->tilt_deg = 0.0f;
      this->speed_deg_s = 0.0f;
      this->lease_ms = 0;
    }
  }

  explicit GimbalCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    owner(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->owner = "";
      this->pan_deg = 0.0f;
      this->tilt_deg = 0.0f;
      this->speed_deg_s = 0.0f;
      this->lease_ms = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _owner_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _owner_type owner;
  using _pan_deg_type =
    float;
  _pan_deg_type pan_deg;
  using _tilt_deg_type =
    float;
  _tilt_deg_type tilt_deg;
  using _speed_deg_s_type =
    float;
  _speed_deg_s_type speed_deg_s;
  using _lease_ms_type =
    uint16_t;
  _lease_ms_type lease_ms;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__owner(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->owner = _arg;
    return *this;
  }
  Type & set__pan_deg(
    const float & _arg)
  {
    this->pan_deg = _arg;
    return *this;
  }
  Type & set__tilt_deg(
    const float & _arg)
  {
    this->tilt_deg = _arg;
    return *this;
  }
  Type & set__speed_deg_s(
    const float & _arg)
  {
    this->speed_deg_s = _arg;
    return *this;
  }
  Type & set__lease_ms(
    const uint16_t & _arg)
  {
    this->lease_ms = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rescue_interfaces::msg::GimbalCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rescue_interfaces::msg::GimbalCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rescue_interfaces::msg::GimbalCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rescue_interfaces::msg::GimbalCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::GimbalCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::GimbalCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::GimbalCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::GimbalCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rescue_interfaces::msg::GimbalCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rescue_interfaces::msg::GimbalCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rescue_interfaces__msg__GimbalCommand
    std::shared_ptr<rescue_interfaces::msg::GimbalCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rescue_interfaces__msg__GimbalCommand
    std::shared_ptr<rescue_interfaces::msg::GimbalCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->owner != other.owner) {
      return false;
    }
    if (this->pan_deg != other.pan_deg) {
      return false;
    }
    if (this->tilt_deg != other.tilt_deg) {
      return false;
    }
    if (this->speed_deg_s != other.speed_deg_s) {
      return false;
    }
    if (this->lease_ms != other.lease_ms) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalCommand_

// alias to use template instance with default allocator
using GimbalCommand =
  rescue_interfaces::msg::GimbalCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_HPP_
