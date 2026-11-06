// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rescue_interfaces:msg/MotionPermission.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__STRUCT_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__STRUCT_HPP_

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
# define DEPRECATED__rescue_interfaces__msg__MotionPermission __attribute__((deprecated))
#else
# define DEPRECATED__rescue_interfaces__msg__MotionPermission __declspec(deprecated)
#endif

namespace rescue_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotionPermission_
{
  using Type = MotionPermission_<ContainerAllocator>;

  explicit MotionPermission_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->event_id = 0ul;
      this->allow_motion = false;
      this->lease_ms = 0;
      this->reason = "";
    }
  }

  explicit MotionPermission_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->event_id = 0ul;
      this->allow_motion = false;
      this->lease_ms = 0;
      this->reason = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _event_id_type =
    uint32_t;
  _event_id_type event_id;
  using _allow_motion_type =
    bool;
  _allow_motion_type allow_motion;
  using _lease_ms_type =
    uint16_t;
  _lease_ms_type lease_ms;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__event_id(
    const uint32_t & _arg)
  {
    this->event_id = _arg;
    return *this;
  }
  Type & set__allow_motion(
    const bool & _arg)
  {
    this->allow_motion = _arg;
    return *this;
  }
  Type & set__lease_ms(
    const uint16_t & _arg)
  {
    this->lease_ms = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rescue_interfaces::msg::MotionPermission_<ContainerAllocator> *;
  using ConstRawPtr =
    const rescue_interfaces::msg::MotionPermission_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rescue_interfaces::msg::MotionPermission_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rescue_interfaces::msg::MotionPermission_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::MotionPermission_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::MotionPermission_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::MotionPermission_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::MotionPermission_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rescue_interfaces::msg::MotionPermission_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rescue_interfaces::msg::MotionPermission_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rescue_interfaces__msg__MotionPermission
    std::shared_ptr<rescue_interfaces::msg::MotionPermission_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rescue_interfaces__msg__MotionPermission
    std::shared_ptr<rescue_interfaces::msg::MotionPermission_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotionPermission_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->event_id != other.event_id) {
      return false;
    }
    if (this->allow_motion != other.allow_motion) {
      return false;
    }
    if (this->lease_ms != other.lease_ms) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotionPermission_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotionPermission_

// alias to use template instance with default allocator
using MotionPermission =
  rescue_interfaces::msg::MotionPermission_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__STRUCT_HPP_
