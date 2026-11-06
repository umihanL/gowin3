// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rescue_interfaces:msg/StereoPair.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__STEREO_PAIR__STRUCT_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__STEREO_PAIR__STRUCT_HPP_

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
// Member 'left'
// Member 'right'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rescue_interfaces__msg__StereoPair __attribute__((deprecated))
#else
# define DEPRECATED__rescue_interfaces__msg__StereoPair __declspec(deprecated)
#endif

namespace rescue_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StereoPair_
{
  using Type = StereoPair_<ContainerAllocator>;

  explicit StereoPair_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    left(_init),
    right(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sequence = 0ull;
      this->synchronization_verified = false;
      this->skew_s = 0.0;
    }
  }

  explicit StereoPair_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    left(_alloc, _init),
    right(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sequence = 0ull;
      this->synchronization_verified = false;
      this->skew_s = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _sequence_type =
    uint64_t;
  _sequence_type sequence;
  using _left_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _left_type left;
  using _right_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _right_type right;
  using _synchronization_verified_type =
    bool;
  _synchronization_verified_type synchronization_verified;
  using _skew_s_type =
    double;
  _skew_s_type skew_s;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__sequence(
    const uint64_t & _arg)
  {
    this->sequence = _arg;
    return *this;
  }
  Type & set__left(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->left = _arg;
    return *this;
  }
  Type & set__right(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->right = _arg;
    return *this;
  }
  Type & set__synchronization_verified(
    const bool & _arg)
  {
    this->synchronization_verified = _arg;
    return *this;
  }
  Type & set__skew_s(
    const double & _arg)
  {
    this->skew_s = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rescue_interfaces::msg::StereoPair_<ContainerAllocator> *;
  using ConstRawPtr =
    const rescue_interfaces::msg::StereoPair_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rescue_interfaces::msg::StereoPair_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rescue_interfaces::msg::StereoPair_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::StereoPair_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::StereoPair_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::StereoPair_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::StereoPair_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rescue_interfaces::msg::StereoPair_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rescue_interfaces::msg::StereoPair_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rescue_interfaces__msg__StereoPair
    std::shared_ptr<rescue_interfaces::msg::StereoPair_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rescue_interfaces__msg__StereoPair
    std::shared_ptr<rescue_interfaces::msg::StereoPair_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StereoPair_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->sequence != other.sequence) {
      return false;
    }
    if (this->left != other.left) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    if (this->synchronization_verified != other.synchronization_verified) {
      return false;
    }
    if (this->skew_s != other.skew_s) {
      return false;
    }
    return true;
  }
  bool operator!=(const StereoPair_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StereoPair_

// alias to use template instance with default allocator
using StereoPair =
  rescue_interfaces::msg::StereoPair_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__STEREO_PAIR__STRUCT_HPP_
