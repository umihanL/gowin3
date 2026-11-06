// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rescue_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position_m'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rescue_interfaces__msg__Detection __attribute__((deprecated))
#else
# define DEPRECATED__rescue_interfaces__msg__Detection __declspec(deprecated)
#endif

namespace rescue_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection_
{
  using Type = Detection_<ContainerAllocator>;

  explicit Detection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position_m(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0l;
      this->class_name = "";
      this->confidence = 0.0f;
      std::fill<typename std::array<float, 4>::iterator, float>(this->bbox_xyxy.begin(), this->bbox_xyxy.end(), 0.0f);
      this->depth_valid = false;
      this->depth_m = 0.0f;
      this->depth_reason = "";
    }
  }

  explicit Detection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : class_name(_alloc),
    bbox_xyxy(_alloc),
    position_m(_alloc, _init),
    depth_reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0l;
      this->class_name = "";
      this->confidence = 0.0f;
      std::fill<typename std::array<float, 4>::iterator, float>(this->bbox_xyxy.begin(), this->bbox_xyxy.end(), 0.0f);
      this->depth_valid = false;
      this->depth_m = 0.0f;
      this->depth_reason = "";
    }
  }

  // field types and members
  using _class_id_type =
    int32_t;
  _class_id_type class_id;
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _bbox_xyxy_type =
    std::array<float, 4>;
  _bbox_xyxy_type bbox_xyxy;
  using _depth_valid_type =
    bool;
  _depth_valid_type depth_valid;
  using _position_m_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_m_type position_m;
  using _depth_m_type =
    float;
  _depth_m_type depth_m;
  using _depth_reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _depth_reason_type depth_reason;

  // setters for named parameter idiom
  Type & set__class_id(
    const int32_t & _arg)
  {
    this->class_id = _arg;
    return *this;
  }
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__bbox_xyxy(
    const std::array<float, 4> & _arg)
  {
    this->bbox_xyxy = _arg;
    return *this;
  }
  Type & set__depth_valid(
    const bool & _arg)
  {
    this->depth_valid = _arg;
    return *this;
  }
  Type & set__position_m(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position_m = _arg;
    return *this;
  }
  Type & set__depth_m(
    const float & _arg)
  {
    this->depth_m = _arg;
    return *this;
  }
  Type & set__depth_reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->depth_reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rescue_interfaces::msg::Detection_<ContainerAllocator> *;
  using ConstRawPtr =
    const rescue_interfaces::msg::Detection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rescue_interfaces::msg::Detection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rescue_interfaces::msg::Detection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::Detection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::Detection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::Detection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::Detection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rescue_interfaces::msg::Detection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rescue_interfaces::msg::Detection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rescue_interfaces__msg__Detection
    std::shared_ptr<rescue_interfaces::msg::Detection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rescue_interfaces__msg__Detection
    std::shared_ptr<rescue_interfaces::msg::Detection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection_ & other) const
  {
    if (this->class_id != other.class_id) {
      return false;
    }
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->bbox_xyxy != other.bbox_xyxy) {
      return false;
    }
    if (this->depth_valid != other.depth_valid) {
      return false;
    }
    if (this->position_m != other.position_m) {
      return false;
    }
    if (this->depth_m != other.depth_m) {
      return false;
    }
    if (this->depth_reason != other.depth_reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection_

// alias to use template instance with default allocator
using Detection =
  rescue_interfaces::msg::Detection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_HPP_
