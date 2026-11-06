// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rescue_interfaces:msg/ManeuverCommand.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__STRUCT_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__rescue_interfaces__msg__ManeuverCommand __attribute__((deprecated))
#else
# define DEPRECATED__rescue_interfaces__msg__ManeuverCommand __declspec(deprecated)
#endif

namespace rescue_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ManeuverCommand_
{
  using Type = ManeuverCommand_<ContainerAllocator>;

  explicit ManeuverCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command_id = "";
      this->edge_id = "";
      this->route_revision = 0ul;
      this->turn = 0;
      this->stop_at_end = false;
      this->speed_limit_m_s = 0.0f;
      this->distance_m = 0.0f;
      this->lease_ms = 0;
    }
  }

  explicit ManeuverCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    command_id(_alloc),
    edge_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command_id = "";
      this->edge_id = "";
      this->route_revision = 0ul;
      this->turn = 0;
      this->stop_at_end = false;
      this->speed_limit_m_s = 0.0f;
      this->distance_m = 0.0f;
      this->lease_ms = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _command_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_id_type command_id;
  using _edge_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _edge_id_type edge_id;
  using _route_revision_type =
    uint32_t;
  _route_revision_type route_revision;
  using _turn_type =
    uint8_t;
  _turn_type turn;
  using _stop_at_end_type =
    bool;
  _stop_at_end_type stop_at_end;
  using _speed_limit_m_s_type =
    float;
  _speed_limit_m_s_type speed_limit_m_s;
  using _distance_m_type =
    float;
  _distance_m_type distance_m;
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
  Type & set__command_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command_id = _arg;
    return *this;
  }
  Type & set__edge_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->edge_id = _arg;
    return *this;
  }
  Type & set__route_revision(
    const uint32_t & _arg)
  {
    this->route_revision = _arg;
    return *this;
  }
  Type & set__turn(
    const uint8_t & _arg)
  {
    this->turn = _arg;
    return *this;
  }
  Type & set__stop_at_end(
    const bool & _arg)
  {
    this->stop_at_end = _arg;
    return *this;
  }
  Type & set__speed_limit_m_s(
    const float & _arg)
  {
    this->speed_limit_m_s = _arg;
    return *this;
  }
  Type & set__distance_m(
    const float & _arg)
  {
    this->distance_m = _arg;
    return *this;
  }
  Type & set__lease_ms(
    const uint16_t & _arg)
  {
    this->lease_ms = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FOLLOW_LANE =
    0u;
  static constexpr uint8_t TURN_LEFT =
    1u;
  static constexpr uint8_t TURN_RIGHT =
    2u;
  static constexpr uint8_t STRAIGHT =
    3u;

  // pointer types
  using RawPtr =
    rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rescue_interfaces__msg__ManeuverCommand
    std::shared_ptr<rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rescue_interfaces__msg__ManeuverCommand
    std::shared_ptr<rescue_interfaces::msg::ManeuverCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManeuverCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->command_id != other.command_id) {
      return false;
    }
    if (this->edge_id != other.edge_id) {
      return false;
    }
    if (this->route_revision != other.route_revision) {
      return false;
    }
    if (this->turn != other.turn) {
      return false;
    }
    if (this->stop_at_end != other.stop_at_end) {
      return false;
    }
    if (this->speed_limit_m_s != other.speed_limit_m_s) {
      return false;
    }
    if (this->distance_m != other.distance_m) {
      return false;
    }
    if (this->lease_ms != other.lease_ms) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManeuverCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManeuverCommand_

// alias to use template instance with default allocator
using ManeuverCommand =
  rescue_interfaces::msg::ManeuverCommand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManeuverCommand_<ContainerAllocator>::FOLLOW_LANE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManeuverCommand_<ContainerAllocator>::TURN_LEFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManeuverCommand_<ContainerAllocator>::TURN_RIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ManeuverCommand_<ContainerAllocator>::STRAIGHT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__STRUCT_HPP_
