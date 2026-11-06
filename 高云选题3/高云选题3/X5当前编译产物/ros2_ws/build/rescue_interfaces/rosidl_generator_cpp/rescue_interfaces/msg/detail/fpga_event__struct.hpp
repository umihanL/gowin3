// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rescue_interfaces:msg/FpgaEvent.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__STRUCT_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__STRUCT_HPP_

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
# define DEPRECATED__rescue_interfaces__msg__FpgaEvent __attribute__((deprecated))
#else
# define DEPRECATED__rescue_interfaces__msg__FpgaEvent __declspec(deprecated)
#endif

namespace rescue_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FpgaEvent_
{
  using Type = FpgaEvent_<ContainerAllocator>;

  explicit FpgaEvent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->event_type = 0;
      this->wire_sequence = 0;
      this->event_id = 0ul;
      this->vehicle_stopped = false;
      this->gimbal_settled = false;
      this->measured_angles_valid = false;
      this->pan_deg = 0.0f;
      this->tilt_deg = 0.0f;
      this->acknowledged_sequence = 0;
      this->acknowledgement_result = 0;
    }
  }

  explicit FpgaEvent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->event_type = 0;
      this->wire_sequence = 0;
      this->event_id = 0ul;
      this->vehicle_stopped = false;
      this->gimbal_settled = false;
      this->measured_angles_valid = false;
      this->pan_deg = 0.0f;
      this->tilt_deg = 0.0f;
      this->acknowledged_sequence = 0;
      this->acknowledgement_result = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _event_type_type =
    uint8_t;
  _event_type_type event_type;
  using _wire_sequence_type =
    uint16_t;
  _wire_sequence_type wire_sequence;
  using _event_id_type =
    uint32_t;
  _event_id_type event_id;
  using _vehicle_stopped_type =
    bool;
  _vehicle_stopped_type vehicle_stopped;
  using _gimbal_settled_type =
    bool;
  _gimbal_settled_type gimbal_settled;
  using _measured_angles_valid_type =
    bool;
  _measured_angles_valid_type measured_angles_valid;
  using _pan_deg_type =
    float;
  _pan_deg_type pan_deg;
  using _tilt_deg_type =
    float;
  _tilt_deg_type tilt_deg;
  using _acknowledged_sequence_type =
    uint16_t;
  _acknowledged_sequence_type acknowledged_sequence;
  using _acknowledgement_result_type =
    uint8_t;
  _acknowledgement_result_type acknowledgement_result;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__event_type(
    const uint8_t & _arg)
  {
    this->event_type = _arg;
    return *this;
  }
  Type & set__wire_sequence(
    const uint16_t & _arg)
  {
    this->wire_sequence = _arg;
    return *this;
  }
  Type & set__event_id(
    const uint32_t & _arg)
  {
    this->event_id = _arg;
    return *this;
  }
  Type & set__vehicle_stopped(
    const bool & _arg)
  {
    this->vehicle_stopped = _arg;
    return *this;
  }
  Type & set__gimbal_settled(
    const bool & _arg)
  {
    this->gimbal_settled = _arg;
    return *this;
  }
  Type & set__measured_angles_valid(
    const bool & _arg)
  {
    this->measured_angles_valid = _arg;
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
  Type & set__acknowledged_sequence(
    const uint16_t & _arg)
  {
    this->acknowledged_sequence = _arg;
    return *this;
  }
  Type & set__acknowledgement_result(
    const uint8_t & _arg)
  {
    this->acknowledgement_result = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CROSSWALK =
    16u;
  static constexpr uint8_t CROSSING_COMPLETE =
    17u;
  static constexpr uint8_t STATUS =
    18u;
  static constexpr uint8_t ACK =
    19u;

  // pointer types
  using RawPtr =
    rescue_interfaces::msg::FpgaEvent_<ContainerAllocator> *;
  using ConstRawPtr =
    const rescue_interfaces::msg::FpgaEvent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rescue_interfaces::msg::FpgaEvent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rescue_interfaces::msg::FpgaEvent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::FpgaEvent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::FpgaEvent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::FpgaEvent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::FpgaEvent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rescue_interfaces::msg::FpgaEvent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rescue_interfaces::msg::FpgaEvent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rescue_interfaces__msg__FpgaEvent
    std::shared_ptr<rescue_interfaces::msg::FpgaEvent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rescue_interfaces__msg__FpgaEvent
    std::shared_ptr<rescue_interfaces::msg::FpgaEvent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FpgaEvent_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->event_type != other.event_type) {
      return false;
    }
    if (this->wire_sequence != other.wire_sequence) {
      return false;
    }
    if (this->event_id != other.event_id) {
      return false;
    }
    if (this->vehicle_stopped != other.vehicle_stopped) {
      return false;
    }
    if (this->gimbal_settled != other.gimbal_settled) {
      return false;
    }
    if (this->measured_angles_valid != other.measured_angles_valid) {
      return false;
    }
    if (this->pan_deg != other.pan_deg) {
      return false;
    }
    if (this->tilt_deg != other.tilt_deg) {
      return false;
    }
    if (this->acknowledged_sequence != other.acknowledged_sequence) {
      return false;
    }
    if (this->acknowledgement_result != other.acknowledgement_result) {
      return false;
    }
    return true;
  }
  bool operator!=(const FpgaEvent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FpgaEvent_

// alias to use template instance with default allocator
using FpgaEvent =
  rescue_interfaces::msg::FpgaEvent_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FpgaEvent_<ContainerAllocator>::CROSSWALK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FpgaEvent_<ContainerAllocator>::CROSSING_COMPLETE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FpgaEvent_<ContainerAllocator>::STATUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FpgaEvent_<ContainerAllocator>::ACK;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__STRUCT_HPP_
