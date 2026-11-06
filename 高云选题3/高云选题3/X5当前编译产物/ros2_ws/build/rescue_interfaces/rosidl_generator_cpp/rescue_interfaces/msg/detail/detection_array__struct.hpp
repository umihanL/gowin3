// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rescue_interfaces:msg/DetectionArray.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__DETECTION_ARRAY__STRUCT_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__DETECTION_ARRAY__STRUCT_HPP_

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
// Member 'detections'
#include "rescue_interfaces/msg/detail/detection__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rescue_interfaces__msg__DetectionArray __attribute__((deprecated))
#else
# define DEPRECATED__rescue_interfaces__msg__DetectionArray __declspec(deprecated)
#endif

namespace rescue_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectionArray_
{
  using Type = DetectionArray_<ContainerAllocator>;

  explicit DetectionArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sequence = 0ull;
      this->width = 0ul;
      this->height = 0ul;
      this->rectified = false;
      this->detector_available = false;
      this->status = "";
      this->stereo_calibration_id = "";
    }
  }

  explicit DetectionArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    status(_alloc),
    stereo_calibration_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sequence = 0ull;
      this->width = 0ul;
      this->height = 0ul;
      this->rectified = false;
      this->detector_available = false;
      this->status = "";
      this->stereo_calibration_id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _sequence_type =
    uint64_t;
  _sequence_type sequence;
  using _width_type =
    uint32_t;
  _width_type width;
  using _height_type =
    uint32_t;
  _height_type height;
  using _rectified_type =
    bool;
  _rectified_type rectified;
  using _detector_available_type =
    bool;
  _detector_available_type detector_available;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _stereo_calibration_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _stereo_calibration_id_type stereo_calibration_id;
  using _detections_type =
    std::vector<rescue_interfaces::msg::Detection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rescue_interfaces::msg::Detection_<ContainerAllocator>>>;
  _detections_type detections;

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
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__rectified(
    const bool & _arg)
  {
    this->rectified = _arg;
    return *this;
  }
  Type & set__detector_available(
    const bool & _arg)
  {
    this->detector_available = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__stereo_calibration_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->stereo_calibration_id = _arg;
    return *this;
  }
  Type & set__detections(
    const std::vector<rescue_interfaces::msg::Detection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rescue_interfaces::msg::Detection_<ContainerAllocator>>> & _arg)
  {
    this->detections = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rescue_interfaces::msg::DetectionArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const rescue_interfaces::msg::DetectionArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rescue_interfaces::msg::DetectionArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rescue_interfaces::msg::DetectionArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::DetectionArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::DetectionArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rescue_interfaces::msg::DetectionArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rescue_interfaces::msg::DetectionArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rescue_interfaces::msg::DetectionArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rescue_interfaces::msg::DetectionArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rescue_interfaces__msg__DetectionArray
    std::shared_ptr<rescue_interfaces::msg::DetectionArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rescue_interfaces__msg__DetectionArray
    std::shared_ptr<rescue_interfaces::msg::DetectionArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectionArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->sequence != other.sequence) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->rectified != other.rectified) {
      return false;
    }
    if (this->detector_available != other.detector_available) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->stereo_calibration_id != other.stereo_calibration_id) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectionArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectionArray_

// alias to use template instance with default allocator
using DetectionArray =
  rescue_interfaces::msg::DetectionArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__DETECTION_ARRAY__STRUCT_HPP_
