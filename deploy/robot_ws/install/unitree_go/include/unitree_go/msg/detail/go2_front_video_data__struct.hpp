// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/Go2FrontVideoData.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__Go2FrontVideoData __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__Go2FrontVideoData __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Go2FrontVideoData_
{
  using Type = Go2FrontVideoData_<ContainerAllocator>;

  explicit Go2FrontVideoData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_frame = 0ull;
    }
  }

  explicit Go2FrontVideoData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_frame = 0ull;
    }
  }

  // field types and members
  using _time_frame_type =
    uint64_t;
  _time_frame_type time_frame;
  using _video720p_type =
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other>;
  _video720p_type video720p;
  using _video360p_type =
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other>;
  _video360p_type video360p;
  using _video180p_type =
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other>;
  _video180p_type video180p;

  // setters for named parameter idiom
  Type & set__time_frame(
    const uint64_t & _arg)
  {
    this->time_frame = _arg;
    return *this;
  }
  Type & set__video720p(
    const std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> & _arg)
  {
    this->video720p = _arg;
    return *this;
  }
  Type & set__video360p(
    const std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> & _arg)
  {
    this->video360p = _arg;
    return *this;
  }
  Type & set__video180p(
    const std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> & _arg)
  {
    this->video180p = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::Go2FrontVideoData_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::Go2FrontVideoData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::Go2FrontVideoData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::Go2FrontVideoData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::Go2FrontVideoData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::Go2FrontVideoData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::Go2FrontVideoData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::Go2FrontVideoData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::Go2FrontVideoData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::Go2FrontVideoData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__Go2FrontVideoData
    std::shared_ptr<unitree_go::msg::Go2FrontVideoData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__Go2FrontVideoData
    std::shared_ptr<unitree_go::msg::Go2FrontVideoData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Go2FrontVideoData_ & other) const
  {
    if (this->time_frame != other.time_frame) {
      return false;
    }
    if (this->video720p != other.video720p) {
      return false;
    }
    if (this->video360p != other.video360p) {
      return false;
    }
    if (this->video180p != other.video180p) {
      return false;
    }
    return true;
  }
  bool operator!=(const Go2FrontVideoData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Go2FrontVideoData_

// alias to use template instance with default allocator
using Go2FrontVideoData =
  unitree_go::msg::Go2FrontVideoData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__STRUCT_HPP_
