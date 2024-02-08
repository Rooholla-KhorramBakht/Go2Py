// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/AudioData.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__AUDIO_DATA__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__AUDIO_DATA__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__AudioData __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__AudioData __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AudioData_
{
  using Type = AudioData_<ContainerAllocator>;

  explicit AudioData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_frame = 0ull;
    }
  }

  explicit AudioData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  using _data_type =
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__time_frame(
    const uint64_t & _arg)
  {
    this->time_frame = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::AudioData_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::AudioData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::AudioData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::AudioData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::AudioData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::AudioData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::AudioData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::AudioData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::AudioData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::AudioData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__AudioData
    std::shared_ptr<unitree_go::msg::AudioData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__AudioData
    std::shared_ptr<unitree_go::msg::AudioData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AudioData_ & other) const
  {
    if (this->time_frame != other.time_frame) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const AudioData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AudioData_

// alias to use template instance with default allocator
using AudioData =
  unitree_go::msg::AudioData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__AUDIO_DATA__STRUCT_HPP_
