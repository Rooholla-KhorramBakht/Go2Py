// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/IMUState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__IMU_STATE__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__IMU_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__IMUState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__IMUState __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IMUState_
{
  using Type = IMUState_<ContainerAllocator>;

  explicit IMUState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->quaternion.begin(), this->quaternion.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->gyroscope.begin(), this->gyroscope.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->accelerometer.begin(), this->accelerometer.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->rpy.begin(), this->rpy.end(), 0.0f);
      this->temperature = 0;
    }
  }

  explicit IMUState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : quaternion(_alloc),
    gyroscope(_alloc),
    accelerometer(_alloc),
    rpy(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->quaternion.begin(), this->quaternion.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->gyroscope.begin(), this->gyroscope.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->accelerometer.begin(), this->accelerometer.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->rpy.begin(), this->rpy.end(), 0.0f);
      this->temperature = 0;
    }
  }

  // field types and members
  using _quaternion_type =
    std::array<float, 4>;
  _quaternion_type quaternion;
  using _gyroscope_type =
    std::array<float, 3>;
  _gyroscope_type gyroscope;
  using _accelerometer_type =
    std::array<float, 3>;
  _accelerometer_type accelerometer;
  using _rpy_type =
    std::array<float, 3>;
  _rpy_type rpy;
  using _temperature_type =
    int8_t;
  _temperature_type temperature;

  // setters for named parameter idiom
  Type & set__quaternion(
    const std::array<float, 4> & _arg)
  {
    this->quaternion = _arg;
    return *this;
  }
  Type & set__gyroscope(
    const std::array<float, 3> & _arg)
  {
    this->gyroscope = _arg;
    return *this;
  }
  Type & set__accelerometer(
    const std::array<float, 3> & _arg)
  {
    this->accelerometer = _arg;
    return *this;
  }
  Type & set__rpy(
    const std::array<float, 3> & _arg)
  {
    this->rpy = _arg;
    return *this;
  }
  Type & set__temperature(
    const int8_t & _arg)
  {
    this->temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::IMUState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::IMUState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::IMUState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::IMUState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::IMUState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::IMUState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::IMUState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::IMUState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::IMUState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::IMUState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__IMUState
    std::shared_ptr<unitree_go::msg::IMUState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__IMUState
    std::shared_ptr<unitree_go::msg::IMUState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IMUState_ & other) const
  {
    if (this->quaternion != other.quaternion) {
      return false;
    }
    if (this->gyroscope != other.gyroscope) {
      return false;
    }
    if (this->accelerometer != other.accelerometer) {
      return false;
    }
    if (this->rpy != other.rpy) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const IMUState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IMUState_

// alias to use template instance with default allocator
using IMUState =
  unitree_go::msg::IMUState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__IMU_STATE__STRUCT_HPP_
