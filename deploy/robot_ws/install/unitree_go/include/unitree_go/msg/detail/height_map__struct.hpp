// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/HeightMap.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__HeightMap __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__HeightMap __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HeightMap_
{
  using Type = HeightMap_<ContainerAllocator>;

  explicit HeightMap_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->stamp = 0.0;
      this->frame_id = "";
      this->resolution = 0.0f;
      this->width = 0ul;
      this->height = 0ul;
      std::fill<typename std::array<float, 2>::iterator, float>(this->origin.begin(), this->origin.end(), 0.0f);
    }
  }

  explicit HeightMap_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame_id(_alloc),
    origin(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->stamp = 0.0;
      this->frame_id = "";
      this->resolution = 0.0f;
      this->width = 0ul;
      this->height = 0ul;
      std::fill<typename std::array<float, 2>::iterator, float>(this->origin.begin(), this->origin.end(), 0.0f);
    }
  }

  // field types and members
  using _stamp_type =
    double;
  _stamp_type stamp;
  using _frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _frame_id_type frame_id;
  using _resolution_type =
    float;
  _resolution_type resolution;
  using _width_type =
    uint32_t;
  _width_type width;
  using _height_type =
    uint32_t;
  _height_type height;
  using _origin_type =
    std::array<float, 2>;
  _origin_type origin;
  using _data_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__stamp(
    const double & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__frame_id(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->frame_id = _arg;
    return *this;
  }
  Type & set__resolution(
    const float & _arg)
  {
    this->resolution = _arg;
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
  Type & set__origin(
    const std::array<float, 2> & _arg)
  {
    this->origin = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::HeightMap_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::HeightMap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::HeightMap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::HeightMap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::HeightMap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::HeightMap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::HeightMap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::HeightMap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::HeightMap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::HeightMap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__HeightMap
    std::shared_ptr<unitree_go::msg::HeightMap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__HeightMap
    std::shared_ptr<unitree_go::msg::HeightMap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HeightMap_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->frame_id != other.frame_id) {
      return false;
    }
    if (this->resolution != other.resolution) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->origin != other.origin) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const HeightMap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HeightMap_

// alias to use template instance with default allocator
using HeightMap =
  unitree_go::msg::HeightMap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__STRUCT_HPP_
