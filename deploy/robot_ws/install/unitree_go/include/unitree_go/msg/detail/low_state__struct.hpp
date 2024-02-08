// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__LOW_STATE__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__LOW_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'imu_state'
#include "unitree_go/msg/detail/imu_state__struct.hpp"
// Member 'motor_state'
#include "unitree_go/msg/detail/motor_state__struct.hpp"
// Member 'bms_state'
#include "unitree_go/msg/detail/bms_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__LowState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__LowState __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LowState_
{
  using Type = LowState_<ContainerAllocator>;

  explicit LowState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : imu_state(_init),
    bms_state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->head.begin(), this->head.end(), 0);
      this->level_flag = 0;
      this->frame_reserve = 0;
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->sn.begin(), this->sn.end(), 0ul);
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->version.begin(), this->version.end(), 0ul);
      this->bandwidth = 0;
      this->motor_state.fill(unitree_go::msg::MotorState_<ContainerAllocator>{_init});
      std::fill<typename std::array<int16_t, 4>::iterator, int16_t>(this->foot_force.begin(), this->foot_force.end(), 0);
      std::fill<typename std::array<int16_t, 4>::iterator, int16_t>(this->foot_force_est.begin(), this->foot_force_est.end(), 0);
      this->tick = 0ul;
      std::fill<typename std::array<uint8_t, 40>::iterator, uint8_t>(this->wireless_remote.begin(), this->wireless_remote.end(), 0);
      this->bit_flag = 0;
      this->adc_reel = 0.0f;
      this->temperature_ntc1 = 0;
      this->temperature_ntc2 = 0;
      this->power_v = 0.0f;
      this->power_a = 0.0f;
      std::fill<typename std::array<uint16_t, 4>::iterator, uint16_t>(this->fan_frequency.begin(), this->fan_frequency.end(), 0);
      this->reserve = 0ul;
      this->crc = 0ul;
    }
  }

  explicit LowState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : head(_alloc),
    sn(_alloc),
    version(_alloc),
    imu_state(_alloc, _init),
    motor_state(_alloc),
    bms_state(_alloc, _init),
    foot_force(_alloc),
    foot_force_est(_alloc),
    wireless_remote(_alloc),
    fan_frequency(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->head.begin(), this->head.end(), 0);
      this->level_flag = 0;
      this->frame_reserve = 0;
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->sn.begin(), this->sn.end(), 0ul);
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->version.begin(), this->version.end(), 0ul);
      this->bandwidth = 0;
      this->motor_state.fill(unitree_go::msg::MotorState_<ContainerAllocator>{_alloc, _init});
      std::fill<typename std::array<int16_t, 4>::iterator, int16_t>(this->foot_force.begin(), this->foot_force.end(), 0);
      std::fill<typename std::array<int16_t, 4>::iterator, int16_t>(this->foot_force_est.begin(), this->foot_force_est.end(), 0);
      this->tick = 0ul;
      std::fill<typename std::array<uint8_t, 40>::iterator, uint8_t>(this->wireless_remote.begin(), this->wireless_remote.end(), 0);
      this->bit_flag = 0;
      this->adc_reel = 0.0f;
      this->temperature_ntc1 = 0;
      this->temperature_ntc2 = 0;
      this->power_v = 0.0f;
      this->power_a = 0.0f;
      std::fill<typename std::array<uint16_t, 4>::iterator, uint16_t>(this->fan_frequency.begin(), this->fan_frequency.end(), 0);
      this->reserve = 0ul;
      this->crc = 0ul;
    }
  }

  // field types and members
  using _head_type =
    std::array<uint8_t, 2>;
  _head_type head;
  using _level_flag_type =
    uint8_t;
  _level_flag_type level_flag;
  using _frame_reserve_type =
    uint8_t;
  _frame_reserve_type frame_reserve;
  using _sn_type =
    std::array<uint32_t, 2>;
  _sn_type sn;
  using _version_type =
    std::array<uint32_t, 2>;
  _version_type version;
  using _bandwidth_type =
    uint16_t;
  _bandwidth_type bandwidth;
  using _imu_state_type =
    unitree_go::msg::IMUState_<ContainerAllocator>;
  _imu_state_type imu_state;
  using _motor_state_type =
    std::array<unitree_go::msg::MotorState_<ContainerAllocator>, 20>;
  _motor_state_type motor_state;
  using _bms_state_type =
    unitree_go::msg::BmsState_<ContainerAllocator>;
  _bms_state_type bms_state;
  using _foot_force_type =
    std::array<int16_t, 4>;
  _foot_force_type foot_force;
  using _foot_force_est_type =
    std::array<int16_t, 4>;
  _foot_force_est_type foot_force_est;
  using _tick_type =
    uint32_t;
  _tick_type tick;
  using _wireless_remote_type =
    std::array<uint8_t, 40>;
  _wireless_remote_type wireless_remote;
  using _bit_flag_type =
    uint8_t;
  _bit_flag_type bit_flag;
  using _adc_reel_type =
    float;
  _adc_reel_type adc_reel;
  using _temperature_ntc1_type =
    int8_t;
  _temperature_ntc1_type temperature_ntc1;
  using _temperature_ntc2_type =
    int8_t;
  _temperature_ntc2_type temperature_ntc2;
  using _power_v_type =
    float;
  _power_v_type power_v;
  using _power_a_type =
    float;
  _power_a_type power_a;
  using _fan_frequency_type =
    std::array<uint16_t, 4>;
  _fan_frequency_type fan_frequency;
  using _reserve_type =
    uint32_t;
  _reserve_type reserve;
  using _crc_type =
    uint32_t;
  _crc_type crc;

  // setters for named parameter idiom
  Type & set__head(
    const std::array<uint8_t, 2> & _arg)
  {
    this->head = _arg;
    return *this;
  }
  Type & set__level_flag(
    const uint8_t & _arg)
  {
    this->level_flag = _arg;
    return *this;
  }
  Type & set__frame_reserve(
    const uint8_t & _arg)
  {
    this->frame_reserve = _arg;
    return *this;
  }
  Type & set__sn(
    const std::array<uint32_t, 2> & _arg)
  {
    this->sn = _arg;
    return *this;
  }
  Type & set__version(
    const std::array<uint32_t, 2> & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__bandwidth(
    const uint16_t & _arg)
  {
    this->bandwidth = _arg;
    return *this;
  }
  Type & set__imu_state(
    const unitree_go::msg::IMUState_<ContainerAllocator> & _arg)
  {
    this->imu_state = _arg;
    return *this;
  }
  Type & set__motor_state(
    const std::array<unitree_go::msg::MotorState_<ContainerAllocator>, 20> & _arg)
  {
    this->motor_state = _arg;
    return *this;
  }
  Type & set__bms_state(
    const unitree_go::msg::BmsState_<ContainerAllocator> & _arg)
  {
    this->bms_state = _arg;
    return *this;
  }
  Type & set__foot_force(
    const std::array<int16_t, 4> & _arg)
  {
    this->foot_force = _arg;
    return *this;
  }
  Type & set__foot_force_est(
    const std::array<int16_t, 4> & _arg)
  {
    this->foot_force_est = _arg;
    return *this;
  }
  Type & set__tick(
    const uint32_t & _arg)
  {
    this->tick = _arg;
    return *this;
  }
  Type & set__wireless_remote(
    const std::array<uint8_t, 40> & _arg)
  {
    this->wireless_remote = _arg;
    return *this;
  }
  Type & set__bit_flag(
    const uint8_t & _arg)
  {
    this->bit_flag = _arg;
    return *this;
  }
  Type & set__adc_reel(
    const float & _arg)
  {
    this->adc_reel = _arg;
    return *this;
  }
  Type & set__temperature_ntc1(
    const int8_t & _arg)
  {
    this->temperature_ntc1 = _arg;
    return *this;
  }
  Type & set__temperature_ntc2(
    const int8_t & _arg)
  {
    this->temperature_ntc2 = _arg;
    return *this;
  }
  Type & set__power_v(
    const float & _arg)
  {
    this->power_v = _arg;
    return *this;
  }
  Type & set__power_a(
    const float & _arg)
  {
    this->power_a = _arg;
    return *this;
  }
  Type & set__fan_frequency(
    const std::array<uint16_t, 4> & _arg)
  {
    this->fan_frequency = _arg;
    return *this;
  }
  Type & set__reserve(
    const uint32_t & _arg)
  {
    this->reserve = _arg;
    return *this;
  }
  Type & set__crc(
    const uint32_t & _arg)
  {
    this->crc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::LowState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::LowState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::LowState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::LowState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::LowState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::LowState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::LowState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::LowState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::LowState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::LowState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__LowState
    std::shared_ptr<unitree_go::msg::LowState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__LowState
    std::shared_ptr<unitree_go::msg::LowState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LowState_ & other) const
  {
    if (this->head != other.head) {
      return false;
    }
    if (this->level_flag != other.level_flag) {
      return false;
    }
    if (this->frame_reserve != other.frame_reserve) {
      return false;
    }
    if (this->sn != other.sn) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->bandwidth != other.bandwidth) {
      return false;
    }
    if (this->imu_state != other.imu_state) {
      return false;
    }
    if (this->motor_state != other.motor_state) {
      return false;
    }
    if (this->bms_state != other.bms_state) {
      return false;
    }
    if (this->foot_force != other.foot_force) {
      return false;
    }
    if (this->foot_force_est != other.foot_force_est) {
      return false;
    }
    if (this->tick != other.tick) {
      return false;
    }
    if (this->wireless_remote != other.wireless_remote) {
      return false;
    }
    if (this->bit_flag != other.bit_flag) {
      return false;
    }
    if (this->adc_reel != other.adc_reel) {
      return false;
    }
    if (this->temperature_ntc1 != other.temperature_ntc1) {
      return false;
    }
    if (this->temperature_ntc2 != other.temperature_ntc2) {
      return false;
    }
    if (this->power_v != other.power_v) {
      return false;
    }
    if (this->power_a != other.power_a) {
      return false;
    }
    if (this->fan_frequency != other.fan_frequency) {
      return false;
    }
    if (this->reserve != other.reserve) {
      return false;
    }
    if (this->crc != other.crc) {
      return false;
    }
    return true;
  }
  bool operator!=(const LowState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LowState_

// alias to use template instance with default allocator
using LowState =
  unitree_go::msg::LowState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__LOW_STATE__STRUCT_HPP_
