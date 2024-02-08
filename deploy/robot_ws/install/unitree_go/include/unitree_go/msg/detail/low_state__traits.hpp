// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_go:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__LOW_STATE__TRAITS_HPP_
#define UNITREE_GO__MSG__DETAIL__LOW_STATE__TRAITS_HPP_

#include "unitree_go/msg/detail/low_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'imu_state'
#include "unitree_go/msg/detail/imu_state__traits.hpp"
// Member 'motor_state'
#include "unitree_go/msg/detail/motor_state__traits.hpp"
// Member 'bms_state'
#include "unitree_go/msg/detail/bms_state__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_go::msg::LowState>()
{
  return "unitree_go::msg::LowState";
}

template<>
inline const char * name<unitree_go::msg::LowState>()
{
  return "unitree_go/msg/LowState";
}

template<>
struct has_fixed_size<unitree_go::msg::LowState>
  : std::integral_constant<bool, has_fixed_size<unitree_go::msg::BmsState>::value && has_fixed_size<unitree_go::msg::IMUState>::value && has_fixed_size<unitree_go::msg::MotorState>::value> {};

template<>
struct has_bounded_size<unitree_go::msg::LowState>
  : std::integral_constant<bool, has_bounded_size<unitree_go::msg::BmsState>::value && has_bounded_size<unitree_go::msg::IMUState>::value && has_bounded_size<unitree_go::msg::MotorState>::value> {};

template<>
struct is_message<unitree_go::msg::LowState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_GO__MSG__DETAIL__LOW_STATE__TRAITS_HPP_
