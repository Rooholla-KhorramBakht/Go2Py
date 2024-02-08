// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_go:msg/SportModeState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__TRAITS_HPP_
#define UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__TRAITS_HPP_

#include "unitree_go/msg/detail/sport_mode_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'stamp'
#include "unitree_go/msg/detail/time_spec__traits.hpp"
// Member 'imu_state'
#include "unitree_go/msg/detail/imu_state__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_go::msg::SportModeState>()
{
  return "unitree_go::msg::SportModeState";
}

template<>
inline const char * name<unitree_go::msg::SportModeState>()
{
  return "unitree_go/msg/SportModeState";
}

template<>
struct has_fixed_size<unitree_go::msg::SportModeState>
  : std::integral_constant<bool, has_fixed_size<unitree_go::msg::IMUState>::value && has_fixed_size<unitree_go::msg::TimeSpec>::value> {};

template<>
struct has_bounded_size<unitree_go::msg::SportModeState>
  : std::integral_constant<bool, has_bounded_size<unitree_go::msg::IMUState>::value && has_bounded_size<unitree_go::msg::TimeSpec>::value> {};

template<>
struct is_message<unitree_go::msg::SportModeState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__TRAITS_HPP_
