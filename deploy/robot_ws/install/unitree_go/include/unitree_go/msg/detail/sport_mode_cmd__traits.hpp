// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_go:msg/SportModeCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__TRAITS_HPP_
#define UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__TRAITS_HPP_

#include "unitree_go/msg/detail/sport_mode_cmd__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'bms_cmd'
#include "unitree_go/msg/detail/bms_cmd__traits.hpp"
// Member 'path_point'
#include "unitree_go/msg/detail/path_point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_go::msg::SportModeCmd>()
{
  return "unitree_go::msg::SportModeCmd";
}

template<>
inline const char * name<unitree_go::msg::SportModeCmd>()
{
  return "unitree_go/msg/SportModeCmd";
}

template<>
struct has_fixed_size<unitree_go::msg::SportModeCmd>
  : std::integral_constant<bool, has_fixed_size<unitree_go::msg::BmsCmd>::value && has_fixed_size<unitree_go::msg::PathPoint>::value> {};

template<>
struct has_bounded_size<unitree_go::msg::SportModeCmd>
  : std::integral_constant<bool, has_bounded_size<unitree_go::msg::BmsCmd>::value && has_bounded_size<unitree_go::msg::PathPoint>::value> {};

template<>
struct is_message<unitree_go::msg::SportModeCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__TRAITS_HPP_
