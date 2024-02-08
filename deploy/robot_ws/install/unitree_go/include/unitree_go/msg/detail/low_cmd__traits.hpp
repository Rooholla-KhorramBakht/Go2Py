// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_go:msg/LowCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__LOW_CMD__TRAITS_HPP_
#define UNITREE_GO__MSG__DETAIL__LOW_CMD__TRAITS_HPP_

#include "unitree_go/msg/detail/low_cmd__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'motor_cmd'
#include "unitree_go/msg/detail/motor_cmd__traits.hpp"
// Member 'bms_cmd'
#include "unitree_go/msg/detail/bms_cmd__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_go::msg::LowCmd>()
{
  return "unitree_go::msg::LowCmd";
}

template<>
inline const char * name<unitree_go::msg::LowCmd>()
{
  return "unitree_go/msg/LowCmd";
}

template<>
struct has_fixed_size<unitree_go::msg::LowCmd>
  : std::integral_constant<bool, has_fixed_size<unitree_go::msg::BmsCmd>::value && has_fixed_size<unitree_go::msg::MotorCmd>::value> {};

template<>
struct has_bounded_size<unitree_go::msg::LowCmd>
  : std::integral_constant<bool, has_bounded_size<unitree_go::msg::BmsCmd>::value && has_bounded_size<unitree_go::msg::MotorCmd>::value> {};

template<>
struct is_message<unitree_go::msg::LowCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_GO__MSG__DETAIL__LOW_CMD__TRAITS_HPP_
