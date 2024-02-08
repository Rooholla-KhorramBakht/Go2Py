// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_go:msg/Res.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__RES__TRAITS_HPP_
#define UNITREE_GO__MSG__DETAIL__RES__TRAITS_HPP_

#include "unitree_go/msg/detail/res__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_go::msg::Res>()
{
  return "unitree_go::msg::Res";
}

template<>
inline const char * name<unitree_go::msg::Res>()
{
  return "unitree_go/msg/Res";
}

template<>
struct has_fixed_size<unitree_go::msg::Res>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<unitree_go::msg::Res>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<unitree_go::msg::Res>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_GO__MSG__DETAIL__RES__TRAITS_HPP_
