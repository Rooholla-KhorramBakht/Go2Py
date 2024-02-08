// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_api:msg/Response.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE__TRAITS_HPP_
#define UNITREE_API__MSG__DETAIL__RESPONSE__TRAITS_HPP_

#include "unitree_api/msg/detail/response__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "unitree_api/msg/detail/response_header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_api::msg::Response>()
{
  return "unitree_api::msg::Response";
}

template<>
inline const char * name<unitree_api::msg::Response>()
{
  return "unitree_api/msg/Response";
}

template<>
struct has_fixed_size<unitree_api::msg::Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<unitree_api::msg::Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<unitree_api::msg::Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE__TRAITS_HPP_
