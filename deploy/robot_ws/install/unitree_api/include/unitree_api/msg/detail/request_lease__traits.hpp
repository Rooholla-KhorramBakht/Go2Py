// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_api:msg/RequestLease.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_LEASE__TRAITS_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST_LEASE__TRAITS_HPP_

#include "unitree_api/msg/detail/request_lease__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_api::msg::RequestLease>()
{
  return "unitree_api::msg::RequestLease";
}

template<>
inline const char * name<unitree_api::msg::RequestLease>()
{
  return "unitree_api/msg/RequestLease";
}

template<>
struct has_fixed_size<unitree_api::msg::RequestLease>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<unitree_api::msg::RequestLease>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<unitree_api::msg::RequestLease>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_LEASE__TRAITS_HPP_
