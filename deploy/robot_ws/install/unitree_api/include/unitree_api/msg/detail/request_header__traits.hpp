// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_api:msg/RequestHeader.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_HEADER__TRAITS_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST_HEADER__TRAITS_HPP_

#include "unitree_api/msg/detail/request_header__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'identity'
#include "unitree_api/msg/detail/request_identity__traits.hpp"
// Member 'lease'
#include "unitree_api/msg/detail/request_lease__traits.hpp"
// Member 'policy'
#include "unitree_api/msg/detail/request_policy__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_api::msg::RequestHeader>()
{
  return "unitree_api::msg::RequestHeader";
}

template<>
inline const char * name<unitree_api::msg::RequestHeader>()
{
  return "unitree_api/msg/RequestHeader";
}

template<>
struct has_fixed_size<unitree_api::msg::RequestHeader>
  : std::integral_constant<bool, has_fixed_size<unitree_api::msg::RequestIdentity>::value && has_fixed_size<unitree_api::msg::RequestLease>::value && has_fixed_size<unitree_api::msg::RequestPolicy>::value> {};

template<>
struct has_bounded_size<unitree_api::msg::RequestHeader>
  : std::integral_constant<bool, has_bounded_size<unitree_api::msg::RequestIdentity>::value && has_bounded_size<unitree_api::msg::RequestLease>::value && has_bounded_size<unitree_api::msg::RequestPolicy>::value> {};

template<>
struct is_message<unitree_api::msg::RequestHeader>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_HEADER__TRAITS_HPP_
