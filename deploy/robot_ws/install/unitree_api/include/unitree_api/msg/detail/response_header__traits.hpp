// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_api:msg/ResponseHeader.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__TRAITS_HPP_
#define UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__TRAITS_HPP_

#include "unitree_api/msg/detail/response_header__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'identity'
#include "unitree_api/msg/detail/request_identity__traits.hpp"
// Member 'status'
#include "unitree_api/msg/detail/response_status__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_api::msg::ResponseHeader>()
{
  return "unitree_api::msg::ResponseHeader";
}

template<>
inline const char * name<unitree_api::msg::ResponseHeader>()
{
  return "unitree_api/msg/ResponseHeader";
}

template<>
struct has_fixed_size<unitree_api::msg::ResponseHeader>
  : std::integral_constant<bool, has_fixed_size<unitree_api::msg::RequestIdentity>::value && has_fixed_size<unitree_api::msg::ResponseStatus>::value> {};

template<>
struct has_bounded_size<unitree_api::msg::ResponseHeader>
  : std::integral_constant<bool, has_bounded_size<unitree_api::msg::RequestIdentity>::value && has_bounded_size<unitree_api::msg::ResponseStatus>::value> {};

template<>
struct is_message<unitree_api::msg::ResponseHeader>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__TRAITS_HPP_
