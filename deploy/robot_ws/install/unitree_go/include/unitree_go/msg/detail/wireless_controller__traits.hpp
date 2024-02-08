// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_go:msg/WirelessController.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__WIRELESS_CONTROLLER__TRAITS_HPP_
#define UNITREE_GO__MSG__DETAIL__WIRELESS_CONTROLLER__TRAITS_HPP_

#include "unitree_go/msg/detail/wireless_controller__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_go::msg::WirelessController>()
{
  return "unitree_go::msg::WirelessController";
}

template<>
inline const char * name<unitree_go::msg::WirelessController>()
{
  return "unitree_go/msg/WirelessController";
}

template<>
struct has_fixed_size<unitree_go::msg::WirelessController>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<unitree_go::msg::WirelessController>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<unitree_go::msg::WirelessController>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_GO__MSG__DETAIL__WIRELESS_CONTROLLER__TRAITS_HPP_
