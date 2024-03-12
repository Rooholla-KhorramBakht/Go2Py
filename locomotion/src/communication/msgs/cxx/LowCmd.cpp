/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: /home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/communication/msgs/idl/LowCmd.idl
  Source: LowCmd.cpp
  Cyclone DDS: v0.11.0

*****************************************************************/
#include "LowCmd.hpp"

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<>
const propvec &get_type_props<::msgs::LowCmd>() {
  static std::mutex mtx;
  static propvec props;
  static std::atomic_bool initialized {false};
  key_endpoint keylist;
  if (initialized.load(std::memory_order_relaxed))
    return props;
  std::lock_guard<std::mutex> lock(mtx);
  if (initialized.load(std::memory_order_relaxed))
    return props;
  props.clear();

  props.push_back(entity_properties_t(0, 0, false, bit_bound::bb_unset, extensibility::ext_final));  //root
  props.push_back(entity_properties_t(1, 0, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::q
  props.push_back(entity_properties_t(1, 1, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::dq
  props.push_back(entity_properties_t(1, 2, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::tau_ff
  props.push_back(entity_properties_t(1, 3, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::kp
  props.push_back(entity_properties_t(1, 4, false, get_bit_bound<float>(), extensibility::ext_final, false));  //::kv
  props.push_back(entity_properties_t(1, 5, false, get_bit_bound<uint8_t>(), extensibility::ext_final, false));  //::e_stop

  entity_properties_t::finish(props, keylist);
  initialized.store(true, std::memory_order_release);
  return props;
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

