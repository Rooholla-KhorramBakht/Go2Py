/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: /home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/communication/msgs/idl/LowCmd.idl
  Source: LowCmd.hpp
  Cyclone DDS: v0.11.0

*****************************************************************/
#ifndef DDSCXX_LOWCMD_HPP_34F396B86CEE831A5ED93F15EB80FE8C
#define DDSCXX_LOWCMD_HPP_34F396B86CEE831A5ED93F15EB80FE8C

#include <utility>
#include <cstdint>
#include <array>

namespace msgs
{
class LowCmd
{
private:
 std::array<float, 12> q_ = { };
 std::array<float, 12> dq_ = { };
 std::array<float, 12> tau_ff_ = { };
 std::array<float, 12> kp_ = { };
 std::array<float, 12> kv_ = { };
 uint8_t e_stop_ = 0;

public:
  LowCmd() = default;

  explicit LowCmd(
    const std::array<float, 12>& q,
    const std::array<float, 12>& dq,
    const std::array<float, 12>& tau_ff,
    const std::array<float, 12>& kp,
    const std::array<float, 12>& kv,
    uint8_t e_stop) :
    q_(q),
    dq_(dq),
    tau_ff_(tau_ff),
    kp_(kp),
    kv_(kv),
    e_stop_(e_stop) { }

  const std::array<float, 12>& q() const { return this->q_; }
  std::array<float, 12>& q() { return this->q_; }
  void q(const std::array<float, 12>& _val_) { this->q_ = _val_; }
  void q(std::array<float, 12>&& _val_) { this->q_ = std::move(_val_); }
  const std::array<float, 12>& dq() const { return this->dq_; }
  std::array<float, 12>& dq() { return this->dq_; }
  void dq(const std::array<float, 12>& _val_) { this->dq_ = _val_; }
  void dq(std::array<float, 12>&& _val_) { this->dq_ = std::move(_val_); }
  const std::array<float, 12>& tau_ff() const { return this->tau_ff_; }
  std::array<float, 12>& tau_ff() { return this->tau_ff_; }
  void tau_ff(const std::array<float, 12>& _val_) { this->tau_ff_ = _val_; }
  void tau_ff(std::array<float, 12>&& _val_) { this->tau_ff_ = std::move(_val_); }
  const std::array<float, 12>& kp() const { return this->kp_; }
  std::array<float, 12>& kp() { return this->kp_; }
  void kp(const std::array<float, 12>& _val_) { this->kp_ = _val_; }
  void kp(std::array<float, 12>&& _val_) { this->kp_ = std::move(_val_); }
  const std::array<float, 12>& kv() const { return this->kv_; }
  std::array<float, 12>& kv() { return this->kv_; }
  void kv(const std::array<float, 12>& _val_) { this->kv_ = _val_; }
  void kv(std::array<float, 12>&& _val_) { this->kv_ = std::move(_val_); }
  uint8_t e_stop() const { return this->e_stop_; }
  uint8_t& e_stop() { return this->e_stop_; }
  void e_stop(uint8_t _val_) { this->e_stop_ = _val_; }

  bool operator==(const LowCmd& _other) const
  {
    (void) _other;
    return q_ == _other.q_ &&
      dq_ == _other.dq_ &&
      tau_ff_ == _other.tau_ff_ &&
      kp_ == _other.kp_ &&
      kv_ == _other.kv_ &&
      e_stop_ == _other.e_stop_;
  }

  bool operator!=(const LowCmd& _other) const
  {
    return !(*this == _other);
  }

};

}

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <> constexpr const char* TopicTraits<::msgs::LowCmd>::getTypeName()
{
  return "msgs::LowCmd";
}

template <> constexpr bool TopicTraits<::msgs::LowCmd>::isKeyless()
{
  return true;
}

#ifdef DDSCXX_HAS_TYPELIB
template<> constexpr unsigned int TopicTraits<::msgs::LowCmd>::type_map_blob_sz() { return 518; }
template<> constexpr unsigned int TopicTraits<::msgs::LowCmd>::type_info_blob_sz() { return 100; }
template<> inline const uint8_t * TopicTraits<::msgs::LowCmd>::type_map_blob() {
  alignas(4) static const uint8_t blob[] = {
 0xc7,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0xf1,  0x99,  0xbb,  0x1c,  0xa1,  0x69,  0x8f,  0x02, 
 0x5f,  0xc8,  0x64,  0x19,  0x21,  0x0b,  0x6a,  0x00,  0xaf,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x9f,  0x00,  0x00,  0x00,  0x06,  0x00,  0x00,  0x00, 
 0x16,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x0c,  0x09,  0x76,  0x94,  0xf4,  0xa6,  0x00,  0x00,  0x16,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x0c,  0x09,  0x47,  0xbc,  0xdc,  0xd7,  0x00,  0x00,  0x16,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x0c,  0x09,  0x16,  0xfc, 
 0x75,  0xc7,  0x00,  0x00,  0x16,  0x00,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x0c,  0x09,  0x26,  0xb5,  0x68,  0xe4,  0x00,  0x00, 
 0x16,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x0c,  0x09,  0x82,  0xd0,  0x91,  0x47,  0x00,  0x00,  0x0b,  0x00,  0x00,  0x00, 
 0x05,  0x00,  0x00,  0x00,  0x01,  0x00,  0x02,  0x41,  0x60,  0xe3,  0x55,  0x00,  0x0d,  0x01,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0xf2,  0x2b,  0x33,  0x3f,  0xba,  0x68,  0xf3,  0xc8,  0xf8,  0xfe,  0xcd,  0x5a, 
 0x4f,  0x81,  0xbf,  0x00,  0xf5,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00,  0x15,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x0d,  0x00,  0x00,  0x00,  0x6d,  0x73,  0x67,  0x73,  0x3a,  0x3a,  0x4c,  0x6f, 
 0x77,  0x43,  0x6d,  0x64,  0x00,  0x00,  0x00,  0x00,  0xd1,  0x00,  0x00,  0x00,  0x06,  0x00,  0x00,  0x00, 
 0x1c,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x0c,  0x09,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x71,  0x00,  0x00,  0x00, 
 0x1d,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x0c,  0x09,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0x64,  0x71,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x21,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x0c,  0x09,  0x00,  0x00,  0x07,  0x00,  0x00,  0x00, 
 0x74,  0x61,  0x75,  0x5f,  0x66,  0x66,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x1d,  0x00,  0x00,  0x00, 
 0x03,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x0c,  0x09,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0x6b,  0x70,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x1d,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00,  0x01,  0x00,  0x90,  0xf3,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x0c,  0x09,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0x6b,  0x76,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x15,  0x00,  0x00,  0x00,  0x05,  0x00,  0x00,  0x00,  0x01,  0x00,  0x02,  0x00, 
 0x07,  0x00,  0x00,  0x00,  0x65,  0x5f,  0x73,  0x74,  0x6f,  0x70,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x22,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0xf2,  0x2b,  0x33,  0x3f,  0xba,  0x68,  0xf3,  0xc8, 
 0xf8,  0xfe,  0xcd,  0x5a,  0x4f,  0x81,  0xbf,  0xf1,  0x99,  0xbb,  0x1c,  0xa1,  0x69,  0x8f,  0x02,  0x5f, 
 0xc8,  0x64,  0x19,  0x21,  0x0b,  0x6a, };
  return blob;
}
template<> inline const uint8_t * TopicTraits<::msgs::LowCmd>::type_info_blob() {
  alignas(4) static const uint8_t blob[] = {
 0x60,  0x00,  0x00,  0x00,  0x01,  0x10,  0x00,  0x40,  0x28,  0x00,  0x00,  0x00,  0x24,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf1,  0x99,  0xbb,  0x1c,  0xa1,  0x69,  0x8f,  0x02,  0x5f,  0xc8,  0x64,  0x19, 
 0x21,  0x0b,  0x6a,  0x00,  0xb3,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x02,  0x10,  0x00,  0x40,  0x28,  0x00,  0x00,  0x00,  0x24,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf2,  0x2b,  0x33,  0x3f,  0xba,  0x68,  0xf3,  0xc8,  0xf8,  0xfe,  0xcd,  0x5a, 
 0x4f,  0x81,  0xbf,  0x00,  0xf9,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00, };
  return blob;
}
#endif //DDSCXX_HAS_TYPELIB

} //namespace topic
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

namespace dds {
namespace topic {

template <>
struct topic_type_name<::msgs::LowCmd>
{
    static std::string value()
    {
      return org::eclipse::cyclonedds::topic::TopicTraits<::msgs::LowCmd>::getTypeName();
    }
};

}
}

REGISTER_TOPIC_TYPE(::msgs::LowCmd)

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<>
const propvec &get_type_props<::msgs::LowCmd>();

namespace {
  static const volatile propvec &properties___msgs__LowCmd = get_type_props<::msgs::LowCmd>();
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write(T& streamer, const ::msgs::LowCmd& instance, const entity_properties_t *props) {
  (void)instance;
  member_id_set member_ids;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!write(streamer, instance.q()[0], instance.q().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!write(streamer, instance.dq()[0], instance.dq().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!write(streamer, instance.tau_ff()[0], instance.tau_ff().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!write(streamer, instance.kp()[0], instance.kp().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!write(streamer, instance.kv()[0], instance.kv().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.e_stop()))
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props, member_ids);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool write(S& str, const ::msgs::LowCmd& instance, key_mode key) {
  const auto &props = get_type_props<::msgs::LowCmd>();
  str.set_mode(cdr_stream::stream_mode::write, key);
  return write(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read(T& streamer, ::msgs::LowCmd& instance, const entity_properties_t *props) {
  (void)instance;
  member_id_set member_ids;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!read(streamer, instance.q()[0], instance.q().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!read(streamer, instance.dq()[0], instance.dq().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!read(streamer, instance.tau_ff()[0], instance.tau_ff().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!read(streamer, instance.kp()[0], instance.kp().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!read(streamer, instance.kv()[0], instance.kv().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.e_stop()))
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props, member_ids);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool read(S& str, ::msgs::LowCmd& instance, key_mode key) {
  const auto &props = get_type_props<::msgs::LowCmd>();
  str.set_mode(cdr_stream::stream_mode::read, key);
  return read(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move(T& streamer, const ::msgs::LowCmd& instance, const entity_properties_t *props) {
  (void)instance;
  member_id_set member_ids;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!move(streamer, instance.q()[0], instance.q().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!move(streamer, instance.dq()[0], instance.dq().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!move(streamer, instance.tau_ff()[0], instance.tau_ff().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!move(streamer, instance.kp()[0], instance.kp().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!move(streamer, instance.kv()[0], instance.kv().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.e_stop()))
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props, member_ids);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool move(S& str, const ::msgs::LowCmd& instance, key_mode key) {
  const auto &props = get_type_props<::msgs::LowCmd>();
  str.set_mode(cdr_stream::stream_mode::move, key);
  return move(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max(T& streamer, const ::msgs::LowCmd& instance, const entity_properties_t *props) {
  (void)instance;
  member_id_set member_ids;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!max(streamer, instance.q()[0], instance.q().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!max(streamer, instance.dq()[0], instance.dq().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 2:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!max(streamer, instance.tau_ff()[0], instance.tau_ff().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 3:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!max(streamer, instance.kp()[0], instance.kp().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 4:
      if (!streamer.start_member(*prop))
        return false;
      if (!streamer.start_consecutive(true, true))
        return false;
      if (!max(streamer, instance.kv()[0], instance.kv().size()))
        return false;
      if (!streamer.finish_consecutive())
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
      case 5:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.e_stop()))
        return false;
      if (!streamer.finish_member(*prop, member_ids))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props, member_ids);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool max(S& str, const ::msgs::LowCmd& instance, key_mode key) {
  const auto &props = get_type_props<::msgs::LowCmd>();
  str.set_mode(cdr_stream::stream_mode::max, key);
  return max(str, instance, props.data()); 
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

#endif // DDSCXX_LOWCMD_HPP_34F396B86CEE831A5ED93F15EB80FE8C
