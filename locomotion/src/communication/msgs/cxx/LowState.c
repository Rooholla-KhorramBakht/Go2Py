/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: LowState.c
  Source: /home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/communication/msgs/idl/LowState.idl
  Cyclone DDS: V0.11.0

*****************************************************************/
#include "LowState.h"

static const uint32_t msgs_LowState_ops [] =
{
  /* LowState */
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, q), 12u,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, dq), 12u,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, ddq), 12u,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, tau_est), 12u,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, tmp), 12u,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, contact), 4u,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, quat), 4u,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, gyro), 3u,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, accel), 3u,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, rpy), 3u,
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (msgs_LowState, imu_tmp),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, voltage),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (msgs_LowState, current),
  DDS_OP_RTS
};

/* Type Information:
  [MINIMAL eca861d2d70da15bdc1055b8dc20] (#deps: 0)
  [COMPLETE f70bf47bfb17eacecb7f4cbc814c] (#deps: 0)
*/
#define TYPE_INFO_CDR_msgs_LowState (unsigned char []){ \
  0x60, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x40, 0x28, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, \
  0x14, 0x00, 0x00, 0x00, 0xf1, 0xec, 0xa8, 0x61, 0xd2, 0xd7, 0x0d, 0xa1, 0x5b, 0xdc, 0x10, 0x55, \
  0xb8, 0xdc, 0x20, 0x00, 0x5f, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x40, 0x28, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, \
  0x14, 0x00, 0x00, 0x00, 0xf2, 0xf7, 0x0b, 0xf4, 0x7b, 0xfb, 0x17, 0xea, 0xce, 0xcb, 0x7f, 0x4c, \
  0xbc, 0x81, 0x4c, 0x00, 0xea, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00\
}
#define TYPE_INFO_CDR_SZ_msgs_LowState 100u
#define TYPE_MAP_CDR_msgs_LowState (unsigned char []){ \
  0x73, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf1, 0xec, 0xa8, 0x61, 0xd2, 0xd7, 0x0d, 0xa1, \
  0x5b, 0xdc, 0x10, 0x55, 0xb8, 0xdc, 0x20, 0x00, 0x5b, 0x01, 0x00, 0x00, 0xf1, 0x51, 0x01, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4b, 0x01, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, \
  0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x0c, 0x09, 0x76, 0x94, 0xf4, 0xa6, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
  0x0c, 0x09, 0x47, 0xbc, 0xdc, 0xd7, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0c, 0x09, 0xe9, 0x16, \
  0x89, 0x09, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, \
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0c, 0x09, 0x8a, 0xf7, 0xae, 0xdf, 0x00, 0x00, \
  0x16, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x0c, 0x09, 0xfa, 0x81, 0x6e, 0xdb, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, \
  0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
  0x04, 0x09, 0x2f, 0x8a, 0x6b, 0xf3, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x09, 0x21, 0xd7, \
  0xdc, 0x6a, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, \
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x09, 0x41, 0xe1, 0xdb, 0x58, 0x00, 0x00, \
  0x16, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x03, 0x09, 0x98, 0x30, 0x99, 0x65, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, \
  0x09, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
  0x03, 0x09, 0xb0, 0x7d, 0x92, 0xed, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x02, 0x04, 0x25, 0x70, 0x6e, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x09, 0xe4, 0x37, 0xba, 0x43, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x09, 0x43, 0xb5, 0xc9, 0x17, 0x00, 0xfe, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
  0xf2, 0xf7, 0x0b, 0xf4, 0x7b, 0xfb, 0x17, 0xea, 0xce, 0xcb, 0x7f, 0x4c, 0xbc, 0x81, 0x4c, 0x00, \
  0xe6, 0x01, 0x00, 0x00, 0xf2, 0x51, 0x01, 0x00, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x0f, 0x00, 0x00, 0x00, 0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x4c, 0x6f, 0x77, 0x53, 0x74, 0x61, \
  0x74, 0x65, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
  0x0c, 0x09, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x71, 0x00, 0x00, 0x00, 0x1d, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
  0x0c, 0x09, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x64, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x1e, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x0c, 0x09, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x64, 0x64, 0x71, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, \
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0c, 0x09, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, \
  0x74, 0x61, 0x75, 0x5f, 0x65, 0x73, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, \
  0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
  0x0c, 0x09, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x74, 0x6d, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x22, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x04, 0x09, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x63, 0x6f, 0x6e, 0x74, \
  0x61, 0x63, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x09, 0x00, 0x00, \
  0x05, 0x00, 0x00, 0x00, 0x71, 0x75, 0x61, 0x74, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, \
  0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
  0x03, 0x09, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x67, 0x79, 0x72, 0x6f, 0x00, 0x00, 0x00, 0x00, \
  0x20, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, 0x01, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x03, 0x09, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x61, 0x63, 0x63, 0x65, \
  0x6c, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x01, 0x00, 0x90, 0xf3, \
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x09, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, \
  0x72, 0x70, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x02, 0x00, 0x08, 0x00, 0x00, 0x00, 0x69, 0x6d, 0x75, 0x5f, 0x74, 0x6d, 0x70, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x00, \
  0x08, 0x00, 0x00, 0x00, 0x76, 0x6f, 0x6c, 0x74, 0x61, 0x67, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x16, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x00, 0x08, 0x00, 0x00, 0x00, \
  0x63, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0xf2, 0xf7, 0x0b, 0xf4, 0x7b, 0xfb, 0x17, 0xea, 0xce, 0xcb, 0x7f, 0x4c, \
  0xbc, 0x81, 0x4c, 0xf1, 0xec, 0xa8, 0x61, 0xd2, 0xd7, 0x0d, 0xa1, 0x5b, 0xdc, 0x10, 0x55, 0xb8, \
  0xdc, 0x20\
}
#define TYPE_MAP_CDR_SZ_msgs_LowState 930u
const dds_topic_descriptor_t msgs_LowState_desc =
{
  .m_size = sizeof (msgs_LowState),
  .m_align = dds_alignof (msgs_LowState),
  .m_flagset = DDS_TOPIC_FIXED_SIZE | DDS_TOPIC_XTYPES_METADATA,
  .m_nkeys = 0u,
  .m_typename = "msgs::LowState",
  .m_keys = NULL,
  .m_nops = 14,
  .m_ops = msgs_LowState_ops,
  .m_meta = "",
  .type_information = { .data = TYPE_INFO_CDR_msgs_LowState, .sz = TYPE_INFO_CDR_SZ_msgs_LowState },
  .type_mapping = { .data = TYPE_MAP_CDR_msgs_LowState, .sz = TYPE_MAP_CDR_SZ_msgs_LowState }
};

