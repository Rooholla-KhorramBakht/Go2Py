/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: LowCmd.h
  Source: /home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/communication/msgs/idl/LowCmd.idl
  Cyclone DDS: V0.11.0

*****************************************************************/
#ifndef DDSC_LOWCMD_H
#define DDSC_LOWCMD_H

#include "dds/ddsc/dds_public_impl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct msgs_LowCmd
{
  float q[12];
  float dq[12];
  float tau_ff[12];
  float kp[12];
  float kv[12];
  uint8_t e_stop;
} msgs_LowCmd;

extern const dds_topic_descriptor_t msgs_LowCmd_desc;

#define msgs_LowCmd__alloc() \
((msgs_LowCmd*) dds_alloc (sizeof (msgs_LowCmd)));

#define msgs_LowCmd_free(d,o) \
dds_sample_free ((d), &msgs_LowCmd_desc, (o))

#ifdef __cplusplus
}
#endif

#endif /* DDSC_LOWCMD_H */
