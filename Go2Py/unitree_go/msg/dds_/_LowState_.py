"""
  Generated by Eclipse Cyclone DDS idlc Python Backend
  Cyclone DDS IDL version: v0.11.0
  Module: Go2Py.unitree_go.msg.dds_
  IDL file: LowState_.idl

"""

from enum import auto
from typing import TYPE_CHECKING, Optional
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

# root module import for resolving types
import Go2Py.unitree_go


@dataclass
@annotate.final
@annotate.autoid("sequential")
class LowState_(idl.IdlStruct, typename="Go2Py.unitree_go.msg.dds_.LowState_"):
    head: types.array[types.uint8, 2]
    level_flag: types.uint8
    frame_reserve: types.uint8
    sn: types.array[types.uint32, 2]
    version: types.array[types.uint32, 2]
    bandwidth: types.uint16
    imu_state: 'Go2Py.unitree_go.msg.dds_.IMUState_'
    motor_state: types.array['Go2Py.unitree_go.msg.dds_.MotorState_', 20]
    bms_state: 'Go2Py.unitree_go.msg.dds_.BmsState_'
    foot_force: types.array[types.int16, 4]
    foot_force_est: types.array[types.int16, 4]
    tick: types.uint32
    wireless_remote: types.array[types.uint8, 40]
    bit_flag: types.uint8
    adc_reel: types.float32
    temperature_ntc1: types.uint8
    temperature_ntc2: types.uint8
    power_v: types.float32
    power_a: types.float32
    fan_frequency: types.array[types.uint16, 4]
    reserve: types.uint32
    crc: types.uint32


