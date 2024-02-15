"""
  Generated by Eclipse Cyclone DDS idlc Python Backend
  Cyclone DDS IDL version: v0.11.0
  Module: msgs
  IDL file: Imu.idl

"""

from enum import auto
from typing import TYPE_CHECKING, Optional
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

# root module import for resolving types
import msgs


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Imu(idl.IdlStruct, typename="msgs.Imu"):
    q: types.array[types.float32, 4]
    gyro: types.array[types.float32, 3]
    accel: types.array[types.float32, 3]
    rpy: types.array[types.float32, 3]
    tmp: types.uint8

